// Copyright 2026 Soumic Sarkar
// SPDX-License-Identifier: Apache-2.0
//
// Autonomous frontier exploration.
//
// Reads the live occupancy grid built by SLAM (/map from RTAB-Map), finds reachable frontiers
// (FrontierSearch) and sends the best one as a nav2_msgs/action/NavigateToPose goal. Any
// server of that action works: Nav2's bt_navigator or origin_frontier_explore's
// custom_navigator.
//
// - A goal is replaced when its frontier has been explored (no frontier left near it).
// - Goals that fail, or where the robot makes no progress, are blacklisted.
// - When no frontiers remain, the robot optionally drives back to its start pose (which also
//   gives SLAM a loop closure) and exploration stops.
// - Look-around: the lidar sees 360 deg but the camera only ~60 deg ahead, so the robot spins
//   once in place at the start and after every `look_around_distance` metres travelled
//   (0 disables), letting the camera (YOLO perception) sweep every area it explores.
// Status is published on ~/status; frontiers are visualized on ~/frontiers.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "origin_frontier_explore/frontier_search.hpp"

namespace origin_frontier_explore
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class FrontierExplorer : public rclcpp::Node
{
public:
  enum class State { kWaiting, kExploring, kReturning, kDone, kStopped, kLookingAround };

  explicit FrontierExplorer(const rclcpp::NodeOptions & options)
  : Node("frontier_explorer", options)
  {
    global_frame_ = declare_parameter("global_frame", "map");
    robot_frame_ = declare_parameter("robot_frame", "base_link");
    action_name_ = declare_parameter("action_name", "navigate_to_pose");
    update_period_ = declare_parameter("update_period", 1.0);
    blacklist_radius_ = declare_parameter("blacklist_radius", 0.6);
    goal_explored_radius_ = declare_parameter("goal_explored_radius", 0.6);
    progress_timeout_ = declare_parameter("progress_timeout", 30.0);
    progress_distance_ = declare_parameter("progress_distance", 0.3);
    empty_checks_to_finish_ = declare_parameter("empty_checks_to_finish", 3);
    blacklist_retry_period_ = declare_parameter("blacklist_retry_period", 45.0);
    blacklist_retries_ = declare_parameter("blacklist_retries", 3);
    return_to_start_ = declare_parameter("return_to_start", true);
    look_around_distance_ = declare_parameter("look_around_distance", 4.0);
    look_around_speed_ = declare_parameter("look_around_speed", 0.6);
    const bool autostart = declare_parameter("autostart", true);

    params_.occupied_threshold = declare_parameter("occupied_threshold", 65);
    params_.robot_clearance = declare_parameter("robot_clearance", 0.40);
    params_.min_frontier_cells = declare_parameter("min_frontier_cells", 10);
    params_.size_weight = declare_parameter("size_weight", 1.0);
    params_.distance_weight = declare_parameter("distance_weight", 1.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/frontiers", 1);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "~/status", rclcpp::QoS(1).transient_local());
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {map_ = m;});

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start", [this](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr res) {
        blacklist_.clear();
        have_start_pose_ = false;
        have_spin_ref_ = false;
        setState(State::kWaiting);
        res->success = true;
      });
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", [this](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr res) {
        cancelGoal();
        if (state_ == State::kLookingAround) {
          cmd_pub_->publish(geometry_msgs::msg::Twist());
        }
        setState(State::kStopped);
        res->success = true;
      });

    setState(autostart ? State::kWaiting : State::kStopped);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(update_period_), [this]() {update();});
    spin_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {lookAroundStep();});
  }

private:
  // ---------------------------------------------------------------- main loop
  void update()
  {
    if (state_ == State::kStopped || state_ == State::kDone) {
      return;
    }
    if (!map_) {
      return;
    }
    double rx, ry;
    if (!robotPose(rx, ry)) {
      return;
    }
    if (!have_start_pose_) {
      start_x_ = rx;
      start_y_ = ry;
      have_start_pose_ = true;
    }
    if (state_ == State::kWaiting) {
      if (!client_->wait_for_action_server(std::chrono::seconds(0))) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "Waiting for action server '%s'", action_name_.c_str());
        return;
      }
      setState(State::kExploring);
    }
    if (state_ == State::kReturning || state_ == State::kLookingAround) {
      return;  // wait for the return goal's result / the spin to finish
    }
    if (look_around_distance_ > 0.0) {
      if (!have_spin_ref_) {
        startLookAround(rx, ry);  // at the start
        return;
      }
      if (std::hypot(rx - spin_ref_x_, ry - spin_ref_y_) >= look_around_distance_) {
        startLookAround(rx, ry);
        return;
      }
    }

    Grid grid;
    grid.width = static_cast<int>(map_->info.width);
    grid.height = static_cast<int>(map_->info.height);
    grid.resolution = map_->info.resolution;
    grid.origin_x = map_->info.origin.position.x;
    grid.origin_y = map_->info.origin.position.y;
    grid.data = map_->data;

    std::vector<Frontier> all = findFrontiers(grid, rx, ry, params_);
    std::vector<Frontier> frontiers;
    for (auto & f : all) {
      if (!isBlacklisted(f.goal_x, f.goal_y)) {
        frontiers.push_back(std::move(f));
      }
    }
    publishMarkers(frontiers);

    // Progress check on the active goal.
    if (goal_active_) {
      const double t = (now() - progress_time_).seconds();
      if (std::hypot(rx - progress_x_, ry - progress_y_) > progress_distance_) {
        progress_x_ = rx;
        progress_y_ = ry;
        progress_time_ = now();
      } else if (t > progress_timeout_) {
        RCLCPP_WARN(get_logger(), "No progress towards (%.2f, %.2f), blacklisting",
          goal_x_, goal_y_);
        blacklist_.emplace_back(goal_x_, goal_y_);
        cancelGoal();
        return;
      }
    }

    if (frontiers.empty()) {
      if (goal_active_) {
        return;
      }
      // Frontiers left, but all blacklisted (the robot could not reach them, e.g. held up by
      // people or a stop of the collision monitor): wait and retry them before giving up.
      if (!all.empty() && retry_rounds_ < blacklist_retries_) {
        if (!waiting_for_retry_) {
          waiting_for_retry_ = true;
          retry_time_ = now();
          RCLCPP_WARN(get_logger(), "%zu frontiers left but all blacklisted; retrying in %.0f s "
            "(round %d/%d)", all.size(), blacklist_retry_period_, retry_rounds_ + 1,
            blacklist_retries_);
        } else if ((now() - retry_time_).seconds() > blacklist_retry_period_) {
          blacklist_.clear();
          waiting_for_retry_ = false;
          ++retry_rounds_;
        }
        return;
      }
      if (++empty_checks_ >= empty_checks_to_finish_) {
        finishExploration(rx, ry);
      }
      return;
    }
    empty_checks_ = 0;
    waiting_for_retry_ = false;

    // Keep the current goal while its frontier still exists.
    if (goal_active_) {
      bool still_frontier = false;
      for (const auto & f : frontiers) {
        for (const auto & [cx, cy] : f.cells) {
          double wx, wy;
          grid.cellToWorld(cx, cy, wx, wy);
          if (std::hypot(wx - goal_x_, wy - goal_y_) < goal_explored_radius_) {
            still_frontier = true;
            break;
          }
        }
        if (still_frontier) {
          break;
        }
      }
      if (still_frontier) {
        return;
      }
      RCLCPP_INFO(get_logger(), "Frontier at (%.2f, %.2f) explored, choosing next",
        goal_x_, goal_y_);
      cancelGoal();
    }
    const auto & best = frontiers.front();
    RCLCPP_INFO(get_logger(),
      "Next frontier (%.2f, %.2f): length %.1f m, travel %.1f m (%zu candidates)",
      best.goal_x, best.goal_y, best.length, best.travel_distance, frontiers.size());
    sendGoal(best.goal_x, best.goal_y, rx, ry);
  }

  void finishExploration(double rx, double ry)
  {
    RCLCPP_INFO(get_logger(), "No frontiers left: exploration complete");
    if (return_to_start_ && std::hypot(rx - start_x_, ry - start_y_) > 0.5) {
      RCLCPP_INFO(get_logger(), "Returning to start (%.2f, %.2f)", start_x_, start_y_);
      setState(State::kReturning);
      sendGoal(start_x_, start_y_, rx, ry);
    } else {
      setState(State::kDone);
    }
  }

  // ---------------------------------------------------------------- look-around
  void startLookAround(double rx, double ry)
  {
    cancelGoal();
    have_spin_ref_ = true;
    spin_ref_x_ = rx;
    spin_ref_y_ = ry;
    double yaw;
    if (!robotYaw(yaw)) {
      return;
    }
    spin_last_yaw_ = yaw;
    spin_turned_ = 0.0;
    spin_start_ = now();
    setState(State::kLookingAround);
  }

  void lookAroundStep()
  {
    if (state_ != State::kLookingAround) {
      return;
    }
    double yaw;
    if (!robotYaw(yaw)) {
      return;
    }
    spin_turned_ += std::fabs(std::atan2(std::sin(yaw - spin_last_yaw_), std::cos(yaw - spin_last_yaw_)));
    spin_last_yaw_ = yaw;
    geometry_msgs::msg::Twist cmd;
    const double timeout = 2.0 * 2.0 * M_PI / look_around_speed_;
    if (spin_turned_ < 2.0 * M_PI && (now() - spin_start_).seconds() < timeout) {
      cmd.angular.z = look_around_speed_;
      cmd_pub_->publish(cmd);
      return;
    }
    cmd_pub_->publish(cmd);  // stop
    setState(State::kExploring);
  }

  bool robotYaw(double & yaw)
  {
    try {
      const auto t = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
      yaw = tf2::getYaw(t.transform.rotation);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  // ---------------------------------------------------------------- goals
  void sendGoal(double gx, double gy, double rx, double ry)
  {
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = global_frame_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = gx;
    goal.pose.pose.position.y = gy;
    tf2::Quaternion q;
    q.setRPY(0, 0, std::atan2(gy - ry, gx - rx));
    goal.pose.pose.orientation = tf2::toMsg(q);

    goal_x_ = gx;
    goal_y_ = gy;
    goal_active_ = true;
    progress_x_ = rx;
    progress_y_ = ry;
    progress_time_ = now();
    const uint64_t id = ++goal_seq_;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback = [this, id](ClientGoalHandle::SharedPtr handle) {
        if (id != goal_seq_) {
          return;
        }
        if (!handle) {
          // Typically the server is up but not active yet (Nav2 lifecycle): retry next update.
          RCLCPP_WARN(get_logger(), "Goal rejected by '%s', retrying", action_name_.c_str());
          goal_active_ = false;
          if (state_ == State::kReturning) {
            setState(State::kExploring);  // re-enters finishExploration once frontiers are empty
          }
          return;
        }
        goal_handle_ = handle;
      };
    opts.result_callback = [this, id](const ClientGoalHandle::WrappedResult & r) {
        if (id != goal_seq_) {
          return;  // result of a goal we already replaced
        }
        goal_active_ = false;
        goal_handle_.reset();
        if (state_ == State::kReturning) {
          RCLCPP_INFO(get_logger(), "Back at start: exploration finished");
          setState(State::kDone);
          return;
        }
        if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "Reached frontier (%.2f, %.2f)", goal_x_, goal_y_);
        } else if (r.code == rclcpp_action::ResultCode::ABORTED) {
          RCLCPP_WARN(get_logger(), "Navigation to (%.2f, %.2f) failed, blacklisting",
            goal_x_, goal_y_);
          blacklist_.emplace_back(goal_x_, goal_y_);
        }
      };
    client_->async_send_goal(goal, opts);
  }

  void cancelGoal()
  {
    ++goal_seq_;  // ignore callbacks of the cancelled goal
    if (goal_handle_) {
      client_->async_cancel_goal(goal_handle_);
    }
    goal_handle_.reset();
    goal_active_ = false;
  }

  bool isBlacklisted(double x, double y) const
  {
    for (const auto & [bx, by] : blacklist_) {
      if (std::hypot(x - bx, y - by) < blacklist_radius_) {
        return true;
      }
    }
    return false;
  }

  bool robotPose(double & x, double & y)
  {
    try {
      const auto t = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
      x = t.transform.translation.x;
      y = t.transform.translation.y;
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Robot pose: %s", e.what());
      return false;
    }
  }

  void setState(State s)
  {
    state_ = s;
    static const char * names[] = {
      "waiting", "exploring", "returning", "done", "stopped", "looking_around"};
    std_msgs::msg::String msg;
    msg.data = names[static_cast<int>(s)];
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "State: %s", msg.data.c_str());
  }

  // ---------------------------------------------------------------- visualization
  void publishMarkers(const std::vector<Frontier> & frontiers)
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    visualization_msgs::msg::Marker cells;
    cells.header.frame_id = global_frame_;
    cells.header.stamp = now();
    cells.ns = "frontier_cells";
    cells.type = visualization_msgs::msg::Marker::POINTS;
    cells.scale.x = cells.scale.y = map_->info.resolution;
    cells.color.r = 0.1f;
    cells.color.g = 0.6f;
    cells.color.b = 1.0f;
    cells.color.a = 1.0f;
    cells.pose.orientation.w = 1.0;

    int id = 0;
    for (size_t i = 0; i < frontiers.size(); ++i) {
      const auto & f = frontiers[i];
      for (const auto & [cx, cy] : f.cells) {
        geometry_msgs::msg::Point p;
        p.x = map_->info.origin.position.x + (cx + 0.5) * map_->info.resolution;
        p.y = map_->info.origin.position.y + (cy + 0.5) * map_->info.resolution;
        p.z = 0.05;
        cells.points.push_back(p);
      }
      visualization_msgs::msg::Marker goal;
      goal.header = cells.header;
      goal.ns = "frontier_goals";
      goal.id = id++;
      goal.type = visualization_msgs::msg::Marker::SPHERE;
      goal.pose.position.x = f.goal_x;
      goal.pose.position.y = f.goal_y;
      goal.pose.position.z = 0.1;
      goal.pose.orientation.w = 1.0;
      goal.scale.x = goal.scale.y = goal.scale.z = i == 0 ? 0.35 : 0.2;
      goal.color.r = i == 0 ? 0.0f : 1.0f;
      goal.color.g = i == 0 ? 1.0f : 0.5f;
      goal.color.a = 1.0f;
      arr.markers.push_back(goal);
    }
    arr.markers.push_back(cells);
    marker_pub_->publish(arr);
  }

  std::string global_frame_, robot_frame_, action_name_;
  double update_period_, blacklist_radius_, goal_explored_radius_;
  double progress_timeout_, progress_distance_;
  int empty_checks_to_finish_;
  double blacklist_retry_period_;
  int blacklist_retries_;
  int retry_rounds_{0};
  bool waiting_for_retry_{false};
  rclcpp::Time retry_time_;
  bool return_to_start_;
  double look_around_distance_, look_around_speed_;
  bool have_spin_ref_{false};
  double spin_ref_x_{0}, spin_ref_y_{0}, spin_last_yaw_{0}, spin_turned_{0};
  rclcpp::Time spin_start_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr spin_timer_;
  FrontierSearchParams params_;

  State state_{State::kWaiting};
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  std::vector<std::pair<double, double>> blacklist_;
  int empty_checks_{0};
  bool have_start_pose_{false};
  double start_x_{0}, start_y_{0};

  bool goal_active_{false};
  uint64_t goal_seq_{0};
  double goal_x_{0}, goal_y_{0};
  double progress_x_{0}, progress_y_{0};
  rclcpp::Time progress_time_;
  ClientGoalHandle::SharedPtr goal_handle_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace origin_frontier_explore

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<origin_frontier_explore::FrontierExplorer>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

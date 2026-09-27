// Copyright 2026 Soumic Sarkar
// SPDX-License-Identifier: Apache-2.0
//
// Custom navigation backend: a nav2_msgs/action/NavigateToPose server built from the
// GridPlanner (A* on /map with inflation) and the DwaController (local obstacle avoidance on
// the filtered lidar cloud). Drop-in alternative to Nav2 for the frontier explorer and for
// goals sent from RViz (/goal_pose).
//
// Obstacles are the lidar points between obstacle_min_z and obstacle_max_z (robot frame).
// They are remembered for memory_duration seconds in memory_frame (a smooth odometry frame),
// which covers the lidar's blind zone close to the ground next to the robot.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "origin_frontier_explore/dwa_controller.hpp"
#include "origin_frontier_explore/grid_planner.hpp"

namespace origin_frontier_explore
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

namespace
{
double normalizeAngle(double a) {return std::atan2(std::sin(a), std::cos(a));}

Grid gridFromMsg(const nav_msgs::msg::OccupancyGrid & m)
{
  Grid g;
  g.width = static_cast<int>(m.info.width);
  g.height = static_cast<int>(m.info.height);
  g.resolution = m.info.resolution;
  g.origin_x = m.info.origin.position.x;
  g.origin_y = m.info.origin.position.y;
  g.data = m.data;
  return g;
}
}  // namespace

class CustomNavigator : public rclcpp::Node
{
public:
  explicit CustomNavigator(const rclcpp::NodeOptions & options)
  : Node("custom_navigator", options)
  {
    global_frame_ = declare_parameter("global_frame", "map");
    robot_frame_ = declare_parameter("robot_frame", "base_link");
    memory_frame_ = declare_parameter("memory_frame", "icp_odom");
    control_rate_ = declare_parameter("control_rate", 10.0);
    replan_period_ = declare_parameter("replan_period", 1.0);
    goal_tolerance_ = declare_parameter("goal_tolerance", 0.3);
    yaw_tolerance_ = declare_parameter("yaw_tolerance", 0.2);
    align_goal_yaw_ = declare_parameter("align_goal_yaw", false);
    lookahead_ = declare_parameter("lookahead", 1.0);
    obstacle_min_z_ = declare_parameter("obstacle_min_z", 0.08);
    obstacle_max_z_ = declare_parameter("obstacle_max_z", 0.70);
    obstacle_range_ = declare_parameter("obstacle_range", 4.0);
    memory_duration_ = declare_parameter("memory_duration", 2.0);
    stuck_timeout_ = declare_parameter("stuck_timeout", 10.0);
    stuck_distance_ = declare_parameter("stuck_distance", 0.15);
    max_recoveries_ = declare_parameter("max_recoveries", 4);
    max_plan_failures_ = declare_parameter("max_plan_failures", 3);

    PlannerParams pp;
    pp.robot_radius = declare_parameter("planner.robot_radius", 0.40);
    pp.inflation_radius = declare_parameter("planner.inflation_radius", 1.0);
    pp.inflation_weight = declare_parameter("planner.inflation_weight", 4.0);
    pp.allow_unknown = declare_parameter("planner.allow_unknown", true);
    pp.unknown_cost = declare_parameter("planner.unknown_cost", 3.0);
    pp.occupied_threshold = declare_parameter("planner.occupied_threshold", 65);
    planner_ = GridPlanner(pp);

    DwaParams dp;
    dp.max_v = declare_parameter("dwa.max_v", 0.4);
    dp.max_w = declare_parameter("dwa.max_w", 1.0);
    dp.acc_v = declare_parameter("dwa.acc_v", 1.0);
    dp.acc_w = declare_parameter("dwa.acc_w", 2.5);
    dp.sim_time = declare_parameter("dwa.sim_time", 2.0);
    dp.half_length = declare_parameter("dwa.half_length", 0.33);
    dp.half_width = declare_parameter("dwa.half_width", 0.29);
    dp.margin = declare_parameter("dwa.margin", 0.06);
    dp.w_progress = declare_parameter("dwa.w_progress", 2.0);
    dp.w_heading = declare_parameter("dwa.w_heading", 0.8);
    dp.w_clearance = declare_parameter("dwa.w_clearance", 0.15);
    dp.w_speed = declare_parameter("dwa.w_speed", 0.6);
    dp.control_period = 1.0 / control_rate_;
    dwa_ = DwaController(dp);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("~/plan", 1);
    traj_pub_ = create_publisher<nav_msgs::msg::Path>("~/local_trajectory", 1);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
        planner_.setMap(gridFromMsg(*m));
        have_map_ = true;
      });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {onCloud(*m);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        cur_v_ = m->twist.twist.linear.x;
        cur_w_ = m->twist.twist.angular.z;
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 1, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        if (active_handle_ && active_handle_->is_active()) {
          finish(false, "preempted by /goal_pose");
        }
        startGoal(*m, nullptr);
      });

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this, "navigate_to_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal>) {
        return have_map_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
        rclcpp_action::GoalResponse::REJECT;
      },
      [](std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandle> handle) {
        if (active_) {
          finish(false, "preempted by a new goal");
        }
        startGoal(handle->get_goal()->pose, handle);
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_), [this]() {controlStep();});
    RCLCPP_INFO(get_logger(), "Custom navigator ready (A* + DWA)");
  }

private:
  // ---------------------------------------------------------------- obstacles
  void onCloud(const sensor_msgs::msg::PointCloud2 & in)
  {
    tf2::Transform base_from_sensor, mem_from_base;
    try {
      tf2::fromMsg(
        tf_buffer_->lookupTransform(robot_frame_, in.header.frame_id, tf2::TimePointZero).transform,
        base_from_sensor);
      tf2::fromMsg(
        tf_buffer_->lookupTransform(memory_frame_, robot_frame_, tf2::TimePointZero).transform,
        mem_from_base);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cloud TF: %s", e.what());
      return;
    }
    int ox = -1, oy = -1, oz = -1;
    for (const auto & f : in.fields) {
      if (f.name == "x") {ox = f.offset;}
      if (f.name == "y") {oy = f.offset;}
      if (f.name == "z") {oz = f.offset;}
    }
    if (ox < 0 || oy < 0 || oz < 0) {
      return;
    }

    // 2D voxel downsampling (5 cm) of points in the robot's height band.
    std::unordered_set<int64_t> seen;
    Scan scan;
    scan.stamp = now();
    const size_t n = static_cast<size_t>(in.width) * in.height;
    for (size_t i = 0; i < n; ++i) {
      const uint8_t * p = &in.data[i * in.point_step];
      float x, y, z;
      std::memcpy(&x, p + ox, 4);
      std::memcpy(&y, p + oy, 4);
      std::memcpy(&z, p + oz, 4);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const tf2::Vector3 b = base_from_sensor * tf2::Vector3(x, y, z);
      if (b.z() < obstacle_min_z_ || b.z() > obstacle_max_z_ ||
        std::hypot(b.x(), b.y()) > obstacle_range_)
      {
        continue;
      }
      const int64_t key = (static_cast<int64_t>(std::floor(b.x() / 0.05)) << 32) ^
        (static_cast<int64_t>(std::floor(b.y() / 0.05)) & 0xffffffff);
      if (!seen.insert(key).second) {
        continue;
      }
      const tf2::Vector3 m = mem_from_base * tf2::Vector3(b.x(), b.y(), 0.0);
      scan.points.emplace_back(m.x(), m.y());
    }
    scans_.push_back(std::move(scan));
    while (!scans_.empty() && (now() - scans_.front().stamp).seconds() > memory_duration_) {
      scans_.pop_front();
    }
  }

  bool obstaclesInRobotFrame(std::vector<std::pair<double, double>> & out)
  {
    tf2::Transform base_from_mem;
    try {
      tf2::fromMsg(
        tf_buffer_->lookupTransform(robot_frame_, memory_frame_, tf2::TimePointZero).transform,
        base_from_mem);
    } catch (const tf2::TransformException &) {
      return false;
    }
    out.clear();
    for (const auto & s : scans_) {
      for (const auto & [x, y] : s.points) {
        const tf2::Vector3 b = base_from_mem * tf2::Vector3(x, y, 0.0);
        out.emplace_back(b.x(), b.y());
      }
    }
    return true;
  }

  // ---------------------------------------------------------------- goals
  void startGoal(const geometry_msgs::msg::PoseStamped & goal, std::shared_ptr<GoalHandle> handle)
  {
    geometry_msgs::msg::PoseStamped g = goal;
    if (!g.header.frame_id.empty() && g.header.frame_id != global_frame_) {
      try {
        g = tf_buffer_->transform(g, global_frame_, tf2::durationFromSec(0.5));
      } catch (const tf2::TransformException & e) {
        active_handle_ = handle;
        active_ = true;
        finish(false, std::string("cannot transform goal: ") + e.what());
        return;
      }
    }
    goal_x_ = g.pose.position.x;
    goal_y_ = g.pose.position.y;
    goal_yaw_ = tf2::getYaw(g.pose.orientation);
    active_handle_ = handle;
    active_ = true;
    path_.clear();
    last_plan_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    plan_failures_ = 0;
    recoveries_ = 0;
    recovery_until_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    start_time_ = now();
    stuck_check_time_ = now();
    have_stuck_ref_ = false;
    RCLCPP_INFO(get_logger(), "New goal (%.2f, %.2f)", goal_x_, goal_y_);
  }

  void finish(bool success, const std::string & msg)
  {
    stop();
    if (active_handle_ && active_handle_->is_active()) {
      auto result = std::make_shared<NavigateToPose::Result>();
      result->error_code = success ? 0 : 1;
      result->error_msg = msg;
      if (success) {
        active_handle_->succeed(result);
      } else if (active_handle_->is_canceling()) {
        active_handle_->canceled(result);
      } else {
        active_handle_->abort(result);
      }
    }
    if (success) {
      RCLCPP_INFO(get_logger(), "Goal reached");
    } else {
      RCLCPP_WARN(get_logger(), "Goal ended: %s", msg.c_str());
    }
    active_ = false;
    active_handle_.reset();
    path_.clear();
  }

  void stop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  // ---------------------------------------------------------------- control loop
  void controlStep()
  {
    if (!active_) {
      return;
    }
    if (active_handle_ && active_handle_->is_canceling()) {
      finish(false, "canceled");
      return;
    }

    double rx, ry, ryaw;
    try {
      const auto t = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
      rx = t.transform.translation.x;
      ry = t.transform.translation.y;
      ryaw = tf2::getYaw(t.transform.rotation);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Robot pose: %s", e.what());
      stop();
      return;
    }
    const rclcpp::Time t_now = now();
    const double dist_goal = std::hypot(goal_x_ - rx, goal_y_ - ry);
    publishFeedback(rx, ry, ryaw, dist_goal);

    // Goal reached (optionally turn to the goal yaw).
    if (dist_goal < goal_tolerance_) {
      const double yaw_err = normalizeAngle(goal_yaw_ - ryaw);
      if (!align_goal_yaw_ || std::fabs(yaw_err) < yaw_tolerance_) {
        finish(true, "");
        return;
      }
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = std::clamp(1.5 * yaw_err, -dwa_.params().max_w, dwa_.params().max_w);
      cmd_pub_->publish(cmd);
      return;
    }

    // Recovery in progress: keep executing it.
    if (t_now < recovery_until_) {
      cmd_pub_->publish(recovery_cmd_);
      return;
    }

    // Global plan.
    if (path_.empty() || (t_now - last_plan_time_).seconds() > replan_period_) {
      Path p;
      if (planner_.plan(rx, ry, goal_x_, goal_y_, p)) {
        path_ = std::move(p);
        plan_failures_ = 0;
        publishPlan();
      } else if (++plan_failures_ >= max_plan_failures_) {
        finish(false, "no path to goal");
        return;
      }
      last_plan_time_ = t_now;
    }
    if (path_.empty()) {
      stop();
      return;
    }

    // Lookahead point on the path, expressed in the robot frame.
    size_t closest = 0;
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path_.size(); ++i) {
      const double d = std::hypot(path_[i].first - rx, path_[i].second - ry);
      if (d < best) {
        best = d;
        closest = i;
      }
    }
    size_t target = closest;
    while (target + 1 < path_.size() &&
      std::hypot(path_[target].first - rx, path_[target].second - ry) < lookahead_)
    {
      ++target;
    }
    const double gx = path_[target].first - rx;
    const double gy = path_[target].second - ry;
    const double tx = std::cos(ryaw) * gx + std::sin(ryaw) * gy;
    const double ty = -std::sin(ryaw) * gx + std::cos(ryaw) * gy;

    // Local obstacle avoidance.
    std::vector<std::pair<double, double>> obstacles;
    if (!obstaclesInRobotFrame(obstacles)) {
      stop();
      return;
    }
    dwa_.setObstacles(obstacles);
    DwaCommand cmd;
    if (!dwa_.compute(cur_v_, cur_w_, tx, ty, cmd)) {
      startRecovery(tx, ty, "no collision-free motion");
      return;
    }

    // Progress check.
    if (!have_stuck_ref_) {
      stuck_ref_x_ = rx;
      stuck_ref_y_ = ry;
      stuck_check_time_ = t_now;
      have_stuck_ref_ = true;
    } else if ((t_now - stuck_check_time_).seconds() > stuck_timeout_) {
      if (std::hypot(rx - stuck_ref_x_, ry - stuck_ref_y_) < stuck_distance_) {
        have_stuck_ref_ = false;
        startRecovery(tx, ty, "no progress");
        return;
      }
      have_stuck_ref_ = false;
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = cmd.v;
    twist.angular.z = cmd.w;
    cmd_pub_->publish(twist);
    publishTrajectory(cmd);
  }

  void startRecovery(double tx, double ty, const std::string & why)
  {
    if (++recoveries_ > max_recoveries_) {
      finish(false, "stuck: " + why);
      return;
    }
    recovery_cmd_ = geometry_msgs::msg::Twist();
    // Prefer backing up if the space behind is free, otherwise rotate towards the target.
    if (dwa_.clearance(-0.25, 0.0, 0.0) > 0.0) {
      recovery_cmd_.linear.x = -0.15;
      recovery_until_ = now() + rclcpp::Duration::from_seconds(1.5);
    } else {
      recovery_cmd_.angular.z = std::atan2(ty, tx) >= 0.0 ? 0.6 : -0.6;
      recovery_until_ = now() + rclcpp::Duration::from_seconds(1.5);
    }
    path_.clear();
    RCLCPP_WARN(get_logger(), "Recovery %d/%d (%s)", recoveries_, max_recoveries_, why.c_str());
  }

  // ---------------------------------------------------------------- output
  void publishFeedback(double rx, double ry, double ryaw, double dist_goal)
  {
    if (!active_handle_) {
      return;
    }
    auto fb = std::make_shared<NavigateToPose::Feedback>();
    fb->current_pose.header.frame_id = global_frame_;
    fb->current_pose.header.stamp = now();
    fb->current_pose.pose.position.x = rx;
    fb->current_pose.pose.position.y = ry;
    tf2::Quaternion q;
    q.setRPY(0, 0, ryaw);
    fb->current_pose.pose.orientation = tf2::toMsg(q);
    fb->navigation_time = now() - start_time_;
    fb->distance_remaining = static_cast<float>(dist_goal);
    fb->number_of_recoveries = static_cast<int16_t>(recoveries_);
    active_handle_->publish_feedback(fb);
  }

  void publishPlan()
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = global_frame_;
    msg.header.stamp = now();
    for (const auto & [x, y] : path_) {
      geometry_msgs::msg::PoseStamped p;
      p.header = msg.header;
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.orientation.w = 1.0;
      msg.poses.push_back(p);
    }
    plan_pub_->publish(msg);
  }

  void publishTrajectory(const DwaCommand & cmd)
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = robot_frame_;
    msg.header.stamp = now();
    for (const auto & [x, y] : cmd.trajectory) {
      geometry_msgs::msg::PoseStamped p;
      p.header = msg.header;
      p.pose.position.x = x;
      p.pose.position.y = y;
      p.pose.orientation.w = 1.0;
      msg.poses.push_back(p);
    }
    traj_pub_->publish(msg);
  }

  struct Scan
  {
    rclcpp::Time stamp;
    std::vector<std::pair<double, double>> points;  // memory_frame
  };

  std::string global_frame_, robot_frame_, memory_frame_;
  double control_rate_, replan_period_, goal_tolerance_, yaw_tolerance_, lookahead_;
  bool align_goal_yaw_;
  double obstacle_min_z_, obstacle_max_z_, obstacle_range_, memory_duration_;
  double stuck_timeout_, stuck_distance_;
  int max_recoveries_, max_plan_failures_;

  GridPlanner planner_;
  DwaController dwa_;
  bool have_map_{false};
  std::deque<Scan> scans_;
  double cur_v_{0.0}, cur_w_{0.0};

  bool active_{false};
  std::shared_ptr<GoalHandle> active_handle_;
  double goal_x_{0}, goal_y_{0}, goal_yaw_{0};
  Path path_;
  rclcpp::Time last_plan_time_, start_time_, recovery_until_, stuck_check_time_;
  int plan_failures_{0}, recoveries_{0};
  geometry_msgs::msg::Twist recovery_cmd_;
  bool have_stuck_ref_{false};
  double stuck_ref_x_{0}, stuck_ref_y_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_, traj_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace origin_frontier_explore

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<origin_frontier_explore::CustomNavigator>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

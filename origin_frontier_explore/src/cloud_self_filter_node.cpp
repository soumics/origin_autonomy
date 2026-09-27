// Copyright 2026 Soumic Sarkar
// SPDX-License-Identifier: Apache-2.0
//
// Removes lidar returns that hit the robot itself.
//
// The Ouster on the Origin One sees the rear of the body and the GNSS antenna (about 15% of
// the points in Gazebo). Every point is transformed into `base_frame`; points inside the
// configured box are dropped, as are NaN/inf points. All point fields are kept.

#include <cmath>
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace origin_frontier_explore
{

class CloudSelfFilter : public rclcpp::Node
{
public:
  explicit CloudSelfFilter(const rclcpp::NodeOptions & options)
  : Node("cloud_self_filter", options)
  {
    base_frame_ = declare_parameter("base_frame", "base_link");
    min_x_ = declare_parameter("box_min_x", -0.40);
    max_x_ = declare_parameter("box_max_x", 0.40);
    min_y_ = declare_parameter("box_min_y", -0.34);
    max_y_ = declare_parameter("box_max_y", 0.34);
    min_z_ = declare_parameter("box_min_z", -0.20);
    max_z_ = declare_parameter("box_max_z", 0.60);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("output", rclcpp::QoS(5));
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onCloud(*msg);});
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2 & in)
  {
    // The lidar is rigidly mounted, so the transform is looked up once per sensor frame.
    if (!have_tf_ || in.header.frame_id != sensor_frame_) {
      try {
        const auto t = tf_buffer_->lookupTransform(
          base_frame_, in.header.frame_id, tf2::TimePointZero);
        tf2::fromMsg(t.transform, sensor_to_base_);
        sensor_frame_ = in.header.frame_id;
        have_tf_ = true;
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for TF: %s", e.what());
        return;
      }
    }

    int off_x = -1, off_y = -1, off_z = -1;
    for (const auto & f : in.fields) {
      if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        continue;
      }
      if (f.name == "x") {off_x = f.offset;}
      if (f.name == "y") {off_y = f.offset;}
      if (f.name == "z") {off_z = f.offset;}
    }
    if (off_x < 0 || off_y < 0 || off_z < 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Cloud has no float x/y/z fields");
      return;
    }

    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out->header = in.header;
    out->fields = in.fields;
    out->is_bigendian = in.is_bigendian;
    out->point_step = in.point_step;
    out->height = 1;
    out->is_dense = true;
    out->data.resize(in.data.size());

    size_t kept = 0;
    const size_t n = static_cast<size_t>(in.width) * in.height;
    for (size_t i = 0; i < n; ++i) {
      const uint8_t * p = &in.data[i * in.point_step];
      float x, y, z;
      std::memcpy(&x, p + off_x, sizeof(float));
      std::memcpy(&y, p + off_y, sizeof(float));
      std::memcpy(&z, p + off_z, sizeof(float));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const tf2::Vector3 b = sensor_to_base_ * tf2::Vector3(x, y, z);
      if (b.x() > min_x_ && b.x() < max_x_ && b.y() > min_y_ && b.y() < max_y_ &&
        b.z() > min_z_ && b.z() < max_z_)
      {
        continue;
      }
      std::memcpy(&out->data[kept * in.point_step], p, in.point_step);
      ++kept;
    }
    out->data.resize(kept * in.point_step);
    out->width = static_cast<uint32_t>(kept);
    out->row_step = out->width * out->point_step;
    pub_->publish(std::move(out));
  }

  std::string base_frame_, sensor_frame_;
  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
  bool have_tf_{false};
  tf2::Transform sensor_to_base_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

}  // namespace origin_frontier_explore

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<origin_frontier_explore::CloudSelfFilter>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

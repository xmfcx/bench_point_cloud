// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 M. Fatih Cırıt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bench_point_cloud/bench_point_cloud_node.hpp"

#include <tf2_ros/tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <random>
#include <string>
#include <vector>

#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


using namespace std::literals;
using namespace std::placeholders;

namespace bench_point_cloud
{
BenchPointCloudNode::BenchPointCloudNode(const rclcpp::NodeOptions & node_options)
: Node("bench_point_cloud", node_options)
{
  pub_cloud_xyziad_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_xyziad", 1);
  pub_cloud_xyzi_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_xyzi", 1);
  pub_cloud_xyziad_sorted_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("cloud_xyziad_sorted", 1);
  pub_cloud_xyzi_sorted_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_xyzi_sorted", 1);

  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  trans_.header.stamp = this->get_clock()->now();
  trans_.header.frame_id = "map";
  trans_.child_frame_id = "xyzi";
  trans_.transform.rotation.set__w(1.0);
  trans_.transform.translation.set__x(-100.0);
  trans_.transform.translation.set__y(0.0);
  tf_static_broadcaster_->sendTransform(trans_);

  trans_.child_frame_id = "xyzi_sorted";
  trans_.transform.translation.set__x(0.0);
  trans_.transform.translation.set__y(0.0);
  tf_static_broadcaster_->sendTransform(trans_);

  trans_.child_frame_id = "xyziad";
  trans_.transform.translation.set__x(-100.0);
  trans_.transform.translation.set__y(-100.0);
  tf_static_broadcaster_->sendTransform(trans_);

  trans_.child_frame_id = "xyziad_sorted";
  trans_.transform.translation.set__x(0.0);
  trans_.transform.translation.set__y(-100.0);
  tf_static_broadcaster_->sendTransform(trans_);


  // Timer
  const auto update_period_ns = rclcpp::Rate(1).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&BenchPointCloudNode::onTimer, this));
}

void BenchPointCloudNode::onTimer()
{
  const size_t num_points = 200000;

  sensor_msgs::msg::PointCloud2 msg_cloud_xyziad;
  CloudModifierXYZIAD modifier_xyziad{msg_cloud_xyziad, "xyziad"};
  modifier_xyziad.resize(num_points);

  sensor_msgs::msg::PointCloud2 msg_cloud_xyzi;
  CloudModifierXYZI modifier_xyzi{msg_cloud_xyzi, "xyzi"};
  modifier_xyzi.resize(num_points);

  // populate the point cloud with deterministic random points
  std::mt19937 gen(0);  // NOLINT(cert-msc51-cpp)
  std::uniform_real_distribution<float> dis(0.0f, 95.0f);
  for (size_t i = 0; i < num_points; ++i) {
    modifier_xyziad[i].x = dis(gen);
    modifier_xyziad[i].y = dis(gen);
    modifier_xyziad[i].z = dis(gen);
    modifier_xyziad[i].intensity = dis(gen);
    modifier_xyziad[i].azimuth = std::atan2(modifier_xyziad[i].y, modifier_xyziad[i].x);
    modifier_xyziad[i].distance =
      std::hypot(modifier_xyziad[i].x, modifier_xyziad[i].y, modifier_xyziad[i].z);

    modifier_xyzi[i].x = modifier_xyziad[i].x;
    modifier_xyzi[i].y = modifier_xyziad[i].y;
    modifier_xyzi[i].z = modifier_xyziad[i].z;
    modifier_xyzi[i].intensity = modifier_xyziad[i].intensity;
  }
  pub_cloud_xyziad_->publish(msg_cloud_xyziad);
  pub_cloud_xyzi_->publish(msg_cloud_xyzi);

  // set the intensities to match the azimuth
  {
    const auto tp_start = std::chrono::high_resolution_clock::now();
    for (auto & p : modifier_xyziad) {
      p.intensity = p.azimuth;
    }
    const auto tp_set = std::chrono::high_resolution_clock::now();
    const auto duration_setting =
      std::chrono::duration_cast<std::chrono::microseconds>(tp_set - tp_start);
    std::sort(
      modifier_xyziad.begin(),
      modifier_xyziad.end(),
      [](const PointXYZIAD & a, const PointXYZIAD & b) { return a.intensity < b.intensity; });

    const auto duration_sorting = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - tp_set);


    RCLCPP_INFO(get_logger(), "xyziad setting took %ld us", duration_setting.count());
    RCLCPP_INFO(get_logger(), "xyziad sorting took %ld us", duration_sorting.count());
    RCLCPP_INFO(
      get_logger(),
      "xyziad   total took %ld us",
      duration_setting.count() + duration_sorting.count());
  }

  {
    const auto tp_start = std::chrono::high_resolution_clock::now();
    for (auto & p : modifier_xyzi) {
      p.intensity = std::atan2(p.y, p.x);
      p.z = std::hypot(p.y, p.x, p.z);
    }
    const auto tp_set = std::chrono::high_resolution_clock::now();
    const auto duration_setting =
      std::chrono::duration_cast<std::chrono::microseconds>(tp_set - tp_start);
    std::sort(
      modifier_xyzi.begin(), modifier_xyzi.end(), [](const PointXYZI & a, const PointXYZI & b) {
        return a.intensity < b.intensity;
      });
    const auto duration_sorting = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - tp_set);


    RCLCPP_INFO(get_logger(), "xyzi   setting took %ld us", duration_setting.count());
    RCLCPP_INFO(get_logger(), "xyzi   sorting took %ld us", duration_sorting.count());
    RCLCPP_INFO(
      get_logger(),
      "xyzi     total took %ld us",
      duration_setting.count() + duration_sorting.count());
  }

  msg_cloud_xyziad.header.frame_id = "xyziad_sorted";
  msg_cloud_xyzi.header.frame_id = "xyzi_sorted";

  pub_cloud_xyziad_sorted_->publish(msg_cloud_xyziad);
  pub_cloud_xyzi_sorted_->publish(msg_cloud_xyzi);
}

}  // namespace bench_point_cloud

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bench_point_cloud::BenchPointCloudNode)

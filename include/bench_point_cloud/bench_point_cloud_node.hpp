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

#ifndef BENCH_POINT_CLOUD__BENCH_POINT_CLOUD_NODE_HPP__
#define BENCH_POINT_CLOUD__BENCH_POINT_CLOUD_NODE_HPP__

#include <tf2_ros/tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "example_interfaces/msg/int32.hpp"
#include "rclcpp/rclcpp.hpp"
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace bench_point_cloud
{
using example_interfaces::msg::Int32;

template <typename T> bool rel_eq(const T & a, const T & b)
{
  static_assert(
    std::is_floating_point<T>::value, "Float comparisons only support floating point types.");

  const auto delta = std::abs(a - b);
  const auto larger = std::max(std::abs(a), std::abs(b));
  const auto max_rel_delta = (larger * std::numeric_limits<T>::epsilon());
  return delta <= max_rel_delta;
}

class BenchPointCloudNode : public rclcpp::Node
{
public:
  explicit BenchPointCloudNode(const rclcpp::NodeOptions & node_options);

private:
  struct PointXYZIAD
  {
    float x;
    float y;
    float z;
    float intensity;
    float azimuth;
    float distance;
    friend bool operator==(const PointXYZIAD & p1, const PointXYZIAD & p2) noexcept
    {
      return rel_eq(p1.x, p2.x) && rel_eq(p1.y, p2.y) && rel_eq(p1.z, p2.z) &&
             rel_eq(p1.intensity, p2.intensity) && rel_eq(p1.azimuth, p2.azimuth) &&
             rel_eq(p1.distance, p2.distance);
    }
  };
  LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(azimuth);
  LIDAR_UTILS__DEFINE_FIELD_GENERATOR_FOR_MEMBER(distance);

  using GeneratorsXYZIAD = std::tuple<
    point_cloud_msg_wrapper::field_x_generator,
    point_cloud_msg_wrapper::field_y_generator,
    point_cloud_msg_wrapper::field_z_generator,
    point_cloud_msg_wrapper::field_intensity_generator,
    field_azimuth_generator,
    field_distance_generator>;

  using GeneratorsXYZI = std::tuple<
    point_cloud_msg_wrapper::field_x_generator,
    point_cloud_msg_wrapper::field_y_generator,
    point_cloud_msg_wrapper::field_z_generator,
    point_cloud_msg_wrapper::field_intensity_generator>;


  struct PointXYZI
  {
    float x;
    float y;
    float z;
    float intensity;
    friend bool operator==(const PointXYZI & p1, const PointXYZI & p2) noexcept
    {
      return rel_eq(p1.x, p2.x) && rel_eq(p1.y, p2.y) && rel_eq(p1.z, p2.z) &&
             rel_eq(p1.intensity, p2.intensity);
    }
  };

  using CloudModifierXYZIAD =
    point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIAD, GeneratorsXYZIAD>;

  using CloudModifierXYZI = point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI, GeneratorsXYZI>;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_xyziad_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_xyzi_{};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_xyziad_sorted_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_xyzi_sorted_{};

  void onTimer();

  rclcpp::TimerBase::SharedPtr timer_{};

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  geometry_msgs::msg::TransformStamped trans_;
};

}  // namespace bench_point_cloud

#endif  // BENCH_POINT_CLOUD__BENCH_POINT_CLOUD_NODE_HPP__

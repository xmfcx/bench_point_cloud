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

#include <algorithm>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Geometry>
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

  // Timer
  const auto update_period_ns = rclcpp::Rate(1).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&BenchPointCloudNode::onTimer, this));
}

void BenchPointCloudNode::onTimer()
{
  const size_t num_points = 200000;
  const size_t num_clouds = 5;

  using sensor_msgs::msg::PointCloud2;

  struct CloudWithModifierXYZIAD
  {
    explicit CloudWithModifierXYZIAD(const std::string & frame_id)
    : cloud_ptr{std::make_unique<PointCloud2>()},
      modifier_ptr{std::make_unique<CloudModifierXYZIAD>(*cloud_ptr, frame_id)}
    {
    }
    std::unique_ptr<PointCloud2> cloud_ptr;
    std::unique_ptr<CloudModifierXYZIAD> modifier_ptr;
  };

  std::vector<CloudWithModifierXYZIAD> cloud_with_mods_xyziad;
  cloud_with_mods_xyziad.reserve(num_clouds);
  for (size_t i = 0; i < num_clouds; ++i) {
    cloud_with_mods_xyziad.emplace_back("xyziad_" + std::to_string(i));
    cloud_with_mods_xyziad.back().modifier_ptr->resize(num_points);
  }


  struct CloudWithModifierXYZI
  {
    explicit CloudWithModifierXYZI(const std::string & frame_id)
    : cloud_ptr{std::make_unique<PointCloud2>()},
      modifier_ptr{std::make_unique<CloudModifierXYZI>(*cloud_ptr, frame_id)}
    {
    }
    std::unique_ptr<PointCloud2> cloud_ptr;
    std::unique_ptr<CloudModifierXYZI> modifier_ptr;
  };

  std::vector<CloudWithModifierXYZI> cloud_with_mods_xyzi;
  cloud_with_mods_xyzi.reserve(num_clouds);
  for (size_t i = 0; i < num_clouds; ++i) {
    cloud_with_mods_xyzi.emplace_back("xyzi_" + std::to_string(i));
    cloud_with_mods_xyzi.back().modifier_ptr->resize(num_points);
  }

  assert(cloud_with_mods_xyziad.size() == num_clouds);
  assert(cloud_with_mods_xyzi.size() == num_clouds);

  //  std::random_device rd;
  //  std::mt19937 gen(rd());
  std::mt19937 gen(0);  // NOLINT(cert-msc51-cpp)
  std::uniform_real_distribution<float> dis(-100.0, 100.0);

  // fill both clouds with same random point clouds with size num_points
  for (size_t ind_cloud = 0; ind_cloud < num_clouds; ++ind_cloud) {
    auto & modifier_xyziad = *cloud_with_mods_xyziad[ind_cloud].modifier_ptr;
    auto & modifier_xyzi = *cloud_with_mods_xyzi[ind_cloud].modifier_ptr;
    for (size_t ind_point = 0; ind_point < num_points; ++ind_point) {
      auto & point_xyziad = modifier_xyziad.at(ind_point);
      point_xyziad.x = dis(gen);
      point_xyziad.y = dis(gen);
      point_xyziad.z = dis(gen);
      point_xyziad.intensity = dis(gen);
      point_xyziad.azimuth = std::atan2(point_xyziad.y, point_xyziad.x);
      point_xyziad.distance = std::hypot(point_xyziad.x, point_xyziad.y, point_xyziad.z);
      auto & point_xyzi = modifier_xyzi[ind_point];
      point_xyzi.x = point_xyziad.x;
      point_xyzi.y = point_xyziad.y;
      point_xyzi.z = point_xyziad.z;
      point_xyzi.intensity = point_xyziad.intensity;
    }
  }


  // Transform & Copy for PointXYZI
  {
    // transform points into same frame
    const auto tp_start = std::chrono::high_resolution_clock::now();

    // allocate memory for the transformed points

    std::vector<CloudWithModifierXYZI> cloud_with_mods_xyzi_trans;
    //    cloud_with_mods_xyzi_trans.reserve(num_clouds);
    for (size_t i = 0; i < num_clouds; ++i) {
      cloud_with_mods_xyzi_trans.emplace_back(
        CloudWithModifierXYZI{"xyzi_common_" + std::to_string(i)});

      const auto test_sta = std::chrono::high_resolution_clock::now();
      cloud_with_mods_xyzi_trans.back().modifier_ptr->resize(num_points);
      const auto test_end = std::chrono::high_resolution_clock::now();
      RCLCPP_INFO(
        get_logger(),
        "xyzi %ld us",
        std::chrono::duration_cast<std::chrono::microseconds>(test_end - test_sta).count());
      RCLCPP_INFO(
        get_logger(),
        ("xyzi size: " + std::to_string(cloud_with_mods_xyzi_trans.back().cloud_ptr->data.size()))
          .c_str());
    }

    const auto tp_cp1 = std::chrono::high_resolution_clock::now();

    auto transform_point_xyzi = [](
                                  const PointXYZI & point_xyzi, const Eigen::Matrix4f & transform) {
      Eigen::Vector4f point_input(point_xyzi.x, point_xyzi.y, point_xyzi.z, 1.0);
      Eigen::Vector4f point_output = transform * point_input;
      return PointXYZI{point_output.x(), point_output.y(), point_output.z(), point_xyzi.intensity};
    };

    const Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    for (size_t ind_cloud = 0; ind_cloud < num_clouds; ++ind_cloud) {
      auto & modifier_xyzi = *cloud_with_mods_xyzi[ind_cloud].modifier_ptr;
      auto & modifier_xyzi_trans = *cloud_with_mods_xyzi_trans[ind_cloud].modifier_ptr;

      // apply the same transform to all points
      std::transform(
        modifier_xyzi.begin(),
        modifier_xyzi.end(),
        modifier_xyzi_trans.begin(),
        std::bind(transform_point_xyzi, std::placeholders::_1, transform));
    }

    const auto tp_cp2 = std::chrono::high_resolution_clock::now();

    // combine transformed point clouds into one
    CloudWithModifierXYZI cloud_with_mods_xyzi_combined("xyzi_combined");
    cloud_with_mods_xyzi_combined.modifier_ptr->resize(num_clouds * num_points);

    const auto tp_cp3 = std::chrono::high_resolution_clock::now();

    // copy each cloud into the combined cloud
    auto iter_last = cloud_with_mods_xyzi_combined.modifier_ptr->begin();
    for (size_t i = 0; i < num_clouds; ++i) {
      auto & modifier_xyzi_trans = *cloud_with_mods_xyzi_trans[i].modifier_ptr;
      iter_last = std::copy(modifier_xyzi_trans.cbegin(), modifier_xyzi_trans.cend(), iter_last);
    }

    const auto tp_end = std::chrono::high_resolution_clock::now();

    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(tp_end - tp_start);

    RCLCPP_INFO(get_logger(), "Transform + Copy xyzi   took %ld us", duration.count());
    RCLCPP_INFO(
      get_logger(),
      "  Allocation     xyzi   took %ld us",
      std::chrono::duration_cast<std::chrono::microseconds>(tp_cp1 - tp_start).count());
    RCLCPP_INFO(
      get_logger(),
      "  Transformation xyzi   took %ld us",
      std::chrono::duration_cast<std::chrono::microseconds>(tp_cp2 - tp_cp1).count());
    RCLCPP_INFO(
      get_logger(),
      "  Comb Alloc     xyzi   took %ld us",
      std::chrono::duration_cast<std::chrono::microseconds>(tp_cp3 - tp_cp2).count());
    RCLCPP_INFO(
      get_logger(),
      "  Copy           xyzi   took %ld us",
      std::chrono::duration_cast<std::chrono::microseconds>(tp_end - tp_cp3).count());
  }


  // Transform & Copy for PointXYZIAD
  {
    // transform points into same frame
    const auto tp_start = std::chrono::high_resolution_clock::now();

    // allocate memory for the transformed points

    std::vector<CloudWithModifierXYZIAD> cloud_with_mods_xyziad_trans;
    cloud_with_mods_xyziad_trans.reserve(num_clouds);
    for (size_t i = 0; i < num_clouds; ++i) {
      cloud_with_mods_xyziad_trans.emplace_back("xyziad_common_" + std::to_string(i));

      const auto test_sta = std::chrono::high_resolution_clock::now();
      cloud_with_mods_xyziad_trans.back().modifier_ptr->resize(num_points);
      const auto test_end = std::chrono::high_resolution_clock::now();
      RCLCPP_INFO(
        get_logger(),
        "xyziad %ld us",
        std::chrono::duration_cast<std::chrono::microseconds>(test_end - test_sta).count());
      RCLCPP_INFO(
        get_logger(),
        ("xyziad size: " + std::to_string(cloud_with_mods_xyziad_trans.back().cloud_ptr->data.size()))
          .c_str());
    }

    const auto tp_cp1 = std::chrono::high_resolution_clock::now();

    auto transform_point_xyziad =
      [](const PointXYZIAD & point_xyziad, const Eigen::Matrix4f & transform) {
        PointXYZIAD point_xyziad_transformed = point_xyziad;
        Eigen::Vector4f point_input(point_xyziad.x, point_xyziad.y, point_xyziad.z, 1.0);
        Eigen::Vector4f point_output = transform * point_input;
        point_xyziad_transformed.x = point_output.x();
        point_xyziad_transformed.y = point_output.y();
        point_xyziad_transformed.z = point_output.z();
        return point_xyziad_transformed;
      };

    const Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    for (size_t ind_cloud = 0; ind_cloud < num_clouds; ++ind_cloud) {
      auto & modifier_xyziad = *cloud_with_mods_xyziad[ind_cloud].modifier_ptr;
      auto & modifier_xyziad_trans = *cloud_with_mods_xyziad_trans[ind_cloud].modifier_ptr;

      // apply the same transform to all points
      std::transform(
        modifier_xyziad.begin(),
        modifier_xyziad.end(),
        modifier_xyziad_trans.begin(),
        std::bind(transform_point_xyziad, std::placeholders::_1, transform));
    }

    const auto tp_cp2 = std::chrono::high_resolution_clock::now();

    // combine transformed point clouds into one
    CloudWithModifierXYZIAD cloud_with_mods_xyziad_combined("xyziad_combined");
    cloud_with_mods_xyziad_combined.modifier_ptr->resize(num_clouds * num_points);

    const auto tp_cp3 = std::chrono::high_resolution_clock::now();

    // copy each cloud into the combined cloud
    auto iter_last = cloud_with_mods_xyziad_combined.modifier_ptr->begin();
    for (size_t i = 0; i < num_clouds; ++i) {
      auto & modifier_xyziad_trans = *cloud_with_mods_xyziad_trans[i].modifier_ptr;
      iter_last =
        std::copy(modifier_xyziad_trans.cbegin(), modifier_xyziad_trans.cend(), iter_last);
    }

    const auto tp_end = std::chrono::high_resolution_clock::now();

    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(tp_end - tp_start);

    RCLCPP_INFO(get_logger(), "Transform + Copy xyziad took %ld us", duration.count());
    RCLCPP_INFO(
      get_logger(),
      "  Allocation     xyziad took %ld us",
      std::chrono::duration_cast<std::chrono::microseconds>(tp_cp1 - tp_start).count());
    RCLCPP_INFO(
      get_logger(),
      "  Transformation xyziad took %ld us",
      std::chrono::duration_cast<std::chrono::microseconds>(tp_cp2 - tp_cp1).count());
    RCLCPP_INFO(
      get_logger(),
      "  Comb Alloc     xyziad took %ld us",
      std::chrono::duration_cast<std::chrono::microseconds>(tp_cp3 - tp_cp2).count());
    RCLCPP_INFO(
      get_logger(),
      "  Copy           xyziad took %ld us",
      std::chrono::duration_cast<std::chrono::microseconds>(tp_end - tp_cp3).count());
  }

  RCLCPP_INFO(get_logger(), "---");
}

}  // namespace bench_point_cloud

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bench_point_cloud::BenchPointCloudNode)

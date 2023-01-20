// Copyright 2022 Jacob Hartzer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef APPLICATION__ROS__STATEPUBLISHERNODE_HPP_
#define APPLICATION__ROS__STATEPUBLISHERNODE_HPP_


#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>

#include "ekf/EKF.hpp"
#include "infrastructure/Logger.hpp"

///
/// @class EkfCalNode
/// @brief A ROS2 node for publishing calibration EKF data
/// @todo TF2 Publishing Flag
/// @todo Option to publish health metrics
/// @todo Option to publish visualization messages
///
class StatePublisherNode : public rclcpp::Node
{
public:
  ///
  /// @brief Constructor for the Calibration EKF Node
  ///
  StatePublisherNode();


  ///
  /// @brief Publish EKF state information
  ///
  void PublishStates();

  ///
  /// @brief Publish entire vector state
  ///
  void PublishVectorState();

  ///
  /// @brief Publish body specific states
  ///
  void PublishBodyState();

  ///
  /// @brief Publish sensor extrinsic transforms
  ///
  void PublishSensorTransforms();

private:
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_PosePub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_TwistPub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_StatePub;

  // TF2 objects
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;
  rclcpp::TimerBase::SharedPtr m_tfTimer;

  // Singletons
  EKF * m_ekf = EKF::getInstance();
  Logger * m_logger = Logger::getInstance();
};

#endif  // APPLICATION__ROS__STATEPUBLISHERNODE_HPP_

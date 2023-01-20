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

#include "StatePublisherNode.hpp"

#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ekf/EKF.hpp"
#include "infrastructure/Logger.hpp"
#include "utility/TypeHelper.hpp"


StatePublisherNode::StatePublisherNode()
: Node("StatePublisherNode")
{
  /// @todo Get publish rate from yaml
  m_tfTimer =
    this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&StatePublisherNode::PublishStates, this));
  m_tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  m_PosePub = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
  m_TwistPub = this->create_publisher<geometry_msgs::msg::TwistStamped>("~/twist", 10);
  m_StatePub = this->create_publisher<std_msgs::msg::Float64MultiArray>("~/state", 10);
}

void StatePublisherNode::PublishStates()
{
  PublishVectorState();
  PublishBodyState();
  PublishSensorTransforms();
}
void StatePublisherNode::PublishVectorState() {}

/// @todo move this into separate node
/// @todo publish full state vector
/// @todo publish transforms
void StatePublisherNode::PublishBodyState()
{
  auto pose_msg = geometry_msgs::msg::PoseStamped();
  auto twist_msg = geometry_msgs::msg::TwistStamped();
  auto state_msg = std_msgs::msg::Float64MultiArray();

  Eigen::VectorXd state = m_ekf->GetState();

  // Position
  pose_msg.pose.position.x = state(0);
  pose_msg.pose.position.y = state(1);
  pose_msg.pose.position.z = state(2);

  // Orientation
  Eigen::Quaterniond quat = TypeHelper::RotVecToQuat(state.segment(9, 3));
  pose_msg.pose.orientation.w = quat.w();
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();

  // Linear Velocity
  twist_msg.twist.linear.x = state(3);
  twist_msg.twist.linear.y = state(4);
  twist_msg.twist.linear.z = state(5);

  // Angular Velocity
  twist_msg.twist.angular.x = state(12);
  twist_msg.twist.angular.y = state(13);
  twist_msg.twist.angular.z = state(14);

  // State msg
  for (unsigned int i = 0; i < state.size(); ++i) {
    state_msg.data.push_back(state(i));
  }

  rclcpp::Time now = this->get_clock()->now();
  pose_msg.header.stamp = now;
  twist_msg.header.stamp = now;

  m_PosePub->publish(pose_msg);
  m_TwistPub->publish(twist_msg);
  m_StatePub->publish(state_msg);
}

///
/// @todo debug issue with future extrapolation in RVIZ
/// @todo possibly separate into separate node that just reads and publishes EKF data
///
void StatePublisherNode::PublishSensorTransforms()
{
  // std::string baseIMUName;
  // std::vector<std::string> sensorNames;
  // std::vector<Eigen::Vector3d> sensorPosOffsets;
  // std::vector<Eigen::Quaterniond> sensorAngOffsets;
  // GetTransforms(baseIMUName, sensorNames, sensorPosOffsets, sensorAngOffsets);

  // geometry_msgs::msg::TransformStamped tf;
  // tf.header.frame_id = baseIMUName;

  // // Publish Sensor transforms
  // for (unsigned int i = 0; i < sensorNames.size(); ++i) {
  //   // Sensor name
  //   tf.child_frame_id = sensorNames[i];
  //   tf.header.stamp = this->get_clock()->now();

  //   // Sensor position
  //   tf.transform.translation.x = 0.0;
  //   tf.transform.translation.y = 0.0;
  //   tf.transform.translation.z = 0.0;

  //   // Sensor Orientation
  //   /// @todo some of these quaternions are not valid (nan)
  //   tf.transform.rotation.w = 1.0;
  //   tf.transform.rotation.x = 0.0;
  //   tf.transform.rotation.y = 0.0;
  //   tf.transform.rotation.z = 0.0;

  //   // Send the transformation
  //   m_tfBroadcaster->sendTransform(tf);
  // }

  // Eigen::VectorXd ekfState = m_ekf->GetState();

  // // Publish Body transforms
  // tf.header.frame_id = "world";
  // tf.child_frame_id = baseIMUName;
  // tf.header.stamp = this->get_clock()->now();

  // // Body position
  // tf.transform.translation.x = 0.0;
  // tf.transform.translation.y = 0.0;
  // tf.transform.translation.z = 0.0;

  // // Body Orientation
  // tf.transform.rotation.w = 1.0;
  // tf.transform.rotation.x = 0.0;
  // tf.transform.rotation.y = 0.0;
  // tf.transform.rotation.z = 0.0;

  // m_tfBroadcaster->sendTransform(tf);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatePublisherNode>());
  rclcpp::shutdown();

  return 0;
}

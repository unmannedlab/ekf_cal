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
#include <std_msgs/msg/float64_multi_array.hpp>

#include "ekf/EKF.hpp"
#include "infrastructure/DebugLogger.hpp"
#include "utility/TypeHelper.hpp"


StatePublisherNode::StatePublisherNode()
: Node("StatePublisherNode")
{
  /// @todo Get publish rate from yaml
  m_tf_timer =
    this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&StatePublisherNode::PublishStates, this));
  m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  m_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
  m_twist_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("~/twist", 10);
  m_state_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("~/state", 10);
}

void StatePublisherNode::PublishStates()
{
  PublishVectorState();
  PublishBodyState();
  PublishSensorTransforms();
}

void StatePublisherNode::PublishVectorState()
{
  Eigen::VectorXd vector_state = m_ekf->GetState().ToVector();
  auto state_vec_msg = std_msgs::msg::Float64MultiArray();

  for (auto & element : vector_state) {
    state_vec_msg.data.push_back(element);
  }
  m_state_pub->publish(state_vec_msg);
}


void StatePublisherNode::PublishBodyState()
{
  auto pose_msg = geometry_msgs::msg::PoseStamped();
  auto twist_msg = geometry_msgs::msg::TwistStamped();

  BodyState body_state = m_ekf->GetState().m_body_state;

  // Position
  pose_msg.pose.position.x = body_state.m_position(0);
  pose_msg.pose.position.y = body_state.m_position(1);
  pose_msg.pose.position.z = body_state.m_position(2);

  // Orientation
  pose_msg.pose.orientation.w = body_state.m_orientation.w();
  pose_msg.pose.orientation.x = body_state.m_orientation.x();
  pose_msg.pose.orientation.y = body_state.m_orientation.y();
  pose_msg.pose.orientation.z = body_state.m_orientation.z();

  // Linear Velocity
  twist_msg.twist.linear.x = body_state.m_velocity(0);
  twist_msg.twist.linear.y = body_state.m_velocity(1);
  twist_msg.twist.linear.z = body_state.m_velocity(2);

  // Angular Velocity
  twist_msg.twist.angular.x = body_state.m_angular_velocity(0);
  twist_msg.twist.angular.y = body_state.m_angular_velocity(1);
  twist_msg.twist.angular.z = body_state.m_angular_velocity(2);

  rclcpp::Time now = this->get_clock()->now();
  pose_msg.header.stamp = now;
  twist_msg.header.stamp = now;

  m_pose_pub->publish(pose_msg);
  m_twist_pub->publish(twist_msg);
}

///
/// @todo debug issue with future extrapolation in RVIZ
///
void StatePublisherNode::PublishSensorTransforms()
{
  State ekfState = m_ekf->GetState();

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->get_clock()->now();
  tf.header.frame_id = "body";

  // Publish IMU transforms
  for (auto const & imuIter : ekfState.m_imu_states) {
    unsigned int id = imuIter.first;
    tf.child_frame_id = std::to_string(id);

    // Sensor position
    tf.transform.translation.x = imuIter.second.position(0);
    tf.transform.translation.y = imuIter.second.position(1);
    tf.transform.translation.z = imuIter.second.position(2);

    // Sensor Orientation
    tf.transform.rotation.w = imuIter.second.orientation.w();
    tf.transform.rotation.x = imuIter.second.orientation.x();
    tf.transform.rotation.y = imuIter.second.orientation.y();
    tf.transform.rotation.z = imuIter.second.orientation.z();

    // Send the transformation
    m_tf_broadcaster->sendTransform(tf);
  }

  // Publish Body transforms
  tf.header.frame_id = "world";
  tf.child_frame_id = "body";

  // Body position
  tf.transform.translation.x = ekfState.m_body_state.m_position(0);
  tf.transform.translation.y = ekfState.m_body_state.m_position(1);
  tf.transform.translation.z = ekfState.m_body_state.m_position(2);

  // Body Orientation
  tf.transform.rotation.w = ekfState.m_body_state.m_orientation.w();
  tf.transform.rotation.x = ekfState.m_body_state.m_orientation.x();
  tf.transform.rotation.y = ekfState.m_body_state.m_orientation.y();
  tf.transform.rotation.z = ekfState.m_body_state.m_orientation.z();

  m_tf_broadcaster->sendTransform(tf);
}

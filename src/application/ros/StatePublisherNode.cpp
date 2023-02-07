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
#include "infrastructure/Logger.hpp"
#include "utility/TypeHelper.hpp"


StatePublisherNode::StatePublisherNode()
: Node("StatePublisherNode")
{
  /// @todo Get publish rate from yaml
  m_tfTimer =
    this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&StatePublisherNode::publishStates, this));
  m_tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  m_posePub = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
  m_twistPub = this->create_publisher<geometry_msgs::msg::TwistStamped>("~/twist", 10);
  m_statePub = this->create_publisher<std_msgs::msg::Float64MultiArray>("~/state", 10);
}

void StatePublisherNode::publishStates()
{
  publishVectorState();
  publishBodyState();
  publishSensorTransforms();
}

void StatePublisherNode::publishVectorState()
{
  Eigen::VectorXd vectorState = m_ekf->getState().toVector();
  auto stateVecMsg = std_msgs::msg::Float64MultiArray();

  for (auto & element : vectorState) {
    stateVecMsg.data.push_back(element);
  }
  m_statePub->publish(stateVecMsg);
}


void StatePublisherNode::publishBodyState()
{
  auto poseMsg = geometry_msgs::msg::PoseStamped();
  auto twistMsg = geometry_msgs::msg::TwistStamped();

  BodyState bodyState = m_ekf->getState().bodyState;

  // Position
  poseMsg.pose.position.x = bodyState.position(0);
  poseMsg.pose.position.y = bodyState.position(1);
  poseMsg.pose.position.z = bodyState.position(2);

  // Orientation
  poseMsg.pose.orientation.w = bodyState.orientation.w();
  poseMsg.pose.orientation.x = bodyState.orientation.x();
  poseMsg.pose.orientation.y = bodyState.orientation.y();
  poseMsg.pose.orientation.z = bodyState.orientation.z();

  // Linear Velocity
  twistMsg.twist.linear.x = bodyState.velocity(0);
  twistMsg.twist.linear.y = bodyState.velocity(1);
  twistMsg.twist.linear.z = bodyState.velocity(2);

  // Angular Velocity
  twistMsg.twist.angular.x = bodyState.angularVelocity(0);
  twistMsg.twist.angular.y = bodyState.angularVelocity(1);
  twistMsg.twist.angular.z = bodyState.angularVelocity(2);

  rclcpp::Time now = this->get_clock()->now();
  poseMsg.header.stamp = now;
  twistMsg.header.stamp = now;

  m_posePub->publish(poseMsg);
  m_twistPub->publish(twistMsg);
}

///
/// @todo debug issue with future extrapolation in RVIZ
///
void StatePublisherNode::publishSensorTransforms()
{
  State ekfState = m_ekf->getState();

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->get_clock()->now();
  tf.header.frame_id = "body";

  // Publish IMU transforms
  for (auto const & imuIter : ekfState.imuStates) {
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
    m_tfBroadcaster->sendTransform(tf);
  }

  // Publish Body transforms
  tf.header.frame_id = "world";
  tf.child_frame_id = "body";

  // Body position
  tf.transform.translation.x = ekfState.bodyState.position(0);
  tf.transform.translation.y = ekfState.bodyState.position(1);
  tf.transform.translation.z = ekfState.bodyState.position(2);

  // Body Orientation
  tf.transform.rotation.w = ekfState.bodyState.orientation.w();
  tf.transform.rotation.x = ekfState.bodyState.orientation.x();
  tf.transform.rotation.y = ekfState.bodyState.orientation.y();
  tf.transform.rotation.z = ekfState.bodyState.orientation.z();

  m_tfBroadcaster->sendTransform(tf);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatePublisherNode>());
  rclcpp::shutdown();

  return 0;
}

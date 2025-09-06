// Copyright 2025 Stuart Alldritt
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <rerun_viz/msg_conversion/sensor_msgs/imu.hpp>

using std::placeholders::_1;

namespace rerun_viz
{

Imu::Imu(
  std::shared_ptr<rerun_viz::Node> node, const std::string & topic_name,
  std::shared_ptr<rerun::RecordingStream> rec)
: node_(node), rec_(rec), topic_name_(topic_name)
{
  subscription_ = node_->getRosNode()->create_subscription<sensor_msgs::msg::Imu>(
    topic_name, rclcpp::SensorDataQoS(), std::bind(&Imu::topic_callback, this, _1));
}

void Imu::topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
{
  if (!rec_) {
    RCLCPP_WARN(
      node_->getRosNode()->get_logger(), "No valid RecordingStream, cannot visualize data.");
    return;
  }

  // We need to check the first value of the covariance matrix for each element (orientation, angular_velocity, linear_acceleration)
  // If that element is -1, then that element is not provided by the underlying sensor
  // https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html

  if (msg->angular_velocity_covariance[0] > -1.0) {
    rec_->log(topic_name_ + "/Imu/angular_velocity/x", rerun::Scalars(msg->angular_velocity.x));
    rec_->log(topic_name_ + "/Imu/angular_velocity/y", rerun::Scalars(msg->angular_velocity.y));
    rec_->log(topic_name_ + "/Imu/angular_velocity/z", rerun::Scalars(msg->angular_velocity.z));
  }

  if (msg->linear_acceleration_covariance[0] > -1.0) {
    rec_->log(
      topic_name_ + "/Imu/linear_acceleration/x", rerun::Scalars(msg->linear_acceleration.x));
    rec_->log(
      topic_name_ + "/Imu/linear_acceleration/y", rerun::Scalars(msg->linear_acceleration.y));
    rec_->log(
      topic_name_ + "/Imu/linear_acceleration/z", rerun::Scalars(msg->linear_acceleration.z));
  }

  if (msg->orientation_covariance[0] > -1.0) {
    rec_->log(
      topic_name_ + "/Imu/orientation",
      rerun::Transform3D().with_axis_length(1.0).with_quaternion(
        rerun::Quaternion().from_xyzw(
          msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w)));
  }
}

}  // namespace rerun_viz

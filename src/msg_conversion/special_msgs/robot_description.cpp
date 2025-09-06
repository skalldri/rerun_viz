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

#include <rerun_viz/msg_conversion/special_msgs/robot_description.hpp>
#include <cstddef>

using std::placeholders::_1;

namespace rerun_viz
{

RobotDescription::RobotDescription(
  std::shared_ptr<rerun_viz::Node> node, const std::string & topic_name,
  std::shared_ptr<rerun::RecordingStream> rec)
: node_(node), rec_(rec), topic_name_(topic_name)
{
  subscription_ = node_->getRosNode()->create_subscription<std_msgs::msg::String>(
    topic_name_, rclcpp::BestAvailableQoS(),
    std::bind(&RobotDescription::topic_callback, this, _1));
}

void RobotDescription::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
  if (!rec_) {
    RCLCPP_WARN(
      node_->getRosNode()->get_logger(), "No valid RecordingStream, cannot visualize data.");
    return;
  }

  rec_->log(topic_name_ + "/RobotDescription", rerun::TextDocument(msg->data));

  try {
    rec_->log_file_from_contents(
      "robot.urdf", reinterpret_cast<const std::byte *>(msg->data.c_str()),
      std::strlen(msg->data.c_str()), topic_name_ + "/RobotDescription");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->getRosNode()->get_logger(), "Failed to log robot description: %s", e.what());
  }
}

}  // namespace rerun_viz

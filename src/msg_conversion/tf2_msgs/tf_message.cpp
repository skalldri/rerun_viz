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

#include <rerun_viz/msg_conversion/tf2_msgs/tf_message.hpp>

#include <rerun/image_utils.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

namespace rerun_viz
{

TFMessage::TFMessage(
  std::shared_ptr<rerun_viz::Node> node, const std::string & topic_name,
  std::shared_ptr<rerun::RecordingStream> rec)
: node_(node), rec_(rec), topic_name_(topic_name)
{
  subscription_ = node_->getRosNode()->create_subscription<tf2_msgs::msg::TFMessage>(
    topic_name_, rclcpp::BestAvailableQoS(), std::bind(&TFMessage::topic_callback, this, _1));
}

void TFMessage::topic_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const
{
  RCLCPP_INFO(
    node_->getRosNode()->get_logger(), "Got TF message with %zu transforms!",
    msg->transforms.size());

  if (!rec_) {
    RCLCPP_WARN(
      node_->getRosNode()->get_logger(), "No valid RecordingStream, cannot visualize data.");
    return;
  }

  // Process each transform in the message
  for (const auto & tf_stamped : msg->transforms) {
    const std::string & parent = tf_stamped.header.frame_id;
    const std::string & child = tf_stamped.child_frame_id;

    RCLCPP_DEBUG(
      node_->getRosNode()->get_logger(), "Processing TF: %s -> %s", parent.c_str(), child.c_str());

    // Add transform to the graph
    bool added = tf_graph_.addTransform(tf_stamped);
    if (!added) {
      RCLCPP_WARN(
        node_->getRosNode()->get_logger(), "Failed to add transform %s -> %s (would create cycle)",
        parent.c_str(), child.c_str());
      continue;
    }

    RCLCPP_DEBUG(
      node_->getRosNode()->get_logger(), "Added TF: %s -> %s", parent.c_str(), child.c_str());
  }

  // Log the entire TF tree to Rerun
  tf_graph_.logToRerun(rec_);

  // Log statistics periodically
  auto [num_frames, num_transforms] = tf_graph_.getStatistics();
  RCLCPP_DEBUG(
    node_->getRosNode()->get_logger(), "TF Graph now contains %zu frames and %zu transforms",
    num_frames, num_transforms);
}

}  // namespace rerun_viz

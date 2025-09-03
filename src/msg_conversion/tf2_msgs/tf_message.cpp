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
: rec_(rec), topic_name_(topic_name), node_(node)
{
  subscription_ = node_->getRosNode()->create_subscription<tf2_msgs::msg::TFMessage>(
    topic_name_, rclcpp::BestAvailableQoS(), std::bind(&TFMessage::topic_callback, this, _1));
}

void TFMessage::topic_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const
{
  RCLCPP_INFO(node_->getRosNode()->get_logger(), "Got TF message!");

  if (!rec_) {
    RCLCPP_WARN(
      node_->getRosNode()->get_logger(), "No valid RecordingStream, cannot visualize data.");
    return;
  }

  for (const auto & tf_stamped : msg->transforms) {
    RCLCPP_INFO(
      node_->getRosNode()->get_logger(), "TF: %s -> %s", tf_stamped.header.frame_id.c_str(),
      tf_stamped.child_frame_id.c_str());

    rec_->log(
      "TF/" + tf_stamped.header.frame_id + "/" + tf_stamped.child_frame_id,
      rerun::Transform3D::from_translation({tf_stamped.transform.translation.x,
                                            tf_stamped.transform.translation.y,
                                            tf_stamped.transform.translation.z})
        .with_relation(rerun::components::TransformRelation::ChildFromParent)
        .with_axis_length(0.1f)
        .with_quaternion(
          rerun::datatypes::Quaternion::from_xyzw(
            tf_stamped.transform.rotation.x, tf_stamped.transform.rotation.y,
            tf_stamped.transform.rotation.z, tf_stamped.transform.rotation.w)));
  }
}

}  // namespace rerun_viz

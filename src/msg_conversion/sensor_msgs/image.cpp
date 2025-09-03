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

#include <rerun_viz/msg_conversion/sensor_msgs/image.hpp>

#include <rerun/image_utils.hpp>

#include <sensor_msgs/image_encodings.hpp>

using std::placeholders::_1;

namespace rerun_viz
{

Image::Image(
  std::shared_ptr<rerun_viz::Node> node, const std::string & topic_name,
  std::shared_ptr<rerun::RecordingStream> rec)
: rec_(rec), topic_name_(topic_name), node_(node)
{
  subscription_ = node_->getRosNode()->create_subscription<sensor_msgs::msg::Image>(
    topic_name_, rclcpp::SensorDataQoS(), std::bind(&Image::topic_callback, this, _1));
}

void Image::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
{
  if (!rec_) {
    RCLCPP_WARN(
      node_->getRosNode()->get_logger(), "No valid RecordingStream, cannot visualize data.");
    return;
  }

  if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
    rec_->log(
      topic_name_ + "/Image",
      rerun::Image::from_rgb24(msg->data, rerun::WidthHeight(msg->width, msg->height)));
  } else if (msg->encoding == sensor_msgs::image_encodings::RGBA8) {
    rec_->log(
      topic_name_ + "/Image",
      rerun::Image::from_rgba32(msg->data, rerun::WidthHeight(msg->width, msg->height)));
  } else if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
    rec_->log(
      topic_name_ + "/Image",
      rerun::Image::from_greyscale8(msg->data, rerun::WidthHeight(msg->width, msg->height)));
  } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    rec_->log(
      topic_name_ + "/Image",
      rerun::Image(
        msg->data, rerun::WidthHeight(msg->width, msg->height), rerun::datatypes::ColorModel::L,
        rerun::datatypes::ChannelDatatype::U16));
  } else {
    RCLCPP_WARN(
      node_->getRosNode()->get_logger(), "Unsupported image encoding '%s' on topic %s",
      msg->encoding.c_str(), topic_name_.c_str());
  }
}

}  // namespace rerun_viz

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

#pragma once

#include <memory>
#include <optional>

#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rerun.hpp>
#include "rclcpp/rclcpp.hpp"

#include <rerun_viz/node.hpp>
#include <rerun_viz/msg_conversion/converter.hpp>

namespace rerun_viz
{

class CameraInfo : public Converter
{
public:
  CameraInfo(
    std::shared_ptr<rerun_viz::Node> node, const std::string & topic_name,
    std::shared_ptr<rerun::RecordingStream> rec = nullptr);

  void topic_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  const std::string getRosTypeName() override { return "sensor_msgs/msg/CameraInfo"; }

  const std::vector<TFRequest> getTfRequests() override;

private:
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_;
  std::shared_ptr<rerun_viz::Node> node_;
  std::shared_ptr<rerun::RecordingStream> rec_;
  std::string topic_name_;
  std::optional<TFRequest> tf_request_;
};

}  // namespace rerun_viz

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
#include <mutex>

#include <std_msgs/msg/string.hpp>
#include <rerun.hpp>
#include "rclcpp/rclcpp.hpp"

#include <rerun_viz/node.hpp>
#include <rerun_viz/msg_conversion/converter.hpp>
#include <urdf/model.h>

namespace rerun_viz
{

class RobotDescription : public Converter
{
public:
  RobotDescription(
    std::shared_ptr<rerun_viz::Node> node, const std::string & topic_name,
    std::shared_ptr<rerun::RecordingStream> rec = nullptr);

  void topic_callback(const std_msgs::msg::String::SharedPtr msg);

  const std::string getRosTypeName() override { return "std_msgs/msg/String"; }

  virtual const std::vector<TFRequest> getTfRequests() override;

  void urdfDepthFirst(urdf::Model & model, std::string root, urdf::JointConstSharedPtr joint);

  void urdfDepthFirst(urdf::Model & model, std::string root, urdf::LinkConstSharedPtr link);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::shared_ptr<rerun_viz::Node> node_;
  std::shared_ptr<rerun::RecordingStream> rec_;
  std::string topic_name_;

  std::recursive_mutex tf_requests_mutex_;
  std::vector<TFRequest> tf_requests_;
};

}  // namespace rerun_viz

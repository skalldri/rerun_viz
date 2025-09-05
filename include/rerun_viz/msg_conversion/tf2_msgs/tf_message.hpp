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

#include <tf2_msgs/msg/tf_message.hpp>
#include <rerun.hpp>
#include "rclcpp/rclcpp.hpp"

#include <rerun_viz/node.hpp>
#include <rerun_viz/msg_conversion/converter.hpp>
#include <rerun_viz/tf_graph.hpp>

namespace rerun_viz
{

class TFMessage : public Converter
{
public:
  TFMessage(
    std::shared_ptr<rerun_viz::Node> node, const std::string & topic_name,
    std::shared_ptr<rerun::RecordingStream> rec = nullptr);

  void topic_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) const;

  const std::string getRosTypeName() override { return "tf2_msgs/msg/TFMessage"; }

  /// Get the current TF graph
  const TFGraph & getTFGraph() const { return tf_graph_; }

  /// Get statistics about the TF graph
  std::pair<size_t, size_t> getGraphStatistics() const { return tf_graph_.getStatistics(); }

private:
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
  std::shared_ptr<rerun_viz::Node> node_;
  std::shared_ptr<rerun::RecordingStream> rec_;
  std::string topic_name_;
  mutable TFGraph tf_graph_;  // mutable because topic_callback is const
};

}  // namespace rerun_viz

#pragma once

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rerun.hpp>

#include <rerun_viz/node.hpp>
#include <rerun_viz/msg_conversion/converter.hpp>

namespace rerun_viz
{

class ConverterFactory
{
public:
  static std::shared_ptr<rerun_viz::Converter> getConverterForRosTopic(
    std::shared_ptr<rerun_viz::Node> node, const std::string & topic, const std::string & msgType,
    std::shared_ptr<rerun::RecordingStream> rec);
};

}  // namespace rerun_viz

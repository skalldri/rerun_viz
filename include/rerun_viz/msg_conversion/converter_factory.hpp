
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rerun_viz/msg_conversion/converter.hpp>

namespace rerun_viz
{

class ConverterFactory
{
public:
  static std::shared_ptr<rerun_viz::Converter> getConverterForRosTopic(
    rclcpp::Node::SharedPtr node, const std::string & topic, const std::string & msgType);
};

}  // namespace rerun_viz

#include <rerun_viz/msg_conversion/converter_factory.hpp>

#include <rerun_viz/msg_conversion/sensor_msgs/point_cloud2.hpp>

namespace rerun_viz
{

std::shared_ptr<rerun_viz::Converter> ConverterFactory::getConverterForRosTopic(
  rclcpp::Node::SharedPtr node, const std::string & topic, const std::string & msgType)
{
  if (msgType == "sensor_msgs/msg/PointCloud2") {
    return std::make_shared<rerun_viz::PointCloud2>(node, topic);
  }

  // Did not match any types
  throw std::runtime_error("Unsupported ROS message type: " + msgType);
}

}  // namespace rerun_viz

#include <rerun_viz/msg_conversion/converter_factory.hpp>

#include <rerun_viz/msg_conversion/sensor_msgs/point_cloud2.hpp>
#include <rerun_viz/msg_conversion/sensor_msgs/imu.hpp>
#include <rerun_viz/msg_conversion/sensor_msgs/image.hpp>
#include <rerun_viz/msg_conversion/sensor_msgs/camera_info.hpp>
#include <rerun_viz/msg_conversion/tf2_msgs/tf_message.hpp>

namespace rerun_viz
{

std::shared_ptr<rerun_viz::Converter> ConverterFactory::getConverterForRosTopic(
  std::shared_ptr<rerun_viz::Node> node, const std::string & topic, const std::string & msgType,
  std::shared_ptr<rerun::RecordingStream> rec)
{
  if (msgType == "sensor_msgs/msg/PointCloud2") {
    return std::make_shared<rerun_viz::PointCloud2>(node, topic, rec);
  } else if (msgType == "sensor_msgs/msg/Imu") {
    return std::make_shared<rerun_viz::Imu>(node, topic, rec);
  } else if (msgType == "sensor_msgs/msg/Image") {
    return std::make_shared<rerun_viz::Image>(node, topic, rec);
  } else if (msgType == "sensor_msgs/msg/CameraInfo") {
    return std::make_shared<rerun_viz::CameraInfo>(node, topic, rec);
  } else if (msgType == "tf2_msgs/msg/TFMessage") {
    return std::make_shared<rerun_viz::TFMessage>(node, topic, rec);
  }

  // Did not match any types
  throw std::runtime_error("Unsupported ROS message type: " + msgType);
}

}  // namespace rerun_viz

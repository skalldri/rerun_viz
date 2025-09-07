#pragma once

#include "rclcpp/rclcpp.hpp"
#include <rerun.hpp>

#include <rerun_viz/msg_conversion/converter.hpp>

#include <thread>
#include <map>
#include <vector>
#include <mutex>
#include <atomic>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2/time.hpp>

namespace rerun_viz
{

class Node : public std::enable_shared_from_this<Node>
{
public:
  /**
   * @brief Construct a new Node object
   * 
   * @param rec a shared pointer to a RecordingStream, can be null
   * @param node a shared pointer to an rclcpp::Node, must not be null
   */
  Node(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<rclcpp::Node> node);

  void on_timer();

  void handleTfRequest(const TFRequest & req);

  /**
   * @brief Given a map of topic names to list of types on that topic, update our subscriptions
   * such that we are subscribed to all topics on all supported types. This will internally cause 
   * the creation and destruction of Converter instances as needed to match the current set of topics and datatypes.
   * 
   * @param topicNamesAndTypes 
   */
  void updateSubscriptions(std::map<std::string, std::vector<std::string>> topicNamesAndTypes);

  /**
   * @brief Get the map current subcriptions, that maps subscribed topics -> list of datatypes on that topic.
   * 
   * @return const std::map<std::string, std::vector<std::string>> The mapping of topic names -> list of datatypes
   * on that topic
   */
  const std::map<std::string, std::vector<std::string>> getSubscriptions();

  /**
   * @brief Get a Converter for a given topic and type, if one exists. Returns an invalid shared pointer if 
   * there is no registered Converter for that topic + type yet.
   * 
   * @param topic the topic name you want a converter for
   * @param type the datatype on the topic you want a converter for 
   * @return std::shared_ptr<Converter> the Converter for that topic and type, or an invalid shared pointer if none exists 
   */
  std::shared_ptr<Converter> getConverter(const std::string & topic, const std::string & type);

  /**
   * @brief Get the underlying ROS node object
   * 
   * @return std::shared_ptr<rclcpp::Node> 
   */
  std::shared_ptr<rclcpp::Node> getRosNode() { return node_; }

  /**
   * @brief Return the TF Frame ID that is used as the "fixed" frame in the world co-ordinate system.
   * 
   * All other TFs will be submitted relative to this frame
   * 
   * @return std::string the fixed frame ID, currently hardcoded. Will make this a rosparam at some point
   */
  std::string getFixedFrameId() { return "loomo_odom"; }

  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time);

private:
  std::mutex subscriptions_mutex_;
  std::map<std::string, std::vector<std::shared_ptr<Converter>>> subscriptions_;
  std::shared_ptr<rerun::RecordingStream> rec_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rerun_viz

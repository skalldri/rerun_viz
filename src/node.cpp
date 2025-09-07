
#include <rerun_viz/node.hpp>
#include <rerun_viz/utils.hpp>
#include <rerun_viz/msg_conversion/converter_factory.hpp>

#include <chrono>
#include <thread>
#include <algorithm>

using std::literals::chrono_literals::operator""s;

namespace rerun_viz
{

Node::Node(std::shared_ptr<rerun::RecordingStream> rec, std::shared_ptr<rclcpp::Node> node)
: rec_(rec), node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "Rerun Viz Node has been started.");

  if (node_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true /* spin thread */);

    timer_ = node_->create_wall_timer(0.1s, [this]() { return this->on_timer(); });
  }
}

void Node::handleTfRequest(const TFRequest & req)
{
  // Another beautiful hack...
  auto tf =
    (req.tf_type == TFRequestType::RotationOnly)
      ? convertTransformToRerun(
          lookupTransform(req.ros_parent_frame_id, req.ros_child_frame_id, tf2::TimePointZero))
          .with_translation({0.0, 0.0, 0.0})
      : convertTransformToRerun(
          lookupTransform(req.ros_parent_frame_id, req.ros_child_frame_id, tf2::TimePointZero));

  rec_->log(req.rerun_entity_path, tf);
}

void Node::on_timer()
{
  if (tf_buffer_) {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);

    // Check all our converters to see if they have any requests for the TF graph.
    // Publish Transform3Ds to any Rerun entity requested by the converters.
    for (const auto & [topic_name, converters] : subscriptions_) {
      for (const auto & converter : converters) {
        const auto tf_requests = converter->getTfRequests();

        for (const auto & req : tf_requests) {
          RCLCPP_DEBUG(
            node_->get_logger(), "Handling TF request for %s -> %s at ReRun path %s",
            req.ros_parent_frame_id.c_str(), req.ros_child_frame_id.c_str(),
            req.rerun_entity_path.c_str());
          try {
            handleTfRequest(req);
          } catch (const std::exception & e) {
            RCLCPP_WARN(
              node_->get_logger(), "Failed to lookup TF %s -> %s for rerun entity %s: %s",
              req.ros_parent_frame_id.c_str(), req.ros_child_frame_id.c_str(),
              req.rerun_entity_path.c_str(), e.what());
          }
        }
      }
    }
  }
}

const std::map<std::string, std::vector<std::string>> Node::getSubscriptions()
{
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);

  // Construct a map of topic_name -> [type names]
  std::map<std::string, std::vector<std::string>> subscriptions;
  for (const auto & [topic_name, converters] : subscriptions_) {
    std::vector<std::string> type_names;
    for (const auto & converter : converters) {
      type_names.push_back(converter->getRosTypeName());
    }
    subscriptions[topic_name] = type_names;
  }

  return subscriptions;
}

std::shared_ptr<Converter> Node::getConverter(const std::string & topic, const std::string & type)
{
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  auto it = subscriptions_.find(topic);
  if (it == subscriptions_.end()) {
    return nullptr;
  }
  for (const auto & converter : it->second) {
    if (converter && converter->getRosTypeName() == type) {
      return converter;
    }
  }
  return nullptr;
}

void Node::updateSubscriptions(std::map<std::string, std::vector<std::string>> topicNamesAndTypes)
{
  // We receive a map of topic_name -> [types]
  //
  // We expect that this represents the full list of known topics.
  //
  // We need to:
  // - Remove any subscriptions we have to topics that are no longer present
  // - Add subscriptions to any new topics we don't already have
  // - Ensure that existing subscriptions have converters for all possible data types on the topic
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);

  std::vector<std::string> topics_to_remove;

  std::shared_ptr<rerun_viz::Node> shared_this;
  try {
    shared_this = shared_from_this();
  } catch (const std::bad_weak_ptr & wp) {
    RCLCPP_ERROR(
      node_->get_logger(), "Cannot update subscriptions: Node is not managed by a shared_ptr");
    return;
  }

  // Step 1: Remove any subscriptions we have to topics that are no longer present
  for (const auto & [topic_name, type_list] : subscriptions_) {
    if (topicNamesAndTypes.find(topic_name) == topicNamesAndTypes.end()) {
      RCLCPP_INFO(node_->get_logger(), "Removing subscription to topic: %s", topic_name.c_str());
      topics_to_remove.push_back(topic_name);
    }
  }

  // Perform the "erase" step outside the iterator
  for (auto & topic_name : topics_to_remove) {
    subscriptions_.erase(topic_name);
  }

  // Step 2: Add subscriptions for any new topics we don't already have
  for (const auto & [topic_name, type_list] : topicNamesAndTypes) {
    // The topic is not in our list of existing subscriptions, so add new subscriptions for
    // all possible datatypes
    if (subscriptions_.find(topic_name) == subscriptions_.end()) {
      std::vector<std::shared_ptr<Converter>> converters_for_topic;
      // Try to add a converter for each datatype
      for (const auto & type : type_list) {
        try {
          auto converter =
            ConverterFactory::getConverterForRosTopic(shared_this, topic_name, type, rec_);
          converters_for_topic.push_back(converter);
          RCLCPP_INFO(
            node_->get_logger(), "Added subscription to topic: %s for type %s", topic_name.c_str(),
            type.c_str());
        } catch (const std::runtime_error & e) {
          RCLCPP_WARN(
            node_->get_logger(), "Cannot subscribe to topic %s with type %s: %s",
            topic_name.c_str(), type.c_str(), e.what());
        }
      }

      subscriptions_[topic_name] = converters_for_topic;
    } else {
      // This topic already exists in our subscription list. We need to check
      // that we have a converter for each datatype on the topic, and remove
      // any that are no longer needed.

      // Check that we have a converter for each type on this topic
      auto & existing_converters = subscriptions_[topic_name];
      for (const auto & type : type_list) {
        bool found = false;
        // Loop through all existing converters, checking their ROS msg type.
        // If we find one that matches, we are done.
        for (const auto & converter : existing_converters) {
          if (converter->getRosTypeName() == type) {
            found = true;
            break;
          }
        }

        // We did not find a converter for this type, so try to create one
        if (!found) {
          try {
            auto converter =
              ConverterFactory::getConverterForRosTopic(shared_this, topic_name, type, rec_);
            existing_converters.push_back(converter);
          } catch (const std::runtime_error & e) {
            RCLCPP_DEBUG(
              node_->get_logger(), "Cannot subscribe to topic %s with type %s: %s",
              topic_name.c_str(), type.c_str(), e.what());
          }
        }
      }

      // Now remove any converters that are no longer needed
      for (auto it = existing_converters.begin(); it != existing_converters.end();) {
        bool found = false;
        for (const auto & type : type_list) {
          if ((*it)->getRosTypeName() == type) {
            found = true;
            break;
          }
        }
        if (!found) {
          RCLCPP_INFO(
            node_->get_logger(), "Removing converter for topic: %s of type: %s", topic_name.c_str(),
            (*it)->getRosTypeName().c_str());
          it = existing_converters.erase(it);
        } else {
          ++it;
        }
      }
    }
  }
}

geometry_msgs::msg::TransformStamped Node::lookupTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time)
{
  return tf_buffer_->lookupTransform(target_frame, source_frame, time);
}

}  // namespace rerun_viz

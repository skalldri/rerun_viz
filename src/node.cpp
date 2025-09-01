
#include <rerun_viz/node.hpp>
#include <rerun_viz/msg_conversion/converter_factory.hpp>

#include <chrono>
#include <thread>

using std::literals::chrono_literals::operator""s;

namespace rerun_viz
{

Node::Node() : rclcpp::Node("rerun_viz_node")
{
  RCLCPP_INFO(this->get_logger(), "Rerun Viz Node has been started.");

  graph_update_thread_ = std::thread(std::bind(&Node::graphUpdateThread, this));

  // rclcpp::on_shutdown([this]() { this->stopAndJoinGraphUpdateThread(); });
}

Node::~Node()
{
  // Force clangformat onto two lines
  std::cout << "~Node Start" << std::endl;
  stopAndJoinGraphUpdateThread();
  std::cout << "~Node End" << std::endl;
}

void Node::stopAndJoinGraphUpdateThread()
{
  std::cout << "Shutting down Rerun Viz Node." << std::endl;
  stop_graph_update_thread_.store(true);

  if (graph_update_thread_.joinable()) {
    graph_update_thread_.join();
  }
  std::cout << "Shutdown complete" << std::endl;
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

  rclcpp::Node::SharedPtr node_shared_ptr;
  try {
    node_shared_ptr = shared_from_this();
  } catch (const std::bad_weak_ptr &) {
    std::cout << "Failed to get Node shared_ptr: cannot update subscriptions" << std::endl;
    return;
  }

  std::vector<std::string> topics_to_remove;

  // Step 1: Remove any subscriptions we have to topics that are no longer present
  for (const auto & [topic_name, type_list] : subscriptions_) {
    if (topicNamesAndTypes.find(topic_name) == topicNamesAndTypes.end()) {
      RCLCPP_INFO(this->get_logger(), "Removing subscription to topic: %s", topic_name.c_str());
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
      RCLCPP_INFO(this->get_logger(), "Adding subscription to topic: %s", topic_name.c_str());

      std::vector<std::shared_ptr<Converter>> converters_for_topic;

      // Try to add a converter for each datatype
      for (const auto & type : type_list) {
        try {
          auto converter =
            ConverterFactory::getConverterForRosTopic(node_shared_ptr, topic_name, type);
          converters_for_topic.push_back(converter);
        } catch (const std::runtime_error & e) {
          try {
            RCLCPP_WARN(
              this->get_logger(), "Cannot subscribe to topic %s with type %s: %s",
              topic_name.c_str(), type.c_str(), e.what());
          } catch (const std::bad_weak_ptr & wp) {
            std::cout << "Failure during ROS logging" << std::endl;
          }
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
              ConverterFactory::getConverterForRosTopic(node_shared_ptr, topic_name, type);
            existing_converters.push_back(converter);
          } catch (const std::runtime_error & e) {
            RCLCPP_DEBUG(
              this->get_logger(), "Cannot subscribe to topic %s with type %s: %s",
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
            this->get_logger(), "Removing converter for topic: %s of type: %s", topic_name.c_str(),
            (*it)->getRosTypeName().c_str());
          it = existing_converters.erase(it);
        } else {
          ++it;
        }
      }
    }
  }
}

bool Node::canGraphUpdateThreadRun()
{
  // Two lines
  bool stop_thread = stop_graph_update_thread_.load();
  bool ok = rclcpp::ok();
  // std::cout << "canGraphUpdateThreadRun: ok=" << ok << ", stop_thread=" << stop_thread << std::endl;
  return ok && !stop_thread;
}

void Node::getTopicsAndUpdateSubscriptions()
{
  std::cout << "getTopicsAndUpdateSubscriptions!" << std::endl;

  auto topicNamesAndTypes = this->get_topic_names_and_types();

  for (const auto & [topic_name, type_list] : topicNamesAndTypes) {
    std::string types;
    for (const auto & type : type_list) {
      if (!types.empty()) {
        types += ", ";
      }
      types += type;
    }
    RCLCPP_DEBUG(this->get_logger(), "  %s: %s", topic_name.c_str(), types.c_str());
  }

  updateSubscriptions(topicNamesAndTypes);
}

void Node::graphUpdateThread()
{
  RCLCPP_INFO(this->get_logger(), "Starting graph update thread");

  while (canGraphUpdateThreadRun()) {
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, 1s);

    if (!canGraphUpdateThreadRun()) {
      std::cout << "Aborting Graph Update Thread!" << std::endl;
      return;
    }

    std::cout << "Updating topic subscriptions from thread!" << std::endl;
    getTopicsAndUpdateSubscriptions();
  }
}

}  // namespace rerun_viz

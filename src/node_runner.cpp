#include <rerun_viz/node_runner.hpp>

#include <chrono>

using std::literals::chrono_literals::operator""s;

namespace rerun_viz
{

NodeRunner::NodeRunner(std::shared_ptr<Node> node) : node_(node)
{
  graph_update_thread_ = std::thread(std::bind(&NodeRunner::graphUpdateThread, this));
}

NodeRunner::~NodeRunner() { stopAndJoinGraphUpdateThread(); }

void NodeRunner::stopAndJoinGraphUpdateThread()
{
  std::cout << "Shutting down Rerun Viz Node." << std::endl;
  stop_graph_update_thread_.store(true);

  if (graph_update_thread_.joinable()) {
    graph_update_thread_.join();
  }
  std::cout << "Shutdown complete" << std::endl;
}

bool NodeRunner::canGraphUpdateThreadRun()
{
  bool stop_thread = stop_graph_update_thread_.load();
  bool ok = rclcpp::ok();
  return ok && !stop_thread;
}

void NodeRunner::graphUpdateThread()
{
  RCLCPP_INFO(node_->getRosNode()->get_logger(), "Starting graph update thread");

  while (canGraphUpdateThreadRun()) {
    rclcpp::Event::SharedPtr event = node_->getRosNode()->get_graph_event();
    node_->getRosNode()->wait_for_graph_change(event, 1s);

    if (!canGraphUpdateThreadRun()) {
      return;
    }

    auto topicNamesAndTypes = node_->getRosNode()->get_topic_names_and_types();
    node_->updateSubscriptions(topicNamesAndTypes);
  }
}

}  // namespace rerun_viz

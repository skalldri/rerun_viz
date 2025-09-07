#pragma once

#include <rerun_viz/node.hpp>
#include <memory>

namespace rerun_viz
{

class NodeRunner
{
public:
  NodeRunner(std::shared_ptr<Node> node);
  ~NodeRunner();

private:
  /**
   * @brief Stop and join the graph update thread.
   * 
   */
  void stopAndJoinGraphUpdateThread();

  /**
   * @brief The entry point for the graph update thread. This will periodically query the ROS node
   * for the current list of topics and types, and call updateSubscriptions() to ensure we are subscribed
   * to all topics.
   */
  void graphUpdateThread();

  /**
   * @brief Helper function that encapsulates the conditions that dictate if the graph update thread should keep running.
   * 
   * @return true keep running
   * @return false quit at the next opportunity
   */
  bool canGraphUpdateThreadRun();

  std::shared_ptr<Node> node_;

  std::thread graph_update_thread_;
  std::atomic<bool> stop_graph_update_thread_{false};
};

}  // namespace rerun_viz


#include "rclcpp/rclcpp.hpp"
#include <rerun_viz/msg_conversion/converter.hpp>
#include <rerun.hpp>

#include <thread>
#include <map>
#include <vector>
#include <mutex>
#include <atomic>

namespace rerun_viz
{

class Node : public rclcpp::Node
{
public:
  Node(std::shared_ptr<rerun::RecordingStream> rec);
  ~Node();

  void updateSubscriptions(std::map<std::string, std::vector<std::string>> topicNamesAndTypes);

  const std::map<std::string, std::vector<std::string>> getSubscriptions();

  void getTopicsAndUpdateSubscriptions();

private:
  void stopAndJoinGraphUpdateThread();
  void graphUpdateThread();
  bool canGraphUpdateThreadRun();

  std::thread graph_update_thread_;
  std::atomic_bool stop_graph_update_thread_{false};

  std::mutex subscriptions_mutex_;
  std::map<std::string, std::vector<std::shared_ptr<Converter>>> subscriptions_;
  std::shared_ptr<rerun::RecordingStream> rec_;
};

}  // namespace rerun_viz

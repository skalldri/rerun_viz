
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

  graph_update_thread_ = std::thread(std::bind(&Node::graphUpdateThread, this));
}

void Node::on_timer()
{
  if (tf_buffer_) {
    auto frames = tf_buffer_->getAllFrameNames();
    for (const auto & f : frames) {
      RCLCPP_INFO(node_->get_logger(), "TF Frame: %s", f.c_str());
    }

    // TODO: this is a massive hack.
    // We need a way to extract the set of required frames from the current set of subscriptions,
    // and then publish the graph of TFs that are needed to cover those frames.
    // I think we probably need to resurrect the "TFMessage" converter to maintain the set of
    // frames as a hierarchy, but farm out to the core TF2 library to perform the actual transform conversions.
    //
    // Wrapping everything in the TF buffer makes it all too abstract.

    // As a proof of concept, let's publish the known TF graph from the robot
    try {
      // Lets just see if this works...
      const auto fixedFrame = getFixedFrameId();

      auto tf =
        convertTransformToRerun(lookupTransform(fixedFrame, "base_link", tf2::TimePointZero));
      rec_->log("/loomo/base_link", tf);

      // Strip the translation component off these joints: we only need the rotation component,
      // the translation is captured by the "joint" that is produced by ReRun's URDF loader
      tf =
        convertTransformToRerun(lookupTransform("base_link", "head_yaw_link", tf2::TimePointZero))
          .with_translation({0.0, 0.0, 0.0});
      rec_->log("/loomo/base_link/neck_yaw_joint/head_yaw_link", tf);

      tf = convertTransformToRerun(
             lookupTransform("head_yaw_link", "head_pitch_link", tf2::TimePointZero))
             .with_translation({0.0, 0.0, 0.0});
      rec_->log(
        "/loomo/base_link/neck_yaw_joint/head_yaw_link/head_pitch_joint/head_pitch_link", tf);
    } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(), "Failed to lookup TF: %s", e.what());
    }
  }
}

Node::~Node()
{
  // TODO: there is a race condition here that needs to be resolved.
  // The race can occur if the last "application" shared-pointer to the Node is destroyed
  // while the graph update thread is in the middle of calling updateSubscriptions(). In this
  // situation the final remaining shared-pointer to the Node lives inside updateSubscriptions().
  // When updateSubscriptions() exits and the shared-pointer goes out of scope, we end up calling the
  // ~Node() destructor from within the context of the graph update thread. This eventually causes us to
  // call std::thread::join() on our own thread, which is not allowed and results in a crash.

  // Possible solutions:
  // - Detect if we are running within our own thread context and do something smarter
  // - Use a weak-pointer for the graph update thread, and only lock it when we need to call updateSubscriptions().
  // This may re-introduce the original problem of the shared-ptr going out of scope while we are in the middle of updateSubscriptions().
  // - Something else ???

  // GDB stacktrace of the crash:
  /*
  #0  __pthread_kill_implementation (no_tid=0, signo=6, threadid=<optimized out>) at ./nptl/pthread_kill.c:44
#1  __pthread_kill_internal (signo=6, threadid=<optimized out>) at ./nptl/pthread_kill.c:78
#2  __GI___pthread_kill (threadid=<optimized out>, signo=signo@entry=6) at ./nptl/pthread_kill.c:89
#3  0x00007ffff2a4527e in __GI_raise (sig=sig@entry=6) at ../sysdeps/posix/raise.c:26
#4  0x00007ffff2a288ff in __GI_abort () at ./stdlib/abort.c:79
#5  0x00007ffff2ea5ff5 in __gnu_cxx::__verbose_terminate_handler () at ../../../../src/libstdc++-v3/libsupc++/vterminate.cc:95
#6  0x00007ffff2ebb0da in __cxxabiv1::__terminate (handler=<optimized out>) at ../../../../src/libstdc++-v3/libsupc++/eh_terminate.cc:48
#7  0x00007ffff2ea58e6 in __cxa_call_terminate (ue_header_in=0x7fffbc001690) at ../../../../src/libstdc++-v3/libsupc++/eh_call.cc:56
#8  0x00007ffff2eba8ba in __cxxabiv1::__gxx_personality_v0 (version=<optimized out>, actions=6, exception_class=5138137972254386944, ue_header=0x7fffbc001690, 
    context=0x7fffc7ffb5d0) at ../../../../src/libstdc++-v3/libsupc++/eh_personality.cc:692
#9  0x00007ffff7edbb06 in _Unwind_RaiseException_Phase2 (exc=exc@entry=0x7fffbc001690, context=context@entry=0x7fffc7ffb5d0, 
    frames_p=frames_p@entry=0x7fffc7ffb6c0) at ../../../src/libgcc/unwind.inc:64
#10 0x00007ffff7edc1f1 in _Unwind_RaiseException (exc=0x7fffbc001690) at ../../../src/libgcc/unwind.inc:136
#11 0x00007ffff2ebb384 in __cxxabiv1::__cxa_throw (obj=<optimized out>, tinfo=0x7ffff30704f8 <typeinfo for std::system_error>, 
    dest=0x7ffff2eecb20 <std::system_error::~system_error()>) at ../../../../src/libstdc++-v3/libsupc++/eh_throw.cc:93
#12 0x00007ffff2eaa2a5 in std::__throw_system_error (__i=35) at ../../../../../src/libstdc++-v3/src/c++11/system_error.cc:595
#13 0x00007ffff2eaa2fe in std::thread::join (this=0x555556c04510) at ../../../../../src/libstdc++-v3/src/c++11/thread.cc:137
#14 0x0000555555d8029f in rerun_viz::Node::stopAndJoinGraphUpdateThread (this=0x555556c04500)
    at /home/salldritt/workspace/ros2_jazzy_rerun/src/rerun_viz/src/node.cpp:34
#15 0x0000555555d800fc in rerun_viz::Node::~Node (this=0x555556c04500, __in_chrg=<optimized out>)
    at /home/salldritt/workspace/ros2_jazzy_rerun/src/rerun_viz/src/node.cpp:25
#16 0x0000555555c91e6b in std::_Destroy<rerun_viz::Node> (__pointer=0x555556c04500) at /usr/include/c++/13/bits/stl_construct.h:151
#17 0x0000555555c88ebe in std::allocator_traits<std::allocator<void> >::destroy<rerun_viz::Node> (__p=0x555556c04500)
    at /usr/include/c++/13/bits/alloc_traits.h:675
#18 std::_Sp_counted_ptr_inplace<rerun_viz::Node, std::allocator<void>, (__gnu_cxx::_Lock_policy)2>::_M_dispose (this=0x555556c044f0)
    at /usr/include/c++/13/bits/shared_ptr_base.h:613
#19 0x0000555555c4ce9b in std::_Sp_counted_base<(__gnu_cxx::_Lock_policy)2>::_M_release_last_use (this=0x555556c044f0)
    at /usr/include/c++/13/bits/shared_ptr_base.h:175
--Type <RET> for more, q to quit, c to continue without paging--c
#20 0x0000555555c47e72 in std::_Sp_counted_base<(__gnu_cxx::_Lock_policy)2>::_M_release_last_use_cold (this=0x555556c044f0)
    at /usr/include/c++/13/bits/shared_ptr_base.h:199
#21 0x0000555555c4226b in std::_Sp_counted_base<(__gnu_cxx::_Lock_policy)2>::_M_release (this=0x555556c044f0) at /usr/include/c++/13/bits/shared_ptr_base.h:353
#22 0x0000555555c47ed7 in std::__shared_count<(__gnu_cxx::_Lock_policy)2>::~__shared_count (this=0x7fffc7ffbba8, __in_chrg=<optimized out>)
    at /usr/include/c++/13/bits/shared_ptr_base.h:1071
#23 0x0000555555c4782a in std::__shared_ptr<rerun_viz::Node, (__gnu_cxx::_Lock_policy)2>::~__shared_ptr (this=0x7fffc7ffbba0, __in_chrg=<optimized out>)
    at /usr/include/c++/13/bits/shared_ptr_base.h:1524
#24 0x0000555555c4786e in std::shared_ptr<rerun_viz::Node>::~shared_ptr (this=0x7fffc7ffbba0, __in_chrg=<optimized out>)
    at /usr/include/c++/13/bits/shared_ptr.h:175
#25 0x0000555555d82999 in rerun_viz::Node::updateSubscriptions (this=0x555556c04500, topicNamesAndTypes=std::map with 3 elements = {...})
    at /home/salldritt/workspace/ros2_jazzy_rerun/src/rerun_viz/src/node.cpp:186
#26 0x0000555555d84d59 in rerun_viz::Node::getTopicsAndUpdateSubscriptions (this=0x555556c04500)
    at /home/salldritt/workspace/ros2_jazzy_rerun/src/rerun_viz/src/node.cpp:212
#27 0x0000555555d85550 in rerun_viz::Node::graphUpdateThread (this=0x555556c04500) at /home/salldritt/workspace/ros2_jazzy_rerun/src/rerun_viz/src/node.cpp:227
#28 0x0000555555d92048 in std::__invoke_impl<void, void (rerun_viz::Node::*&)(), rerun_viz::Node*&> (
    __f=@0x555556bc24a8: (void (rerun_viz::Node::*)(class rerun_viz::Node * const)) 0x555555d85018 <rerun_viz::Node::graphUpdateThread()>, 
    __t=@0x555556bc24b8: 0x555556c04500) at /usr/include/c++/13/bits/invoke.h:74
#29 0x0000555555d91efe in std::__invoke<void (rerun_viz::Node::*&)(), rerun_viz::Node*&> (
    __fn=@0x555556bc24a8: (void (rerun_viz::Node::*)(class rerun_viz::Node * const)) 0x555555d85018 <rerun_viz::Node::graphUpdateThread()>)
    at /usr/include/c++/13/bits/invoke.h:96
#30 0x0000555555d91d8f in std::_Bind<void (rerun_viz::Node::*(rerun_viz::Node*))()>::__call<void, , 0ul>(std::tuple<>&&, std::_Index_tuple<0ul>) (
    this=0x555556bc24a8, __args=...) at /usr/include/c++/13/functional:506
#31 0x0000555555d91c59 in std::_Bind<void (rerun_viz::Node::*(rerun_viz::Node*))()>::operator()<, void>() (this=0x555556bc24a8)
    at /usr/include/c++/13/functional:591
#32 0x0000555555d91ba0 in std::__invoke_impl<void, std::_Bind<void (rerun_viz::Node::*(rerun_viz::Node*))()>>(std::__invoke_other, std::_Bind<void (rerun_viz::Node::*(rerun_viz::Node*))()>&&) (__f=...) at /usr/include/c++/13/bits/invoke.h:61
#33 0x0000555555d91afa in std::__invoke<std::_Bind<void (rerun_viz::Node::*(rerun_viz::Node*))()>>(std::_Bind<void (rerun_viz::Node::*(rerun_viz::Node*))()>&&) (
    __fn=...) at /usr/include/c++/13/bits/invoke.h:96
#34 0x0000555555d91a16 in std::thread::_Invoker<std::tuple<std::_Bind<void (rerun_viz::Node::*(rerun_viz::Node*))()> > >::_M_invoke<0ul>(std::_Index_tuple<0ul>) (
    this=0x555556bc24a8) at /usr/include/c++/13/bits/std_thread.h:292
#35 0x0000555555d9199e in std::thread::_Invoker<std::tuple<std::_Bind<void (rerun_viz::Node::*(rerun_viz::Node*))()> > >::operator()() (this=0x555556bc24a8)
    at /usr/include/c++/13/bits/std_thread.h:299
#36 0x0000555555d9195a in std::thread::_State_impl<std::thread::_Invoker<std::tuple<std::_Bind<void (rerun_viz::Node::*(rerun_viz::Node*))()> > > >::_M_run() (
    this=0x555556bc24a0) at /usr/include/c++/13/bits/std_thread.h:244
#37 0x00007ffff2eecdb4 in std::execute_native_thread_routine (__p=0x555556bc24a0) at ../../../../../src/libstdc++-v3/src/c++11/thread.cc:104
#38 0x00007ffff2a9caa4 in start_thread (arg=<optimized out>) at ./nptl/pthread_create.c:447
#39 0x00007ffff2b29c3c in clone3 () at ../sysdeps/unix/sysv/linux/x86_64/clone3.S:78

  */

  // Force clangformat onto two lines
  stopAndJoinGraphUpdateThread();
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
  auto topicNamesAndTypes = node_->get_topic_names_and_types();

  for (const auto & [topic_name, type_list] : topicNamesAndTypes) {
    std::string types;
    for (const auto & type : type_list) {
      if (!types.empty()) {
        types += ", ";
      }
      types += type;
    }
    RCLCPP_DEBUG(node_->get_logger(), "  %s: %s", topic_name.c_str(), types.c_str());
  }

  updateSubscriptions(topicNamesAndTypes);
}

void Node::graphUpdateThread()
{
  RCLCPP_INFO(node_->get_logger(), "Starting graph update thread");

  while (canGraphUpdateThreadRun()) {
    rclcpp::Event::SharedPtr event = node_->get_graph_event();
    node_->wait_for_graph_change(event, 1s);

    if (!canGraphUpdateThreadRun()) {
      return;
    }

    getTopicsAndUpdateSubscriptions();
  }
}

geometry_msgs::msg::TransformStamped Node::lookupTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time)
{
  return tf_buffer_->lookupTransform(target_frame, source_frame, time);
}

}  // namespace rerun_viz

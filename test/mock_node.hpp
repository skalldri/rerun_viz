#pragma once

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "rclcpp/rclcpp.hpp"

class MockNode : public rclcpp::Node
{
public:
  template <
    typename MessageT, typename CallbackT, typename AllocatorT, typename SubscriptionT,
    typename MessageMemoryStrategyT>
  std::shared_ptr<SubscriptionT> Node::create_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos, CallbackT && callback,
    const SubscriptionOptionsWithAllocator<AllocatorT> & options,
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strat) override
  {
    return std::shared_ptr<SubscriptionT>();
  }
}

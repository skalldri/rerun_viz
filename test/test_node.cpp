// Copyright 2025 Stuart Alldritt
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include <rerun_viz/node.hpp>

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

class NodeTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    // Force clang-format to keep this on two lines
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    // Force clang-format to keep this on two lines
    rclcpp::shutdown();
  }
};

TEST_F(NodeTestFixture, TestSubscriptionsUnsupportedMessageType)
{
  std::shared_ptr<rerun_viz::Node> node = std::make_shared<rerun_viz::Node>(nullptr);

  std::map<std::string, std::vector<std::string>> topicNamesAndTypes;

  topicNamesAndTypes["/my_topic"] =
    std::vector<std::string>({"some_msgs/msg/Nonexistent", "some_msgs/msg/Unsupported"});

  // Should not crash or throw for unsupported topics
  node->updateSubscriptions(topicNamesAndTypes);

  // We should contain an entry for the /my_topic subscription
  auto subscriptions = node->getSubscriptions();
  EXPECT_EQ(subscriptions.size(), 1);
  EXPECT_EQ(subscriptions.count("/my_topic"), 1);
  EXPECT_EQ(subscriptions["/my_topic"].size(), 0);  // Should not have any converters for /my_topic
}

TEST_F(NodeTestFixture, TestSubscriptions)
{
  std::shared_ptr<rerun_viz::Node> node = std::make_shared<rerun_viz::Node>(nullptr);

  // Create a node to publish some fake topics
  auto publisher_node = std::make_shared<rclcpp::Node>("publisher_node");

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node);
  executor.add_node(publisher_node);
  executor.spin_once();

  auto chatter_publisher =
    publisher_node->create_publisher<std_msgs::msg::String>("/chatter", rclcpp::BestAvailableQoS());

  auto pc2_publisher = publisher_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/points", rclcpp::SensorDataQoS());

  // Force an update to the
}

// Test for topics that have multiple types, adding and removing

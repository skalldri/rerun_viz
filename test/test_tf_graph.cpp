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
#include <gmock/gmock.h>

#include <rerun_viz/tf_graph.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using ::testing::ElementsAre;
using ::testing::IsEmpty;
using ::testing::UnorderedElementsAre;

class TFGraphTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create some test transforms
    base_to_link1_ = createTransform("base", "link1", 1.0, 0.0, 0.0);
    link1_to_link2_ = createTransform("link1", "link2", 0.0, 1.0, 0.0);
    link2_to_link3_ = createTransform("link2", "link3", 0.0, 0.0, 1.0);
    base_to_sensor_ = createTransform("base", "sensor", 0.5, 0.5, 0.5);
  }

  geometry_msgs::msg::TransformStamped createTransform(
    const std::string & parent, const std::string & child, double x, double y, double z,
    double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.header.stamp.sec = 123;
    tf.header.stamp.nanosec = 456789;

    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;

    tf.transform.rotation.x = qx;
    tf.transform.rotation.y = qy;
    tf.transform.rotation.z = qz;
    tf.transform.rotation.w = qw;

    return tf;
  }

  rerun_viz::TFGraph graph_;
  geometry_msgs::msg::TransformStamped base_to_link1_;
  geometry_msgs::msg::TransformStamped link1_to_link2_;
  geometry_msgs::msg::TransformStamped link2_to_link3_;
  geometry_msgs::msg::TransformStamped base_to_sensor_;
};

// Test basic transform addition
TEST_F(TFGraphTest, AddTransformBasic)
{
  EXPECT_TRUE(graph_.addTransform(base_to_link1_));

  auto [num_frames, num_transforms] = graph_.getStatistics();
  EXPECT_EQ(num_frames, 2);
  EXPECT_EQ(num_transforms, 1);

  EXPECT_THAT(graph_.getAllFrames(), UnorderedElementsAre("base", "link1"));
}

// Test multiple transform addition
TEST_F(TFGraphTest, AddMultipleTransforms)
{
  EXPECT_TRUE(graph_.addTransform(base_to_link1_));
  EXPECT_TRUE(graph_.addTransform(link1_to_link2_));
  EXPECT_TRUE(graph_.addTransform(link2_to_link3_));

  auto [num_frames, num_transforms] = graph_.getStatistics();
  EXPECT_EQ(num_frames, 4);
  EXPECT_EQ(num_transforms, 3);

  EXPECT_THAT(graph_.getAllFrames(), UnorderedElementsAre("base", "link1", "link2", "link3"));
}

// Test parent-child relationships
TEST_F(TFGraphTest, ParentChildRelationships)
{
  graph_.addTransform(base_to_link1_);
  graph_.addTransform(link1_to_link2_);
  graph_.addTransform(base_to_sensor_);

  // Test getChildren
  EXPECT_THAT(graph_.getChildren("base"), UnorderedElementsAre("link1", "sensor"));
  EXPECT_THAT(graph_.getChildren("link1"), UnorderedElementsAre("link2"));
  EXPECT_THAT(graph_.getChildren("link2"), IsEmpty());
  EXPECT_THAT(graph_.getChildren("nonexistent"), IsEmpty());

  // Test getParent
  EXPECT_EQ(graph_.getParent("link1"), "base");
  EXPECT_EQ(graph_.getParent("link2"), "link1");
  EXPECT_EQ(graph_.getParent("sensor"), "base");
  EXPECT_EQ(graph_.getParent("base"), std::nullopt);
  EXPECT_EQ(graph_.getParent("nonexistent"), std::nullopt);
}

// Test transform retrieval
TEST_F(TFGraphTest, TransformRetrieval)
{
  graph_.addTransform(base_to_link1_);

  auto transform_data = graph_.getTransform("base", "link1");
  ASSERT_TRUE(transform_data.has_value());

  const auto & tf = transform_data->transform;
  EXPECT_EQ(tf.header.frame_id, "base");
  EXPECT_EQ(tf.child_frame_id, "link1");
  EXPECT_DOUBLE_EQ(tf.transform.translation.x, 1.0);
  EXPECT_DOUBLE_EQ(tf.transform.translation.y, 0.0);
  EXPECT_DOUBLE_EQ(tf.transform.translation.z, 0.0);

  // Test nonexistent transform
  EXPECT_EQ(graph_.getTransform("base", "nonexistent"), std::nullopt);
  EXPECT_EQ(graph_.getTransform("nonexistent", "link1"), std::nullopt);
}

// Test cycle detection
TEST_F(TFGraphTest, CycleDetection)
{
  // Build a chain: base -> link1 -> link2
  EXPECT_TRUE(graph_.addTransform(base_to_link1_));
  EXPECT_TRUE(graph_.addTransform(link1_to_link2_));

  // Try to create a cycle: link2 -> base
  auto cycle_transform = createTransform("link2", "base", 0.0, 0.0, 0.0);
  EXPECT_FALSE(graph_.addTransform(cycle_transform));

  // Verify the cycle wasn't added
  auto [num_frames, num_transforms] = graph_.getStatistics();
  EXPECT_EQ(num_transforms, 2);
  EXPECT_EQ(graph_.getParent("base"), std::nullopt);
}

// Test self-loop detection
TEST_F(TFGraphTest, SelfLoopDetection)
{
  auto self_loop = createTransform("base", "base", 0.0, 0.0, 0.0);
  EXPECT_FALSE(graph_.addTransform(self_loop));

  auto [num_frames, num_transforms] = graph_.getStatistics();
  EXPECT_EQ(num_transforms, 0);
}

// Test wouldCreateCycle method
TEST_F(TFGraphTest, WouldCreateCycleMethod)
{
  graph_.addTransform(base_to_link1_);
  graph_.addTransform(link1_to_link2_);

  // Should detect potential cycles
  EXPECT_TRUE(graph_.wouldCreateCycle("link2", "base"));
  EXPECT_TRUE(graph_.wouldCreateCycle("link2", "link1"));
  EXPECT_TRUE(graph_.wouldCreateCycle("link1", "base"));

  // Should not detect cycles for valid additions
  EXPECT_FALSE(graph_.wouldCreateCycle("link2", "link3"));
  EXPECT_FALSE(graph_.wouldCreateCycle("base", "sensor"));

  // Self-loop should be detected
  EXPECT_TRUE(graph_.wouldCreateCycle("base", "base"));
}

// Test transform replacement
TEST_F(TFGraphTest, TransformReplacement)
{
  // Add initial transform
  graph_.addTransform(base_to_link1_);
  EXPECT_EQ(graph_.getParent("link1"), "base");

  // Add a new transform that replaces the parent of link1
  auto new_parent_to_link1 = createTransform("new_parent", "link1", 2.0, 3.0, 4.0);
  EXPECT_TRUE(graph_.addTransform(new_parent_to_link1));

  // Verify the parent has been updated
  EXPECT_EQ(graph_.getParent("link1"), "new_parent");
  EXPECT_THAT(graph_.getChildren("base"), IsEmpty());
  EXPECT_THAT(graph_.getChildren("new_parent"), UnorderedElementsAre("link1"));

  // Verify the transform data is updated
  auto transform_data = graph_.getTransform("new_parent", "link1");
  ASSERT_TRUE(transform_data.has_value());
  EXPECT_DOUBLE_EQ(transform_data->transform.transform.translation.x, 2.0);
}

// Test path finding
TEST_F(TFGraphTest, PathFinding)
{
  // Build chain: base -> link1 -> link2 -> link3
  graph_.addTransform(base_to_link1_);
  graph_.addTransform(link1_to_link2_);
  graph_.addTransform(link2_to_link3_);

  // Add branch: base -> sensor
  graph_.addTransform(base_to_sensor_);

  // Test direct paths
  EXPECT_THAT(graph_.getPath("base", "link3"), ElementsAre("base", "link1", "link2", "link3"));
  EXPECT_THAT(graph_.getPath("link1", "link3"), ElementsAre("link1", "link2", "link3"));

  // Test reverse paths
  EXPECT_THAT(graph_.getPath("link3", "base"), ElementsAre("link3", "link2", "link1", "base"));

  // Test branch paths
  EXPECT_THAT(graph_.getPath("sensor", "link2"), ElementsAre("sensor", "base", "link1", "link2"));
  EXPECT_THAT(graph_.getPath("link2", "sensor"), ElementsAre("link2", "link1", "base", "sensor"));

  // Test same frame
  EXPECT_THAT(graph_.getPath("base", "base"), ElementsAre("base"));

  // Test nonexistent paths
  EXPECT_THAT(graph_.getPath("base", "nonexistent"), IsEmpty());
  EXPECT_THAT(graph_.getPath("nonexistent", "base"), IsEmpty());
}

// Test transform removal
TEST_F(TFGraphTest, TransformRemoval)
{
  graph_.addTransform(base_to_link1_);
  graph_.addTransform(link1_to_link2_);
  graph_.addTransform(base_to_sensor_);

  // Remove middle transform
  graph_.removeTransform("link1", "link2");

  EXPECT_EQ(graph_.getParent("link2"), std::nullopt);
  EXPECT_THAT(graph_.getChildren("link1"), IsEmpty());
  EXPECT_EQ(graph_.getTransform("link1", "link2"), std::nullopt);

  // Other transforms should remain
  EXPECT_EQ(graph_.getParent("link1"), "base");
  EXPECT_EQ(graph_.getParent("sensor"), "base");

  auto [num_frames, num_transforms] = graph_.getStatistics();
  EXPECT_EQ(num_transforms, 2);
}

// Test empty graph
TEST_F(TFGraphTest, EmptyGraph)
{
  auto [num_frames, num_transforms] = graph_.getStatistics();
  EXPECT_EQ(num_frames, 0);
  EXPECT_EQ(num_transforms, 0);

  EXPECT_THAT(graph_.getAllFrames(), IsEmpty());
  EXPECT_THAT(graph_.getChildren("any"), IsEmpty());
  EXPECT_EQ(graph_.getParent("any"), std::nullopt);
  EXPECT_EQ(graph_.getTransform("any", "other"), std::nullopt);
  EXPECT_THAT(graph_.getPath("any", "other"), IsEmpty());
}

// Test complex tree structure
TEST_F(TFGraphTest, ComplexTreeStructure)
{
  // Create a more complex tree:
  //       base
  //      /    \\
  //   link1   sensor1
  //   /  \\       \\
  // link2 link3  sensor2

  auto base_to_link1 = createTransform("base", "link1", 1, 0, 0);
  auto base_to_sensor1 = createTransform("base", "sensor1", 0, 1, 0);
  auto link1_to_link2 = createTransform("link1", "link2", 0, 0, 1);
  auto link1_to_link3 = createTransform("link1", "link3", 1, 1, 0);
  auto sensor1_to_sensor2 = createTransform("sensor1", "sensor2", 0, 0, 2);

  EXPECT_TRUE(graph_.addTransform(base_to_link1));
  EXPECT_TRUE(graph_.addTransform(base_to_sensor1));
  EXPECT_TRUE(graph_.addTransform(link1_to_link2));
  EXPECT_TRUE(graph_.addTransform(link1_to_link3));
  EXPECT_TRUE(graph_.addTransform(sensor1_to_sensor2));

  // Test statistics
  auto [num_frames, num_transforms] = graph_.getStatistics();
  EXPECT_EQ(num_frames, 6);
  EXPECT_EQ(num_transforms, 5);

  // Test parent-child relationships
  EXPECT_THAT(graph_.getChildren("base"), UnorderedElementsAre("link1", "sensor1"));
  EXPECT_THAT(graph_.getChildren("link1"), UnorderedElementsAre("link2", "link3"));
  EXPECT_THAT(graph_.getChildren("sensor1"), UnorderedElementsAre("sensor2"));

  // Test paths across branches
  EXPECT_THAT(
    graph_.getPath("link2", "sensor2"),
    ElementsAre("link2", "link1", "base", "sensor1", "sensor2"));
  EXPECT_THAT(graph_.getPath("link3", "link2"), ElementsAre("link3", "link1", "link2"));
}

// Test logToRerun (basic functionality test - we can't easily test the actual Rerun logging)
TEST_F(TFGraphTest, LogToRerunBasic)
{
  graph_.addTransform(base_to_link1_);

  // Test with null recording stream (should not crash)
  EXPECT_NO_THROW(graph_.logToRerun(nullptr));

  // We can't easily test the actual Rerun logging without mocking,
  // but we can at least verify the method doesn't crash with a valid graph
  // In a real scenario, you might want to use dependency injection or mocking
  // for the RecordingStream to test the actual logging behavior
}

// Test getPathToRoot method
TEST_F(TFGraphTest, GetPathToRoot)
{
  // Build chain: base -> link1 -> link2 -> link3
  graph_.addTransform(base_to_link1_);
  graph_.addTransform(link1_to_link2_);
  graph_.addTransform(link2_to_link3_);

  // Add branch: base -> sensor
  graph_.addTransform(base_to_sensor_);

  // Test paths to root (should be from frame to root)
  EXPECT_THAT(graph_.getPathToRoot("base"), ElementsAre("base"));  // Root has no parent
  EXPECT_THAT(graph_.getPathToRoot("link1"), ElementsAre("link1", "base"));
  EXPECT_THAT(graph_.getPathToRoot("link2"), ElementsAre("link2", "link1", "base"));
  EXPECT_THAT(graph_.getPathToRoot("link3"), ElementsAre("link3", "link2", "link1", "base"));
  EXPECT_THAT(graph_.getPathToRoot("sensor"), ElementsAre("sensor", "base"));

  // Test nonexistent frame
  EXPECT_THAT(graph_.getPathToRoot("nonexistent"), ElementsAre("nonexistent"));
}

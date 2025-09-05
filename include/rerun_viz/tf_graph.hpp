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

#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <optional>
#include <memory>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rerun.hpp>

namespace rerun_viz
{

class TFGraph
{
public:
  struct TransformData
  {
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Time timestamp;

    // Default constructor
    TransformData() : timestamp(rclcpp::Time(0, 0, RCL_ROS_TIME)) {}

    TransformData(const geometry_msgs::msg::TransformStamped & tf)
    : transform(tf), timestamp(rclcpp::Time(tf.header.stamp))
    {
    }
  };

  TFGraph() = default;
  ~TFGraph() = default;

  /// Add or update a transform in the graph
  /// @param tf The transform to add
  /// @return true if successfully added, false if it would create a cycle
  bool addTransform(const geometry_msgs::msg::TransformStamped & tf);

  /// Remove a transform from the graph
  /// @param parent The parent frame ID
  /// @param child The child frame ID
  void removeTransform(const std::string & parent, const std::string & child);

  /// Get all children of a frame
  /// @param frame The frame ID
  /// @return Vector of child frame IDs
  std::vector<std::string> getChildren(const std::string & frame) const;

  /// Get the parent of a frame
  /// @param frame The frame ID
  /// @return The parent frame ID, or nullopt if no parent
  std::optional<std::string> getParent(const std::string & frame) const;

  /// Get all frame IDs in the graph
  /// @return Set of all frame IDs
  std::unordered_set<std::string> getAllFrames() const;

  /// Find path from one frame to another
  /// @param from Source frame ID
  /// @param to Target frame ID
  /// @return Vector representing the path, empty if no path exists
  std::vector<std::string> getPath(const std::string & from, const std::string & to) const;

  /// Get transform data between two frames
  /// @param parent Parent frame ID
  /// @param child Child frame ID
  /// @return Transform data if it exists
  std::optional<TransformData> getTransform(
    const std::string & parent, const std::string & child) const;

  /// Check if the graph would have a cycle if we added this transform
  /// @param parent Parent frame ID
  /// @param child Child frame ID
  /// @return true if adding this transform would create a cycle
  bool wouldCreateCycle(const std::string & parent, const std::string & child) const;

  /// Log the entire TF tree to Rerun
  /// @param rec Rerun recording stream
  void logToRerun(std::shared_ptr<rerun::RecordingStream> rec) const;

  /// Get statistics about the graph
  /// @return Pair of (number of frames, number of transforms)
  std::pair<size_t, size_t> getStatistics() const;

  /// Get the path from a frame to its root
  /// @param frame The frame to find path for
  /// @return Vector of frame names from frame to root (frame first, root last)
  std::vector<std::string> getPathToRoot(const std::string & frame) const;

private:
  // Adjacency list: parent_frame -> list of child_frames
  std::unordered_map<std::string, std::vector<std::string>> children_;

  // Reverse lookup: child_frame -> parent_frame
  std::unordered_map<std::string, std::string> parents_;

  // Transform data: "parent->child" -> TransformData
  std::unordered_map<std::string, TransformData> transforms_;

  /// Create a key for transform storage
  /// @param parent Parent frame ID
  /// @param child Child frame ID
  /// @return String key for the transform
  std::string makeTransformKey(const std::string & parent, const std::string & child) const;

  /// Depth-first search to check for cycles
  /// @param current Current frame being visited
  /// @param target Target frame we're looking for
  /// @param visited Set of already visited frames
  /// @return true if target is reachable from current
  bool dfsHasPath(
    const std::string & current, const std::string & target,
    std::unordered_set<std::string> & visited) const;

  /// Log a single transform to Rerun
  /// @param rec Rerun recording stream
  /// @param parent Parent frame ID
  /// @param child Child frame ID
  /// @param transform_data Transform data to log
  void logTransformToRerun(
    std::shared_ptr<rerun::RecordingStream> rec, const std::string & parent,
    const std::string & child, const TransformData & transform_data) const;
};

}  // namespace rerun_viz

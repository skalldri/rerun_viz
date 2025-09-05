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

#include <rerun_viz/tf_graph.hpp>

#include <queue>
#include <algorithm>

namespace rerun_viz
{

bool TFGraph::addTransform(const geometry_msgs::msg::TransformStamped & tf)
{
  const std::string & parent = tf.header.frame_id;
  const std::string & child = tf.child_frame_id;

  // Check if adding this transform would create a cycle
  if (wouldCreateCycle(parent, child)) {
    return false;
  }

  // Remove existing parent-child relationship for this child if it exists
  auto parent_it = parents_.find(child);
  if (parent_it != parents_.end()) {
    const std::string & old_parent = parent_it->second;

    // Remove child from old parent's children list
    auto & old_parent_children = children_[old_parent];
    old_parent_children.erase(
      std::remove(old_parent_children.begin(), old_parent_children.end(), child),
      old_parent_children.end());

    // Remove old transform data
    transforms_.erase(makeTransformKey(old_parent, child));
  }

  // Add new relationship
  parents_[child] = parent;
  children_[parent].push_back(child);

  // Store transform data
  const std::string key = makeTransformKey(parent, child);
  transforms_[key] = TransformData(tf);

  return true;
}

void TFGraph::removeTransform(const std::string & parent, const std::string & child)
{
  // Remove from parent's children list
  auto parent_it = children_.find(parent);
  if (parent_it != children_.end()) {
    auto & parent_children = parent_it->second;
    parent_children.erase(
      std::remove(parent_children.begin(), parent_children.end(), child), parent_children.end());

    // Remove parent entry if no children left
    if (parent_children.empty()) {
      children_.erase(parent_it);
    }
  }

  // Remove from child's parent
  parents_.erase(child);

  // Remove transform data
  transforms_.erase(makeTransformKey(parent, child));
}

std::vector<std::string> TFGraph::getChildren(const std::string & frame) const
{
  auto it = children_.find(frame);
  if (it != children_.end()) {
    return it->second;
  }
  return {};
}

std::optional<std::string> TFGraph::getParent(const std::string & frame) const
{
  auto it = parents_.find(frame);
  if (it != parents_.end()) {
    return it->second;
  }
  return std::nullopt;
}

std::unordered_set<std::string> TFGraph::getAllFrames() const
{
  std::unordered_set<std::string> frames;

  // Add all parents
  for (const auto & [parent, children] : children_) {
    frames.insert(parent);
    // Add all children
    for (const auto & child : children) {
      frames.insert(child);
    }
  }

  return frames;
}

std::vector<std::string> TFGraph::getPath(const std::string & from, const std::string & to) const
{
  if (from == to) {
    return {from};
  }

  // BFS to find shortest path
  std::queue<std::string> queue;
  std::unordered_map<std::string, std::string> came_from;
  std::unordered_set<std::string> visited;

  queue.push(from);
  visited.insert(from);

  while (!queue.empty()) {
    std::string current = queue.front();
    queue.pop();

    // Check children (going down the tree)
    auto children_it = children_.find(current);
    if (children_it != children_.end()) {
      for (const auto & child : children_it->second) {
        if (visited.find(child) == visited.end()) {
          visited.insert(child);
          came_from[child] = current;
          queue.push(child);

          if (child == to) {
            // Reconstruct path
            std::vector<std::string> path;
            std::string step = to;
            while (step != from) {
              path.push_back(step);
              step = came_from[step];
            }
            path.push_back(from);
            std::reverse(path.begin(), path.end());
            return path;
          }
        }
      }
    }

    // Check parent (going up the tree)
    auto parent_it = parents_.find(current);
    if (parent_it != parents_.end()) {
      const std::string & parent = parent_it->second;
      if (visited.find(parent) == visited.end()) {
        visited.insert(parent);
        came_from[parent] = current;
        queue.push(parent);

        if (parent == to) {
          // Reconstruct path
          std::vector<std::string> path;
          std::string step = to;
          while (step != from) {
            path.push_back(step);
            step = came_from[step];
          }
          path.push_back(from);
          std::reverse(path.begin(), path.end());
          return path;
        }
      }
    }
  }

  return {};  // No path found
}

std::optional<TFGraph::TransformData> TFGraph::getTransform(
  const std::string & parent, const std::string & child) const
{
  const std::string key = makeTransformKey(parent, child);
  auto it = transforms_.find(key);
  if (it != transforms_.end()) {
    return it->second;
  }
  return std::nullopt;
}

bool TFGraph::wouldCreateCycle(const std::string & parent, const std::string & child) const
{
  // If child is the same as parent, it's a self-loop
  if (parent == child) {
    return true;
  }

  // Check if parent is reachable from child (which would create a cycle)
  std::unordered_set<std::string> visited;
  return dfsHasPath(child, parent, visited);
}

void TFGraph::logToRerun(std::shared_ptr<rerun::RecordingStream> rec) const
{
  if (!rec) {
    return;
  }

  // Log all transforms
  for (const auto & [key, transform_data] : transforms_) {
    // Parse the key to get parent and child
    size_t arrow_pos = key.find("->");
    if (arrow_pos != std::string::npos) {
      std::string parent = key.substr(0, arrow_pos);
      std::string child = key.substr(arrow_pos + 2);
      logTransformToRerun(rec, parent, child, transform_data);
    }
  }
}

std::pair<size_t, size_t> TFGraph::getStatistics() const
{
  return {getAllFrames().size(), transforms_.size()};
}

std::string TFGraph::makeTransformKey(const std::string & parent, const std::string & child) const
{
  return parent + "->" + child;
}

std::vector<std::string> TFGraph::getPathToRoot(const std::string & frame) const
{
  std::vector<std::string> path;
  std::string current = frame;

  // Walk up the tree to the root
  while (true) {
    path.push_back(current);
    auto parent_opt = getParent(current);
    if (!parent_opt.has_value()) {
      // Reached root
      break;
    }
    current = parent_opt.value();
  }

  return path;
}

bool TFGraph::dfsHasPath(
  const std::string & current, const std::string & target,
  std::unordered_set<std::string> & visited) const
{
  if (current == target) {
    return true;
  }

  if (visited.find(current) != visited.end()) {
    return false;  // Already visited, avoid infinite loop
  }

  visited.insert(current);

  // Check all children
  auto children_it = children_.find(current);
  if (children_it != children_.end()) {
    for (const auto & child : children_it->second) {
      if (dfsHasPath(child, target, visited)) {
        return true;
      }
    }
  }

  return false;
}

void TFGraph::logTransformToRerun(
  std::shared_ptr<rerun::RecordingStream> rec, const std::string & /* parent */,
  const std::string & child, const TransformData & transform_data) const
{
  const auto & tf = transform_data.transform.transform;

  // Build the full path from root to child
  std::string full_path = "TF";
  auto path_to_child = getPathToRoot(child);

  // Reverse the path since getPathToRoot returns child->root, but we want root->child
  std::reverse(path_to_child.begin(), path_to_child.end());

  for (const auto & frame : path_to_child) {
    full_path += "/" + frame;
  }

  rec->log(
    full_path, rerun::Transform3D::from_translation({static_cast<float>(tf.translation.x),
                                                     static_cast<float>(tf.translation.y),
                                                     static_cast<float>(tf.translation.z)})
                 .with_relation(
                   rerun::components::TransformRelation::
                     ParentFromChild /*rerun::components::TransformRelation::ChildFromParent*/)
                 .with_axis_length(0.1f)
                 .with_quaternion(
                   rerun::datatypes::Quaternion::from_xyzw(
                     tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w)));
}

}  // namespace rerun_viz

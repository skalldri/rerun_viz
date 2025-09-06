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

#include <rerun_viz/msg_conversion/special_msgs/robot_description.hpp>
#include <rerun_viz/utils.hpp>
#include <urdf/model.h>

#include <cstddef>

using std::placeholders::_1;

namespace rerun_viz
{

RobotDescription::RobotDescription(
  std::shared_ptr<rerun_viz::Node> node, const std::string & topic_name,
  std::shared_ptr<rerun::RecordingStream> rec)
: node_(node), rec_(rec), topic_name_(topic_name)
{
  subscription_ = node_->getRosNode()->create_subscription<std_msgs::msg::String>(
    topic_name_, rclcpp::BestAvailableQoS(),
    std::bind(&RobotDescription::topic_callback, this, _1));
}

void RobotDescription::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (!rec_) {
    RCLCPP_WARN(
      node_->getRosNode()->get_logger(), "No valid RecordingStream, cannot visualize data.");
    return;
  }

  rec_->log(topic_name_ + "/RobotDescription", rerun::TextDocument(msg->data));

  urdf::Model model;
  model.initString(msg->data);

  // ReRun's URDF loader loads like this:
  //     /<robot_name>/<root_link_name>
  //                                   /<joint_name>/<child_link_name>
  //
  // So we need to publish transforms like (FROM -> TO : TF TYPE @ ReRun Entity Path):
  //  <fixed_frame> -> <root_link_name> : ROTATION+TRANSLATION @ /<robot_name>/<root_link_name>
  //  <root_link_name> -> <child_link_name> : ROTATION+ONLY @ /<robot_name>/<root_link_name>
  //
  // TODO: this probably works for "revolute" and "continuous" joints, but there are other types.
  // Need to figure them out.

  {
    std::string rerun_path = "/" + model.getName() + "/" + model.getRoot()->name;

    std::lock_guard lock(tf_requests_mutex_);
    tf_requests_.clear();

    // Special case: handle the root link, which needs to be relative to the fixed frame
    // TODO: once we support changing the fixed-frame at runtime, this won't work anymore.
    // Maybe we can have a "magic" frame name that means "use the current fixed frame"?
    TFRequest req;
    req.ros_parent_frame_id = node_->getFixedFrameId();
    req.ros_child_frame_id = model.getRoot()->name;
    req.rerun_entity_path = rerun_path;
    req.tf_type = TFRequestType::TranslationAndRotation;
    tf_requests_.push_back(req);

    // Now DFS the rest of the URDF to find all the joints and links
    urdfDepthFirst(model, "/" + model.getName(), model.getRoot());
  }

  try {
    rec_->log_file_from_contents(
      "robot.urdf", reinterpret_cast<const std::byte *>(msg->data.c_str()),
      std::strlen(msg->data.c_str()), topic_name_ + "/RobotDescription");

  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->getRosNode()->get_logger(), "Failed to log robot description: %s", e.what());
  }
}

void RobotDescription::urdfDepthFirst(
  urdf::Model & model, std::string root, urdf::JointConstSharedPtr joint)
{
  RCLCPP_INFO(node_->getRosNode()->get_logger(), "URDF Joint: Root: %s", root.c_str());
  root = root + "/" + joint->name;
  RCLCPP_INFO(node_->getRosNode()->get_logger(), "URDF Joint: Root + self: %s", root.c_str());

  if (joint->type != urdf::Joint::CONTINUOUS && joint->type != urdf::Joint::REVOLUTE) {
    // TODO: support other joint types
    // timebomb to avoid suprises
    throw std::runtime_error(
      "Unsupported joint type in URDF: only CONTINUOUS and REVOLUTE are supported");
  }

  // We need to publish the TF on the re-run entity representing the child link
  std::string rerun_entity = root + "/" + joint->child_link_name;

  RCLCPP_INFO(
    node_->getRosNode()->get_logger(),
    "URDF TF NEEDED: %s -> %s, type ROTATION ONLY @ rerun entity path %s",
    joint->parent_link_name.c_str(), joint->child_link_name.c_str(), rerun_entity.c_str());

  TFRequest req;
  req.ros_parent_frame_id = joint->parent_link_name;
  req.ros_child_frame_id = joint->child_link_name;
  req.rerun_entity_path = rerun_entity;
  req.tf_type = TFRequestType::RotationOnly;

  {
    std::lock_guard lock(tf_requests_mutex_);
    tf_requests_.push_back(req);
  }

  // Resolve the next child link in the chain...
  auto child_link = model.getLink(joint->child_link_name);

  // DFS into the direct child link of this joint. There is only ever exactly one child link of a joint.
  urdfDepthFirst(model, root, child_link);
}

void RobotDescription::urdfDepthFirst(
  urdf::Model & model, std::string root, urdf::LinkConstSharedPtr link)
{
  RCLCPP_INFO(node_->getRosNode()->get_logger(), "URDF Link: Root: %s", root.c_str());
  root = root + "/" + link->name;
  RCLCPP_INFO(node_->getRosNode()->get_logger(), "URDF Link: Root + self: %s", root.c_str());

  // First DFS into each link...
  // TODO: doesn't seem to produce correct results. Skip child_links for now.
  // for (const auto & l : link->child_links) {
  //   urdfDepthFirst(model, root, l);
  // }

  // Now the joints...
  for (const auto & j : link->child_joints) {
    urdfDepthFirst(model, root, j);
  }
}

const std::vector<TFRequest> RobotDescription::getTfRequests()
{
  std::lock_guard lock(tf_requests_mutex_);
  return tf_requests_;
}

}  // namespace rerun_viz

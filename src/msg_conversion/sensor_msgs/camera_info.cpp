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

#include <rerun_viz/msg_conversion/sensor_msgs/camera_info.hpp>
#include <rerun_viz/msg_conversion/tf2_msgs/tf_message.hpp>
#include <rerun_viz/utils.hpp>

#include <tf2/time.hpp>

using std::placeholders::_1;

namespace rerun_viz
{

CameraInfo::CameraInfo(
  std::shared_ptr<rerun_viz::Node> node, const std::string & topic_name,
  std::shared_ptr<rerun::RecordingStream> rec)
: node_(node), rec_(rec), topic_name_(topic_name)
{
  subscription_ = node_->getRosNode()->create_subscription<sensor_msgs::msg::CameraInfo>(
    topic_name_, rclcpp::SensorDataQoS(), std::bind(&CameraInfo::topic_callback, this, _1));
}

void CameraInfo::topic_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) const
{
  if (!rec_) {
    RCLCPP_WARN(
      node_->getRosNode()->get_logger(), "No valid RecordingStream, cannot visualize data.");
    return;
  }

  try {
    std::string cameraNamespace = rerun_viz::getCameraNamespaceFromTopic(topic_name_);

    // Check if camera is calibrated (K[0] == 0.0 indicates uncalibrated camera)
    if (msg->k[0] == 0.0) {
      RCLCPP_WARN(
        node_->getRosNode()->get_logger(),
        "Camera '%s' appears to be uncalibrated (K[0] = 0.0), skipping intrinsics visualization.",
        cameraNamespace.c_str());
      return;
    }

    // TODO: fix timestamping
    geometry_msgs::msg::TransformStamped t = node_->lookupTransform(
      node_->getFixedFrameId(), msg->header.frame_id, tf2::TimePointZero /*msg->header.stamp*/);

    // Try to get the TF transform for the camera frame
    std::optional<rerun::Transform3D> camera_transform = convertTransformToRerun(t.transform);

    // Extract camera intrinsics from K matrix
    // K = [fx  0 cx]
    //     [ 0 fy cy]
    //     [ 0  0  1]
    const double fx = msg->k[0];  // Focal length X
    const double fy = msg->k[4];  // Focal length Y
    const double cx = msg->k[2];  // Principal point X
    const double cy = msg->k[5];  // Principal point Y

    // Create the intrinsics matrix in the format expected by Rerun
    std::array<float, 9> intrinsics_matrix = {
      static_cast<float>(fx),
      0.0f,
      static_cast<float>(cx),
      0.0f,
      static_cast<float>(fy),
      static_cast<float>(cy),
      0.0f,
      0.0f,
      1.0f};

    // Create the pinhole projection
    auto pinhole_projection =
      rerun::components::PinholeProjection::from_mat3x3(intrinsics_matrix.data());

    // Create resolution
    auto resolution = rerun::components::Resolution(
      std::array<float, 2>{static_cast<float>(msg->width), static_cast<float>(msg->height)});

    // Create the pinhole archetype
    auto pinhole =
      rerun::archetypes::Pinhole()
        .with_image_from_camera(pinhole_projection)
        .with_resolution(resolution)
        .with_camera_xyz(
          rerun::components::ViewCoordinates::RDF)  // ROS standard: X=Right, Y=Down, Z=Forward
        .with_image_plane_distance(1.0);

    // If we have a transform, log it along with the pinhole
    if (camera_transform.has_value()) {
      rec_->log(cameraNamespace, camera_transform.value());
    }

    // Log the pinhole camera model to Rerun
    rec_->log(cameraNamespace, pinhole);

    RCLCPP_DEBUG(
      node_->getRosNode()->get_logger(),
      "Logged camera intrinsics for '%s': fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, resolution=%dx%d",
      cameraNamespace.c_str(), fx, fy, cx, cy, msg->width, msg->height);

  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node_->getRosNode()->get_logger(), "Failed to get camera namespace from topic %s: %s",
      topic_name_.c_str(), e.what());
  }
}

}  // namespace rerun_viz

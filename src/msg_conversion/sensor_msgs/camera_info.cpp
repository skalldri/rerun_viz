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
#include <image_geometry/pinhole_camera_model.hpp>

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

void CameraInfo::topic_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!rec_) {
    RCLCPP_WARN(
      node_->getRosNode()->get_logger(), "No valid RecordingStream, cannot visualize data.");
    return;
  }

  try {
    std::string entityPath = rerun_viz::getCameraNamespaceFromTopic(topic_name_);

    // Check if camera is calibrated (K[0] == 0.0 indicates uncalibrated camera)
    if (msg->k[0] == 0.0) {
      RCLCPP_WARN(
        node_->getRosNode()->get_logger(),
        "Camera '%s' appears to be uncalibrated (K[0] = 0.0), skipping intrinsics visualization.",
        entityPath.c_str());
      return;
    }

    tf_request_.emplace(
      node_->getFixedFrameId(), std::string(msg->header.frame_id), entityPath,
      TFRequestType::TranslationAndRotation);

    image_geometry::PinholeCameraModel cam_model;
    if (!cam_model.fromCameraInfo(*msg)) {
      RCLCPP_WARN(
        node_->getRosNode()->get_logger(), "Failed to load pinhole camera model for %s.",
        entityPath.c_str());
    }

    const cv::Matx33d & intrinsics = cam_model.fullIntrinsicMatrix();

    // Create the intrinsics matrix in the format expected by Rerun
    // Rerun expects column-major order
    std::array<float, 9> intrinsics_matrix = {
      static_cast<float>(intrinsics(0, 0)), static_cast<float>(intrinsics(1, 0)),
      static_cast<float>(intrinsics(2, 0)), static_cast<float>(intrinsics(0, 1)),
      static_cast<float>(intrinsics(1, 1)), static_cast<float>(intrinsics(2, 1)),
      static_cast<float>(intrinsics(0, 2)), static_cast<float>(intrinsics(1, 2)),
      static_cast<float>(intrinsics(2, 2))};

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

    // Log the pinhole camera model to Rerun
    rec_->log(entityPath, pinhole);

    // RCLCPP_DEBUG(
    //   node_->getRosNode()->get_logger(),
    //   "Logged camera intrinsics for '%s': fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, resolution=%dx%d",
    //   entityPath.c_str(), fx, fy, cx, cy, msg->width, msg->height);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      node_->getRosNode()->get_logger(), "Failed to get camera namespace from topic %s: %s",
      topic_name_.c_str(), e.what());
  }
}

const std::vector<TFRequest> CameraInfo::getTfRequests()
{
  if (tf_request_.has_value()) {
    return std::vector<TFRequest>({tf_request_.value()});
  }

  return std::vector<TFRequest>();
}

}  // namespace rerun_viz

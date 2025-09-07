#pragma once

#include <string>

namespace rerun_viz
{

enum class TFRequestType
{
  TranslationAndRotation,  // Get a TF that includes both translation and rotation components
  RotationOnly,  // Get a TF that includes only rotation, no translation. Useful for URDF revolute / continuous joints
};

struct TFRequest
{
  TFRequest() = delete;

  TFRequest(
    std::string ros_parent_frame_id, std::string ros_child_frame_id, std::string rerun_entity_path,
    TFRequestType tf_type)
  : ros_parent_frame_id(ros_parent_frame_id),
    ros_child_frame_id(ros_child_frame_id),
    rerun_entity_path(rerun_entity_path),
    tf_type(tf_type) {};

  std::string ros_parent_frame_id;
  std::string ros_child_frame_id;
  std::string rerun_entity_path;
  TFRequestType tf_type;
};

}  // namespace rerun_viz

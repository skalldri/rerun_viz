#include <rerun_viz/utils.hpp>

#include <stdexcept>
#include <algorithm>

namespace rerun_viz
{

bool isCameraTopic(const std::string & topic_name, const std::vector<std::string> & topic_types)
{
  // Special types that indicate a camera topic
  std::vector<std::string> camera_types = {"sensor_msgs/msg/Image", "sensor_msgs/msg/CameraInfo"};

  // If any of the topic types match the known camera-types, we will consider this to be a camera topic
  if (std::any_of(topic_types.begin(), topic_types.end(), [camera_types](const std::string & type) {
        return std::find(camera_types.begin(), camera_types.end(), type) != camera_types.end();
      })) {
    return true;
  }

  return false;
}

std::string getCameraNamespaceFromTopic(const std::string & topic_name)
{
  // Camera topics are expected to be published under a common namespace. Ex:
  //
  // /camera1/image_raw
  // /camera1/camera_info
  //
  // In this example, the camera namespace is "/camera1"
  //
  // Topics don't necessarily start with a "/". We should respect this and return the namespace
  // in the same format as the topic name.
  auto last_slash = topic_name.find_last_of('/');

  // Could not find any slashes in the topic name
  if (last_slash == std::string::npos) {
    throw std::runtime_error("Cannot determine camera namespace from topic: " + topic_name);
  }

  // The only slash is the first one, so there is no namespace
  if (last_slash == 0) {
    throw std::runtime_error("Cannot determine camera namespace from topic: " + topic_name);
  }

  return topic_name.substr(0, last_slash);
}

bool isInNamespace(const std::string & topic, const std::string & ns)
{
  // Remove trailing '/' from namespace if it exists
  if (ns.back() == '/') {
    return isInNamespace(topic, ns.substr(0, ns.size() - 1));
  }

  if (ns.empty() || ns == "/") {
    return true;  // Everything is in the global namespace
  }

  if (topic == ns) {
    return true;  // Exact match
  }

  // Topics within a namespace must be longer than the namespace itself
  if (topic.size() <= ns.size()) {
    return false;
  }

  // A topic is within a namespace if
  if (topic.substr(0, ns.size()) == ns) {
    // Ensure the next character is a '/' to avoid partial matches
    return topic[ns.size()] == '/';
  }
  return false;
}

// Helper function to convert a geometry_msgs Transform to rerun Transform3D
rerun::Transform3D convertTransformToRerun(const geometry_msgs::msg::TransformStamped & tf)
{
  return convertTransformToRerun(tf.transform);
}

// Helper function to convert a geometry_msgs Transform to rerun Transform3D
rerun::Transform3D convertTransformToRerun(const geometry_msgs::msg::Transform & tf)
{
  return rerun::Transform3D::from_translation({static_cast<float>(tf.translation.x),
                                               static_cast<float>(tf.translation.y),
                                               static_cast<float>(tf.translation.z)})
    .with_quaternion(
      rerun::datatypes::Quaternion::from_xyzw(
        tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w))
    .with_relation(rerun::components::TransformRelation::ParentFromChild);
}

}  // namespace rerun_viz

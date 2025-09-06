#pragma once

#include <string>
#include <vector>

#include <rerun.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace rerun_viz
{

bool isCameraTopic(const std::string & topic_name, const std::vector<std::string> & topic_types);

std::string getCameraNamespaceFromTopic(const std::string & topic_name);

bool isInNamespace(const std::string & topic, const std::string & ns);

rerun::Transform3D convertTransformToRerun(const geometry_msgs::msg::TransformStamped & tf);
rerun::Transform3D convertTransformToRerun(const geometry_msgs::msg::Transform & tf);

}  // namespace rerun_viz

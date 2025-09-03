#pragma once

#include <string>
#include <vector>

namespace rerun_viz
{

bool isCameraTopic(const std::string & topic_name, const std::vector<std::string> & topic_types);

std::string getCameraNamespaceFromTopic(const std::string & topic_name);

bool isInNamespace(const std::string & topic, const std::string & ns);

}  // namespace rerun_viz

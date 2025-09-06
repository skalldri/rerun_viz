#pragma once

#include <rerun_viz/tf_request.hpp>
#include <string>
#include <vector>

namespace rerun_viz
{

class Converter
{
public:
  virtual const std::string getRosTypeName() = 0;

  /**
   * @brief Query if the converter would like any TF data to be published to ReRun.
   * 
   * This is an optional function, and the default implementation returns an empty list.
   * 
   * @return const std::vector<TFRequest> a vector of TFRequests that the converter would like published to ReRun
   */
  virtual const std::vector<TFRequest> getTfRequests() { return std::vector<TFRequest>(); };
};

}  // namespace rerun_viz

#pragma once

namespace rerun_viz
{

class Converter
{
public:
  virtual const std::string getRosTypeName() = 0;
};

}  // namespace rerun_viz

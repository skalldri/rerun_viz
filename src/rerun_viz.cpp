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
#include <rerun_viz/rerun_viz.hpp>

#include <cstdio>
#include <filesystem>
#include <iostream>
#include <memory>

#include <rerun.hpp>
#include <rerun/spawn_options.hpp>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rerun_viz/node.hpp>
#include <rerun_viz/node_runner.hpp>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  std::string rerun_viewer_search_path = "";
  rerun::SpawnOptions spawnOptions;

  try {
    rerun_viewer_search_path = ament_index_cpp::get_package_prefix(kRerunViewerVendorPackageName);
    spawnOptions.executable_path = rerun_viewer_search_path.append("/bin/rerun-cli");

    if (!std::filesystem::exists(spawnOptions.executable_path)) {
      throw std::runtime_error(
        std::string("rerun-cli executable not found at: ").append(spawnOptions.executable_path));
    }
  } catch (const std::runtime_error & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    std::cerr << "ReRun will try to find the viewer in your PATH." << std::endl;
  } catch (const ament_index_cpp::PackageNotFoundError & e) {
    std::cerr << "Could not find rerun_viewer_vendor package: " << e.what() << std::endl;
    std::cerr << "ReRun will try to find the viewer in your PATH." << std::endl;
  }

  // Create a new `RecordingStream` which sends data over gRPC to the viewer process.
  std::shared_ptr<rerun::RecordingStream> rec =
    std::make_shared<rerun::RecordingStream>("rerun_example_cpp");

  // Try to spawn a new viewer instance.
  rec->spawn(spawnOptions).exit_on_failure();

  rclcpp::init(argc, argv);

  // Must be constructed only after rclcpp::init()!
  std::shared_ptr<rclcpp::Node> ros_node = std::make_shared<rclcpp::Node>("rerun_viz_node");
  std::shared_ptr<rerun_viz::Node> rerun_viz_node =
    std::make_shared<rerun_viz::Node>(rec, ros_node);

  rerun_viz::NodeRunner runner = rerun_viz::NodeRunner(rerun_viz_node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ros_node);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}

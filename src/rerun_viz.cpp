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

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>
#include <rerun/spawn_options.hpp>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

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
  const auto rec = rerun::RecordingStream("rerun_example_cpp");

  // Try to spawn a new viewer instance.
  rec.spawn(spawnOptions).exit_on_failure();

  // Create some data using the `grid` utility function.
  std::vector<rerun::Position3D> points =
    rerun::demo::grid3d<rerun::Position3D, float>(-10.f, 10.f, 10);
  std::vector<rerun::Color> colors = rerun::demo::grid3d<rerun::Color, uint8_t>(0, 255, 10);

  // Log the "my_points" entity with our data, using the `Points3D` archetype.
  rec.log("my_points", rerun::Points3D(points).with_colors(colors).with_radii({0.5f}));

  return 0;
}

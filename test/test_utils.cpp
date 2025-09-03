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

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

#include <rerun_viz/utils.hpp>

#include <memory>

TEST(UtilsTest, TestIsCameraTopic)
{
  EXPECT_TRUE(rerun_viz::isCameraTopic("/camera", {"sensor_msgs/msg/Image"}));
  EXPECT_TRUE(rerun_viz::isCameraTopic("/camera", {"sensor_msgs/msg/CameraInfo"}));

  EXPECT_FALSE(rerun_viz::isCameraTopic("/camera", {"sensor_msgs/msg/PointCloud2"}));
  EXPECT_FALSE(rerun_viz::isCameraTopic("/camera", {"sensor_msgs/msg/Imu"}));

  EXPECT_TRUE(
    rerun_viz::isCameraTopic(
      "/camera", {"sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/CameraInfo"}));
}

TEST(UtilsTest, TestGetCameraNamespaceFromTopic)
{
  EXPECT_EQ(rerun_viz::getCameraNamespaceFromTopic("/camera/image_raw"), "/camera");
  EXPECT_EQ(rerun_viz::getCameraNamespaceFromTopic("camera/camera_info"), "camera");

  EXPECT_EQ(rerun_viz::getCameraNamespaceFromTopic("/camera/camera_info/"), "/camera/camera_info");
  EXPECT_EQ(rerun_viz::getCameraNamespaceFromTopic("camera/camera_info/"), "camera/camera_info");

  EXPECT_EQ(
    rerun_viz::getCameraNamespaceFromTopic("/my/very/long/camera/camera_info"),
    "/my/very/long/camera");
  EXPECT_EQ(
    rerun_viz::getCameraNamespaceFromTopic("my/very/long/camera/camera_info"),
    "my/very/long/camera");

  EXPECT_THROW(rerun_viz::getCameraNamespaceFromTopic("/nope"), std::runtime_error);
  EXPECT_THROW(rerun_viz::getCameraNamespaceFromTopic("nope"), std::runtime_error);
}

TEST(UtilsTest, TestIsInNamespace)
{
  EXPECT_TRUE(rerun_viz::isInNamespace("/camera/image_raw", "/camera"));
  EXPECT_TRUE(rerun_viz::isInNamespace("/camera/image_raw", "/camera/"));

  // TODO: would be nice to support this case too but for now lets just get something working
  // EXPECT_TRUE(rerun_viz::isInNamespace("camera/image_raw", "/camera"));
  // EXPECT_TRUE(rerun_viz::isInNamespace("camera/image_raw", "/camera/"));

  EXPECT_TRUE(rerun_viz::isInNamespace("/camera/image_raw/compressed", "/camera"));
  EXPECT_TRUE(rerun_viz::isInNamespace("/camera/camera_info", "/camera"));

  EXPECT_FALSE(rerun_viz::isInNamespace("/points", "/camera"));
}

// Test for topics that have multiple types, adding and removing

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
#include <gmock/gmock.h>

#include <rerun_viz/msg_conversion/sensor_msgs/camera_info.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rerun_viz/node.hpp>

class CameraInfoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    auto ros_node = std::make_shared<rclcpp::Node>("test_camera_info_node");
    node_ = std::make_shared<rerun_viz::Node>(nullptr, ros_node);
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  sensor_msgs::msg::CameraInfo createCameraInfoMsg(
    uint32_t width = 640, uint32_t height = 480, double fx = 525.0, double fy = 525.0,
    double cx = 320.0, double cy = 240.0)
  {
    sensor_msgs::msg::CameraInfo msg;

    msg.header.frame_id = "camera_optical_frame";
    msg.header.stamp = rclcpp::Time(123, 456789);

    msg.width = width;
    msg.height = height;
    msg.distortion_model = "plumb_bob";

    // Set K matrix (camera intrinsics)
    // K = [fx  0 cx]
    //     [ 0 fy cy]
    //     [ 0  0  1]
    msg.k[0] = fx;
    msg.k[1] = 0.0;
    msg.k[2] = cx;
    msg.k[3] = 0.0;
    msg.k[4] = fy;
    msg.k[5] = cy;
    msg.k[6] = 0.0;
    msg.k[7] = 0.0;
    msg.k[8] = 1.0;

    // Set P matrix (projection matrix)
    // P = [fx'  0  cx' Tx]
    //     [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    msg.p[0] = fx;
    msg.p[1] = 0.0;
    msg.p[2] = cx;
    msg.p[3] = 0.0;
    msg.p[4] = 0.0;
    msg.p[5] = fy;
    msg.p[6] = cy;
    msg.p[7] = 0.0;
    msg.p[8] = 0.0;
    msg.p[9] = 0.0;
    msg.p[10] = 1.0;
    msg.p[11] = 0.0;

    // Set R matrix (rectification matrix - identity for monocular)
    msg.r[0] = 1.0;
    msg.r[1] = 0.0;
    msg.r[2] = 0.0;
    msg.r[3] = 0.0;
    msg.r[4] = 1.0;
    msg.r[5] = 0.0;
    msg.r[6] = 0.0;
    msg.r[7] = 0.0;
    msg.r[8] = 1.0;

    // Set distortion coefficients (k1, k2, t1, t2, k3)
    msg.d = {0.1, -0.2, 0.001, 0.002, 0.05};

    return msg;
  }

  std::shared_ptr<rerun_viz::Node> node_;
};

// Test basic construction
TEST_F(CameraInfoTest, TestConstruct)
{
  EXPECT_NO_THROW({
    auto converter = rerun_viz::CameraInfo(node_, "/camera/camera_info", nullptr);
    EXPECT_EQ(converter.getRosTypeName(), "sensor_msgs/msg/CameraInfo");
  });
}

// Test callback with null recording stream
TEST_F(CameraInfoTest, TestCallbackNullRecordingStream)
{
  auto converter = rerun_viz::CameraInfo(node_, "/camera/camera_info", nullptr);
  auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>(createCameraInfoMsg());

  // Should not crash with null recording stream
  EXPECT_NO_THROW(converter.topic_callback(msg));
}

// Test callback with uncalibrated camera
TEST_F(CameraInfoTest, TestCallbackUncalibratedCamera)
{
  auto rec = std::make_shared<rerun::RecordingStream>("test", "test");
  auto converter = rerun_viz::CameraInfo(node_, "/camera/camera_info", rec);

  auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>(createCameraInfoMsg());
  msg->k[0] = 0.0;  // Set fx to 0 to indicate uncalibrated camera

  // Should handle uncalibrated camera gracefully
  EXPECT_NO_THROW(converter.topic_callback(msg));
}

// Test callback with calibrated camera
TEST_F(CameraInfoTest, TestCallbackCalibratedCamera)
{
  auto rec = std::make_shared<rerun::RecordingStream>("test", "test");
  auto converter = rerun_viz::CameraInfo(node_, "/camera/camera_info", rec);

  auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>(
    createCameraInfoMsg(1920, 1080, 800.0, 800.0, 960.0, 540.0));

  // Should handle calibrated camera without throwing
  EXPECT_NO_THROW(converter.topic_callback(msg));
}

// Test callback with different image resolutions
TEST_F(CameraInfoTest, TestCallbackDifferentResolutions)
{
  auto rec = std::make_shared<rerun::RecordingStream>("test", "test");
  auto converter = rerun_viz::CameraInfo(node_, "/camera/camera_info", rec);

  // Test various common camera resolutions
  std::vector<std::pair<uint32_t, uint32_t>> resolutions = {
    {320, 240},    // QVGA
    {640, 480},    // VGA
    {1280, 720},   // HD
    {1920, 1080},  // Full HD
    {3840, 2160}   // 4K
  };

  for (const auto & [width, height] : resolutions) {
    auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>(createCameraInfoMsg(width, height));

    EXPECT_NO_THROW(converter.topic_callback(msg));
  }
}

// Test callback with invalid topic name
TEST_F(CameraInfoTest, TestCallbackInvalidTopicName)
{
  auto rec = std::make_shared<rerun::RecordingStream>("test", "test");
  auto converter = rerun_viz::CameraInfo(node_, "/invalid_topic", rec);

  auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>(createCameraInfoMsg());

  // Should handle invalid topic name gracefully
  EXPECT_NO_THROW(converter.topic_callback(msg));
}

// Test with extreme focal lengths
TEST_F(CameraInfoTest, TestCallbackExtremeFocalLengths)
{
  auto rec = std::make_shared<rerun::RecordingStream>("test", "test");
  auto converter = rerun_viz::CameraInfo(node_, "/camera/camera_info", rec);

  // Test very small focal lengths
  auto msg_small = std::make_shared<sensor_msgs::msg::CameraInfo>(
    createCameraInfoMsg(640, 480, 1.0, 1.0, 320.0, 240.0));
  EXPECT_NO_THROW(converter.topic_callback(msg_small));

  // Test very large focal lengths
  auto msg_large = std::make_shared<sensor_msgs::msg::CameraInfo>(
    createCameraInfoMsg(640, 480, 10000.0, 10000.0, 320.0, 240.0));
  EXPECT_NO_THROW(converter.topic_callback(msg_large));
}

// Test with off-center principal points
TEST_F(CameraInfoTest, TestCallbackOffCenterPrincipalPoint)
{
  auto rec = std::make_shared<rerun::RecordingStream>("test", "test");
  auto converter = rerun_viz::CameraInfo(node_, "/camera/camera_info", rec);

  // Test principal point at corner
  auto msg_corner = std::make_shared<sensor_msgs::msg::CameraInfo>(
    createCameraInfoMsg(640, 480, 525.0, 525.0, 0.0, 0.0));
  EXPECT_NO_THROW(converter.topic_callback(msg_corner));

  // Test principal point outside image
  auto msg_outside = std::make_shared<sensor_msgs::msg::CameraInfo>(
    createCameraInfoMsg(640, 480, 525.0, 525.0, 1000.0, 1000.0));
  EXPECT_NO_THROW(converter.topic_callback(msg_outside));
}

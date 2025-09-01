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
#include <rerun_viz/msg_conversion/sensor_msgs/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

sensor_msgs::msg::PointCloud2 getTestPointCloud2()
{
  sensor_msgs::msg::PointCloud2 pc2_msg;

  pc2_msg.height = 1;
  pc2_msg.width = 4;
  sensor_msgs::PointCloud2Modifier modifier(pc2_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(pc2_msg.height * pc2_msg.width);

  sensor_msgs::PointCloud2Iterator<float> iter_x(pc2_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pc2_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pc2_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pc2_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pc2_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pc2_msg, "b");

  std::vector<float> point_data = {
    1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 12.0f,
  };

  std::vector<uint8_t> color_data = {
    255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 0,
  };

  for (size_t i = 0; i < (pc2_msg.height * pc2_msg.width);
       ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
    *iter_x = point_data[3 * i + 0];
    *iter_y = point_data[3 * i + 1];
    *iter_z = point_data[3 * i + 2];
    *iter_r = color_data[3 * i + 0];
    *iter_g = color_data[3 * i + 1];
    *iter_b = color_data[3 * i + 2];
  }

  return pc2_msg;
}

void checkPoints3DAgainstPointCloud2(
  const rerun::Points3D & points3d, const sensor_msgs::msg::PointCloud2 & pc2_msg)
{
  ASSERT_EQ(points3d.positions.value().length(), pc2_msg.height * pc2_msg.width);

  /*sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc2_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc2_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc2_msg, "z");

  auto * list_array =
    static_cast<const arrow::FixedSizeListArray *>(points3d.positions.value().array);
  auto * values = static_cast<const arrow::FloatArray *>(&list_array->values());

  std::vector<rerun::datatypes::Vec3D> vecs;
  for (int64_t i = 0; i < list_array->length(); ++i) {
    float x = values->Value(i * 3 + 0);
    float y = values->Value(i * 3 + 1);
    float z = values->Value(i * 3 + 2);
    vecs.emplace_back(x, y, z);
    // or: vecs.push_back(rerun::datatypes::Vec3D({x, y, z}));
  }*/
}

TEST(PointCloud2, TestConversion)
{
  sensor_msgs::msg::PointCloud2 in = getTestPointCloud2();
  rerun::Points3D out = rerun_viz::PointCloud2::toRerunType(in);
  checkPoints3DAgainstPointCloud2(out, in);
}

TEST(PointCloud2, TestConstruct)
{
  rclcpp::init(0, nullptr);

  auto publisher_node = std::make_shared<rclcpp::Node>("publisher_node");
  auto pc2_publisher = publisher_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/points", rclcpp::SensorDataQoS());

  auto listener_node = std::make_shared<rclcpp::Node>("listener_node");
  rerun_viz::PointCloud2 node(listener_node, "/points");

  // Verify that `/points` has one subscriber
  EXPECT_EQ(publisher_node->count_subscribers("/points"), 1);

  rclcpp::shutdown();
}

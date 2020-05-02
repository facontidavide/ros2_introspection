#include <cstdio>
#include <iostream>
#include <ros2_introspection/ros2_introspection.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <gtest/gtest.h>
#include "test_utils.h"

using namespace Ros2Introspection;

// IMU is a type with:
//  - type hierarchy,
//  - a string
//  - fixed size arrays

TEST(Ros2Introspection, Imu) {

  std::string topic_type = "sensor_msgs/Imu";
  sensor_msgs::msg::Imu imu_msg;

  imu_msg.header.stamp.sec = 3;
  imu_msg.header.stamp.nanosec = 4;
  imu_msg.header.frame_id = "hell";
  imu_msg.orientation.x = 111;
  imu_msg.orientation.y = 112;
  imu_msg.orientation.z = 113;
  imu_msg.orientation.w = 114;

  imu_msg.angular_velocity.x = 131;
  imu_msg.angular_velocity.y = 132;
  imu_msg.angular_velocity.z = 133;

  imu_msg.linear_acceleration.x = 151;
  imu_msg.linear_acceleration.y = 152;
  imu_msg.linear_acceleration.z = 153;

  for(size_t i=0; i<9; i++)
  {
    imu_msg.orientation_covariance[i] = i;
    imu_msg.angular_velocity_covariance[i] = 10+i;
    imu_msg.linear_acceleration_covariance[i] = 20+i;
  }

  const auto* imu_typesupport =
    rosbag2::get_typesupport(topic_type, rosidl_typesupport_cpp::typesupport_identifier);
  auto serialized_msg =  RmwInterface().serialize_message(imu_msg, imu_typesupport);


  FlatMessage flat_message;
  Parser parser;
  parser.registerMessageType("imu", topic_type);

  BufferView buffer(serialized_msg->buffer, serialized_msg->buffer_length);
  parser.deserializeIntoFlatMessage("imu", buffer, &flat_message, 100);

  RenamedValues renamed;

  ConvertFlatMessageToRenamedValues(flat_message, renamed);

  for(const auto& pair: renamed)
  {
    std::cout << pair.first << " = " << pair.second << std::endl;
  }

  size_t index = 0;
  ASSERT_EQ( renamed[index].first, "/imu/header/stamp/sec");
  ASSERT_EQ( renamed[index].second, imu_msg.header.stamp.sec);
  index++;
  ASSERT_EQ( renamed[index].first, "/imu/header/stamp/nanosec");
  ASSERT_EQ( renamed[index].second, imu_msg.header.stamp.nanosec);
  index++;
  ASSERT_EQ( renamed[index].first, "/imu/orientation/x");
  ASSERT_EQ( renamed[index].second, imu_msg.orientation.x);
  index++;
  ASSERT_EQ( renamed[index].first, "/imu/orientation/y");
  ASSERT_EQ( renamed[index].second, imu_msg.orientation.y);
  index++;
  ASSERT_EQ( renamed[index].first, "/imu/orientation/z");
  ASSERT_EQ( renamed[index].second, imu_msg.orientation.z);
  index++;
  ASSERT_EQ( renamed[index].first, "/imu/orientation/w");
  ASSERT_EQ( renamed[index].second, imu_msg.orientation.w);
  index++;

  for(size_t i=0; i<9; i++)
  {
    std::string expected_name = "/imu/orientation_covariance.";
    expected_name += std::to_string(i);
    ASSERT_EQ( renamed[index].first, expected_name);
    ASSERT_EQ( renamed[index].second, imu_msg.orientation_covariance[i]);
    index++;
  }

  ASSERT_EQ( renamed[index].first, "/imu/angular_velocity/x");
  ASSERT_EQ( renamed[index].second, imu_msg.angular_velocity.x);
  index++;
  ASSERT_EQ( renamed[index].first, "/imu/angular_velocity/y");
  ASSERT_EQ( renamed[index].second, imu_msg.angular_velocity.y);
  index++;
  ASSERT_EQ( renamed[index].first, "/imu/angular_velocity/z");
  ASSERT_EQ( renamed[index].second, imu_msg.angular_velocity.z);
  index++;

  for(size_t i=0; i<9; i++)
  {
    std::string expected_name = "/imu/angular_velocity_covariance.";
    expected_name += std::to_string(i);
    ASSERT_EQ( renamed[index].first, expected_name);
    ASSERT_EQ( renamed[index].second, imu_msg.angular_velocity_covariance[i]);
    index++;
  }

  ASSERT_EQ( renamed[index].first, "/imu/linear_acceleration/x");
  ASSERT_EQ( renamed[index].second, imu_msg.linear_acceleration.x);
  index++;
  ASSERT_EQ( renamed[index].first, "/imu/linear_acceleration/y");
  ASSERT_EQ( renamed[index].second, imu_msg.linear_acceleration.y);
  index++;
  ASSERT_EQ( renamed[index].first, "/imu/linear_acceleration/z");
  ASSERT_EQ( renamed[index].second, imu_msg.linear_acceleration.z);
  index++;

  for(size_t i=0; i<9; i++)
  {
    std::string expected_name = "/imu/linear_acceleration_covariance.";
    expected_name += std::to_string(i);
    ASSERT_EQ( renamed[index].first, expected_name);
    ASSERT_EQ( renamed[index].second, imu_msg.linear_acceleration_covariance[i]);
    index++;
  }
}

// Good to test variable arrays. Test also the maximum array size
TEST(Ros2Introspection, Polygon) {

  geometry_msgs::msg::Polygon polygon;
  std::string topic_type = "geometry_msgs/Polygon";

  for(int i=0; i<10; i++)
  {
    geometry_msgs::msg::Point32 point;
    point.x = 10*i;
    point.y = 10*i + 1;
    point.z = 10*i + 2;
    polygon.points.push_back(point);
  }

  const auto* typesupport =
    rosbag2::get_typesupport(topic_type, rosidl_typesupport_cpp::typesupport_identifier);
  auto serialized_msg =  RmwInterface().serialize_message(polygon, typesupport);

  //-------------------------
  Parser parser;
  FlatMessage flat_message;
  RenamedValues renamed;

  parser.registerMessageType("poly", topic_type);

  BufferView buffer(serialized_msg->buffer, serialized_msg->buffer_length);
  const int max_array_size = 3;
  parser.deserializeIntoFlatMessage("poly", buffer, &flat_message, max_array_size);

  ConvertFlatMessageToRenamedValues(flat_message, renamed);

  for(const auto& pair: renamed)
  {
    std::cout << pair.first << " = " << pair.second << std::endl;
  }

  ASSERT_EQ( renamed.size(), 3*max_array_size );

  size_t index = 0;
  ASSERT_EQ( renamed[index].first, "/poly/points.0/x");
  ASSERT_EQ( renamed[index].second, polygon.points[0].x);
  index++;
  ASSERT_EQ( renamed[index].first, "/poly/points.0/y");
  ASSERT_EQ( renamed[index].second, polygon.points[0].y);
  index++;
  ASSERT_EQ( renamed[index].first, "/poly/points.0/z");
  ASSERT_EQ( renamed[index].second, polygon.points[0].z);
  index++;

  ASSERT_EQ( renamed[index].first, "/poly/points.1/x");
  ASSERT_EQ( renamed[index].second, polygon.points[1].x);
  index++;
  ASSERT_EQ( renamed[index].first, "/poly/points.1/y");
  ASSERT_EQ( renamed[index].second, polygon.points[1].y);
  index++;
  ASSERT_EQ( renamed[index].first, "/poly/points.1/z");
  ASSERT_EQ( renamed[index].second, polygon.points[1].z);
  index++;

  ASSERT_EQ( renamed[index].first, "/poly/points.2/x");
  ASSERT_EQ( renamed[index].second, polygon.points[2].x);
  index++;
  ASSERT_EQ( renamed[index].first, "/poly/points.2/y");
  ASSERT_EQ( renamed[index].second, polygon.points[2].y);
  index++;
  ASSERT_EQ( renamed[index].first, "/poly/points.2/z");
  ASSERT_EQ( renamed[index].second, polygon.points[2].z);
}

// Good to test constants
TEST(Ros2Introspection, Battery) {

  std::string topic_type = "sensor_msgs/BatteryState";
  sensor_msgs::msg::BatteryState battery;

  // for this test, let's just check first and last field
  battery.header.stamp.sec = 123;

  battery.voltage = 1;
  battery.temperature = 2;
  battery.current = 3;
  battery.charge = 4;
  battery.charge = 4;
  battery.capacity = 6;
  battery.design_capacity = 7;
  battery.percentage = 8;
  battery.power_supply_status = 9;
  battery.power_supply_technology = 10;
  battery.present = true;
  battery.cell_voltage.resize(3);
  battery.cell_voltage[0] = 5;
  battery.cell_voltage[1] = 6;
  battery.cell_voltage[2] = 7;

  battery.serial_number = 666;

  const auto* typesupport =
    rosbag2::get_typesupport(topic_type, rosidl_typesupport_cpp::typesupport_identifier);
  auto serialized_msg =  RmwInterface().serialize_message(battery, typesupport);

  //-------------------------
  Parser parser;
  FlatMessage flat_message;
  RenamedValues renamed;

  parser.registerMessageType("battery", topic_type);

  BufferView buffer(serialized_msg->buffer, serialized_msg->buffer_length);
  const int max_array_size = 3;
  parser.deserializeIntoFlatMessage("battery", buffer, &flat_message, max_array_size);

  ConvertFlatMessageToRenamedValues(flat_message, renamed);

  for(const auto& pair: renamed)
  {
    std::cout << pair.first << " = " << pair.second << std::endl;
  }
}


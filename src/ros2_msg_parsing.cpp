#include <cstdio>
#include <iostream>
#include <ros2_introspection/ros2_introspection.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "utils.h"


int main(int argc, char ** argv)
{
  std::string topic_type = "sensor_msgs/Imu";
  sensor_msgs::msg::Imu imu_msg;

  imu_msg.header.stamp.sec = 3;
  imu_msg.header.stamp.nanosec = 4;
  imu_msg.header.frame_id = "hello_world";
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

  for(int i=0; i<9; i++)
  {
    imu_msg.orientation_covariance[i] = i;
    imu_msg.angular_velocity_covariance[i] = 10+i;
    imu_msg.linear_acceleration_covariance[i] = 20+i;
  }

  const auto* imu_typesupport =
    rosbag2::get_typesupport(topic_type, rosidl_typesupport_cpp::typesupport_identifier);

  RmwInterface mm;
  auto serialized_msg =  mm.serialize_message(imu_msg, imu_typesupport);
  auto imu_out = mm.deserialize_message<sensor_msgs::msg::Imu>(serialized_msg, imu_typesupport);

  using namespace Ros2Introspection;

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

  return 0;
}

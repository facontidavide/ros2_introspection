#include <cstdio>
#include <iostream>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosbag2/typesupport_helpers.hpp>
#include <rosbag2/types/introspection_message.hpp>
#include <rcutils/time.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <rmw/rmw.h>
#include "utils.h"

struct TopicField
{
  std::string key;
  double value;
};

template <typename T> inline double CastFromBuffer(uint8_t* buffer, size_t& offset)
{
  double value = static_cast<double>(*reinterpret_cast<T*>(buffer+offset));
  offset += sizeof(T);
  return value;
}

void generateMessageTypesVec(std::vector<TopicField> &membersVec,
                             const std::string &path,
                             const rosidl_message_type_support_t *typeData,
                             uint8_t* buffer_ptr,
                             size_t& offset)
{
  using namespace rosidl_typesupport_introspection_cpp;

  const auto* members = static_cast<const MessageMembers*>(typeData->data);

  for(size_t i = 0; i < members->member_count_; i++)
  {
    int array_size = 1;
    const MessageMember& member = members->members_[i];
    if(member.is_array_ && member.array_size_ != 0)
    {
      array_size = member.array_size_;
    }

    for (int a=0; a < array_size; a++)
    {
      if(member.type_id_ != ROS_TYPE_MESSAGE && member.type_id_ != ROS_TYPE_STRING)
      {
        double value = 0;
        switch( member.type_id_)
        {
          case ROS_TYPE_FLOAT:   value = CastFromBuffer<float>(buffer_ptr, offset); break;
          case ROS_TYPE_DOUBLE:  value = CastFromBuffer<double>(buffer_ptr, offset); break;

          case ROS_TYPE_INT64:   value = CastFromBuffer<int64_t>(buffer_ptr, offset); break;
          case ROS_TYPE_INT32:   value = CastFromBuffer<int32_t>(buffer_ptr, offset); break;
          case ROS_TYPE_INT16:   value = CastFromBuffer<int16_t>(buffer_ptr, offset); break;
          case ROS_TYPE_INT8:    value = CastFromBuffer<int8_t>(buffer_ptr, offset); break;

          case ROS_TYPE_UINT64:  value = CastFromBuffer<uint64_t>(buffer_ptr, offset); break;
          case ROS_TYPE_UINT32:  value = CastFromBuffer<uint32_t>(buffer_ptr, offset); break;
          case ROS_TYPE_UINT16:  value = CastFromBuffer<uint16_t>(buffer_ptr, offset); break;
          case ROS_TYPE_UINT8:   value = CastFromBuffer<uint8_t>(buffer_ptr, offset); break;

          case ROS_TYPE_BOOLEAN: value = CastFromBuffer<bool>(buffer_ptr, offset); break;
        }
        TopicField pair = {path + "/" + member.name_, value};
        if( array_size != 1)
        {
          pair.key.append(".");
          pair.key.append( std::to_string(a));
        }
        std::cout << pair.key << " = " << pair.value << std::endl;
        membersVec.push_back( std::move(pair) );
      }
      else if(member.type_id_ == ROS_TYPE_STRING)
      {
        int32_t str_length = *reinterpret_cast<int32_t*>(buffer_ptr + offset);
        offset += sizeof(int32_t);
        std::string str(reinterpret_cast<char*>(buffer_ptr + offset), str_length);
        std::cout << str << std::endl;
        offset += str_length;
      }
      else if(member.type_id_ == ROS_TYPE_MESSAGE){
        generateMessageTypesVec(membersVec,
                                path + "/" + member.name_,
                                member.members_,
                                buffer_ptr,
                                offset);
      }
    } // end for array
  } // end for memebers
}

int main(int argc, char ** argv)
{
  std::string topic_type = "sensor_msgs/Imu";
  sensor_msgs::msg::Imu imu_in;

  imu_in.header.stamp.sec = 3;
  imu_in.header.stamp.nanosec = 4;
  imu_in.header.frame_id = "hello_world";
  imu_in.orientation.x = 111;
  imu_in.orientation.y = 112;
  imu_in.orientation.z = 113;
  imu_in.orientation.w = 114;

  imu_in.angular_velocity.x = 131;
  imu_in.angular_velocity.y = 132;
  imu_in.angular_velocity.z = 133;

  imu_in.linear_acceleration.x = 151;
  imu_in.linear_acceleration.y = 152;
  imu_in.linear_acceleration.z = 153;

  const auto* imu_typesupport =
    rosbag2::get_typesupport(topic_type, rosidl_typesupport_cpp::typesupport_identifier);

  RmwInterface mm;

  auto serialized_msg =  mm.serialize_message(imu_in, imu_typesupport);
  auto imu_out = mm.deserialize_message<sensor_msgs::msg::Imu>(serialized_msg, imu_typesupport);

  const auto* introspection_support =
    rosbag2::get_typesupport(topic_type, rosidl_typesupport_introspection_cpp::typesupport_identifier);


  auto allocator = rcutils_get_default_allocator();
  auto msg_buffer = rosbag2::allocate_introspection_message(introspection_support, &allocator);

  std::vector<TopicField> members;

  size_t offset = 0;
  double header_val = CastFromBuffer<int32_t>(serialized_msg->buffer, offset);

  generateMessageTypesVec(members, "imu", introspection_support, serialized_msg->buffer, offset);

  //  for(const auto& pair: members)
  //  {
  //    std::cout << pair.key << " = " << pair.value << std::endl;
  //  }

  return 0;
}

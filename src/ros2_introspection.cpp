#include <ros2_introspection/ros2_introspection.hpp>

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosbag2/typesupport_helpers.hpp>
#include <rosbag2/types/introspection_message.hpp>
#include <rcutils/time.h>
#include <functional>
#include <cmath>
#include  <sensor_msgs/msg/imu__rosidl_typesupport_fastrtps_c.h>
#include  <fastcdr/Cdr.h>

namespace Ros2Introspection
{

void Parser::registerMessageType(
  const std::string &message_identifier,
  const std::string &type_name)
{
  if( _registered_messages.count(message_identifier) != 0)
  {
    return;
  }

  Ros2MessageInfo info;

  using TypeSupport = rosidl_message_type_support_t;

  info.type_support =
    rosbag2::get_typesupport(type_name, rosidl_typesupport_introspection_cpp::typesupport_identifier);

  //------- create StringTree --------
  using rosidl_typesupport_introspection_cpp::MessageMember;
  using rosidl_typesupport_introspection_cpp::MessageMembers;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;

  std::function<void(StringTreeNode* node, const TypeSupport*)> recursivelyCreateTree;
  recursivelyCreateTree = [&](StringTreeNode* node, const TypeSupport* type_data)
  {
    const auto* members = static_cast<const MessageMembers*>(type_data->data);

    node->children().reserve(members->member_count_);

    for(size_t i = 0; i < members->member_count_; i++)
    {
      const MessageMember& member = members->members_[i];
      StringTreeNode *new_node = node->addChild( member.name_ );
      if( member.is_array_ )
      {
        new_node->children().reserve(1);
        new_node = new_node->addChild("#");
      }
      if(member.type_id_ == ROS_TYPE_MESSAGE)
      {
        //recursion
        recursivelyCreateTree(new_node, member.members_);
      }
    }
  };

  info.field_tree.root()->children().reserve(1);
  auto starting_node =  info.field_tree.root()->addChild(message_identifier);

  // start building recursively
  recursivelyCreateTree( starting_node, info.type_support );

  std::pair<std::string,Ros2MessageInfo> pair;
  pair.first = message_identifier;
  pair.second = std::move(info);
  _registered_messages.insert( std::move(pair) );
}


template <typename T> inline T CastFromBuffer(eprosima::fastcdr::Cdr& cdr)
{
  T tmp;
  cdr.deserialize(tmp);
  return tmp;
}

bool Parser::deserializeIntoFlatMessage(
  const std::string &msg_identifier,
  const rcutils_uint8_array_t* msg,
  FlatMessage *flat_container,
  const uint32_t max_array_size) const
{
  const auto message_info_it = _registered_messages.find(msg_identifier);
  if(message_info_it == _registered_messages.end())
  {
    throw std::runtime_error("Message not registered");
  }

  eprosima::fastcdr::FastBuffer buffer( reinterpret_cast<char*>(msg->buffer), msg->buffer_length);
  eprosima::fastcdr::Cdr cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                             eprosima::fastcdr::Cdr::DDS_CDR);
  cdr.read_encapsulation();

  auto& message_info = message_info_it->second;

  using TypeSupport = rosidl_message_type_support_t;

  std::function<void(const TypeSupport*, StringTreeLeaf&, bool)> recursiveParser;

  //---------- recursive function -------------
  recursiveParser = [&](const TypeSupport* type_data,
                        StringTreeLeaf& tree_leaf,
                        bool skip_save)
  {
    using namespace rosidl_typesupport_introspection_cpp;
    const auto* members = static_cast<const MessageMembers*>(type_data->data);

    for(size_t index = 0; index < members->member_count_; index++)
    {
      const MessageMember& member = members->members_[index];

      auto new_tree_leaf = tree_leaf;
      new_tree_leaf.node_ptr = tree_leaf.node_ptr->child(index);

      size_t array_size = 1;

      if(member.is_array_)
      {
        if( member.array_size_ == 0)
        {
          array_size = size_t(CastFromBuffer<uint32_t>(cdr));
        }
        else{
          array_size = member.array_size_;
        }
      }

      if(array_size > MAX_ARRAY_SIZE &&
          (member.type_id_ == ROS_TYPE_INT8 || member.type_id_ == ROS_TYPE_UINT8))
      {
        uint32_t blob_size = CastFromBuffer<uint32_t>(cdr);

        if( !skip_save ){
          BufferView blob(cdr.getCurrentPosition(), blob_size);
          flat_container->blobs.push_back( {new_tree_leaf, std::move(blob)} );
        }
        cdr.jump(blob_size);

        continue;
      }

      if( array_size > 1)
      {
        new_tree_leaf.index_array.push_back(0);
        new_tree_leaf.node_ptr = new_tree_leaf.node_ptr->child(0);
      }

      for (size_t a=0; a < array_size; a++)
      {
        if( array_size > 1)
        {
          new_tree_leaf.index_array.back() = a;
        }

        if( a >= max_array_size)
        {
          skip_save = true;
        }

        if(member.type_id_ != ROS_TYPE_MESSAGE && member.type_id_ != ROS_TYPE_STRING)
        {
          double value = 0;
          switch( member.type_id_)
          {
            case ROS_TYPE_FLOAT:   value = double(CastFromBuffer<float>(cdr)); break;
            case ROS_TYPE_DOUBLE:  value = double(CastFromBuffer<double>(cdr)); break;

            case ROS_TYPE_INT64:   value = double(CastFromBuffer<int64_t>(cdr)); break;
            case ROS_TYPE_INT32:   value = double(CastFromBuffer<int32_t>(cdr)); break;
            case ROS_TYPE_INT16:   value = double(CastFromBuffer<int16_t>(cdr)); break;
            case ROS_TYPE_INT8:    value = double(CastFromBuffer<int8_t>(cdr)); break;

            case ROS_TYPE_UINT64:  value = double(CastFromBuffer<uint64_t>(cdr)); break;
            case ROS_TYPE_UINT32:  value = double(CastFromBuffer<uint32_t>(cdr)); break;
            case ROS_TYPE_UINT16:  value = double(CastFromBuffer<uint16_t>(cdr)); break;
            case ROS_TYPE_UINT8:   value = double(CastFromBuffer<uint8_t>(cdr)); break;

            case ROS_TYPE_BOOLEAN: value = double(CastFromBuffer<bool>(cdr)); break;
          }
          if( !skip_save )
          {
            flat_container->values.push_back( {new_tree_leaf, value} );
          }
        }
        else if(member.type_id_ == ROS_TYPE_STRING)
        {
          if( !skip_save ){
            std::string str;
            cdr.deserialize( str );
            flat_container->strings.push_back( {new_tree_leaf, std::move(str)} );
          }
          else{
            static std::string tmp;
            cdr.deserialize( tmp );
          }
        }
        else if(member.type_id_ == ROS_TYPE_MESSAGE)
        {
          recursiveParser( member.members_, new_tree_leaf, skip_save );
        }
      } // end for array
    } // end for members
  };
  //---------- END recursive function -------------

  flat_container->blobs.clear();
  flat_container->strings.clear();
  flat_container->values.clear();
  flat_container->tree = &message_info.field_tree;

  StringTreeLeaf rootnode;
  rootnode.node_ptr = message_info.field_tree.croot()->child(0);
  recursiveParser( message_info.type_support, rootnode, false);

  return true;
}

void ConvertFlatMessageToRenamedValues(
  const FlatMessage &flat,
  RenamedValues &renamed)
{
  const auto& values = flat.values;
  renamed.resize(values.size());

  for(size_t i=0; i<values.size(); i++)
  {
    const auto& value = values[i];
    auto& dest  = renamed[i];
    value.first.toStr( dest.first );
    dest.second = value.second;
  }
}

} // end namespace

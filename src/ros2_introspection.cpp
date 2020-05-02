#include <ros2_introspection/ros2_introspection.hpp>

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosbag2/typesupport_helpers.hpp>
#include <rosbag2/types/introspection_message.hpp>
#include <rcutils/time.h>

#include <functional>

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

  // start building recurisively
  recursivelyCreateTree( info.field_tree.root(), info.type_support );

  std::pair<std::string,Ros2MessageInfo> pair;
  pair.first = message_identifier;
  pair.second = std::move(info);
  _registered_messages.insert( std::move(pair) );
}

template <typename T> inline T CastFromBuffer(BufferView& buffer)
{
  const size_t N = sizeof(T);
  if( N > buffer.size())
  {
    throw std::runtime_error("Buffer overrun");
  };
  T value = (*reinterpret_cast<T*>(buffer.data()));
  buffer = buffer.subspan(sizeof(T));
  return value;
}


bool Parser::deserializeIntoFlatMessage(
  const std::string &msg_identifier,
  BufferView buffer,
  FlatMessage *flat_container,
  const uint32_t max_array_size) const
{
  const auto message_info_it = _registered_messages.find(msg_identifier);
  if(message_info_it == _registered_messages.end())
  {
    throw std::runtime_error("Message not registered");
  }

  auto& message_info = message_info_it->second;

  using TypeSupport = rosidl_message_type_support_t;

  std::function<void(const TypeSupport*, BufferView&, StringTreeLeaf&)> recursiveParser;

  //---------- recursive function -------------
  recursiveParser = [&](const TypeSupport* type_data,
                        BufferView& buffer,
                        StringTreeLeaf& tree_leaf)
  {
    using namespace rosidl_typesupport_introspection_cpp;
    const auto* members = static_cast<const MessageMembers*>(type_data->data);

    for(size_t index = 0; index < members->member_count_; index++)
    {
      auto new_tree_leaf = tree_leaf;
      new_tree_leaf.node_ptr = tree_leaf.node_ptr->child(index);

      size_t array_size = 1;
      const MessageMember& member = members->members_[index];
      if(member.is_array_ && member.array_size_ != 0)
      {
        array_size = member.array_size_;
        new_tree_leaf.index_array.push_back(0);
        new_tree_leaf.node_ptr = new_tree_leaf.node_ptr->child(0);
      }

      for (size_t a=0; a < array_size; a++)
      {
        if(member.type_id_ != ROS_TYPE_MESSAGE && member.type_id_ != ROS_TYPE_STRING)
        {
          double value = 0;
          switch( member.type_id_)
          {
            case ROS_TYPE_FLOAT:   value = double(CastFromBuffer<float>(buffer)); break;
            case ROS_TYPE_DOUBLE:  value = double(CastFromBuffer<double>(buffer)); break;

            case ROS_TYPE_INT64:   value = double(CastFromBuffer<int64_t>(buffer)); break;
            case ROS_TYPE_INT32:   value = double(CastFromBuffer<int32_t>(buffer)); break;
            case ROS_TYPE_INT16:   value = double(CastFromBuffer<int16_t>(buffer)); break;
            case ROS_TYPE_INT8:    value = double(CastFromBuffer<int8_t>(buffer)); break;

            case ROS_TYPE_UINT64:  value = double(CastFromBuffer<uint64_t>(buffer)); break;
            case ROS_TYPE_UINT32:  value = double(CastFromBuffer<uint32_t>(buffer)); break;
            case ROS_TYPE_UINT16:  value = double(CastFromBuffer<uint16_t>(buffer)); break;
            case ROS_TYPE_UINT8:   value = double(CastFromBuffer<uint8_t>(buffer)); break;

            case ROS_TYPE_BOOLEAN: value = double(CastFromBuffer<bool>(buffer)); break;
          }
          if( array_size > 1)
          {
            new_tree_leaf.index_array.back() = a;
          }
          flat_container->value.push_back( {new_tree_leaf, value} );
        }
        else if(member.type_id_ == ROS_TYPE_STRING)
        {
          int32_t str_length = CastFromBuffer<int32_t>(buffer);
          if( str_length > buffer.size())
          {
            throw std::runtime_error("Buffer overrun");
          };
          StringView str(reinterpret_cast<char*>(buffer.data()), size_t(str_length));
          buffer = buffer.subspan(str_length);
          flat_container->string.push_back( {new_tree_leaf, str} );
        }
        else if(member.type_id_ == ROS_TYPE_MESSAGE){
          recursiveParser( member.members_, buffer, new_tree_leaf );
        }
      } // end for array
    } // end for members
  };
  //---------- END recursive function -------------

  flat_container->blob.clear();
  flat_container->string.clear();
  flat_container->value.clear();

  flat_container->tree = &message_info.field_tree;

  StringTreeLeaf rootnode;
  rootnode.node_ptr = message_info.field_tree.croot();

  buffer = buffer.subspan(4);

  recursiveParser( message_info.type_support, buffer, rootnode);

  if( buffer.size() > 1 )
  {
    throw std::runtime_error("Buffer left after parsing");
  }

  return true;
}

void ConvertFlatMessageToRenamedValues(
  const FlatMessage &flat,
  RenamedValues &renamed)
{
  const auto& values = flat.value;
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

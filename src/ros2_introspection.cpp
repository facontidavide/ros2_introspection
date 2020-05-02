#include <ros2_introspection/ros2_introspection.hpp>

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosbag2/typesupport_helpers.hpp>
#include <rosbag2/types/introspection_message.hpp>
#include <rcutils/time.h>
#include <fastcdr/FastBuffer.h>
#include <fastcdr/FastCdr.h>
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

  info.field_tree.root()->children().reserve(1);
  auto starting_node =  info.field_tree.root()->addChild(message_identifier);

  // start building recurisively
  recursivelyCreateTree( starting_node, info.type_support );

  std::pair<std::string,Ros2MessageInfo> pair;
  pair.first = message_identifier;
  pair.second = std::move(info);
  _registered_messages.insert( std::move(pair) );
}

template <typename T> inline T CastFromBuffer(BufferView buffer, size_t offset)
{
  const size_t N = sizeof(T);
  if( offset+N > buffer.size())
  {
    throw std::runtime_error("Buffer overrun");
  };
  T value = (*reinterpret_cast<T*>(buffer.data() + offset));
 // offset += N;
  return value;
}


bool Parser::deserializeIntoFlatMessage(
  const std::string &msg_identifier,
  BufferView buffer,
  FlatMessage *flat_container,
  const uint32_t max_array_size) const
{
  eprosima::fastcdr::FastBuffer fast_buffer( reinterpret_cast<char*>(buffer.data()), buffer.size());
  eprosima::fastcdr::FastCdr fast_cdr(fast_buffer);

  const auto message_info_it = _registered_messages.find(msg_identifier);
  if(message_info_it == _registered_messages.end())
  {
    throw std::runtime_error("Message not registered");
  }

  auto& message_info = message_info_it->second;

  using TypeSupport = rosidl_message_type_support_t;

  std::function<void(const TypeSupport*, BufferView&, StringTreeLeaf&, size_t, bool)> recursiveParser;

  //---------- recursive function -------------
  recursiveParser = [&](const TypeSupport* type_data,
                        BufferView& buffer,
                        StringTreeLeaf& tree_leaf,
                        size_t parent_offset,
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

      size_t offset = parent_offset + member.offset_;

      if(member.is_array_)
      {
        if( member.array_size_ == 0)
        {
          array_size = size_t(CastFromBuffer<uint32_t>(buffer, offset));
        }
        else{
          array_size = member.array_size_;
        }
      }

      if(array_size > MAX_ARRAY_SIZE &&
         (member.type_id_ == ROS_TYPE_INT8 || member.type_id_ == ROS_TYPE_UINT8))
      {
        int32_t blob_size = CastFromBuffer<int32_t>(buffer, offset);
        if( blob_size > buffer.size())
        {
          throw std::runtime_error("Buffer overrun");
        };
        if( !skip_save ){
          BufferView blob(reinterpret_cast<uint8_t*>(buffer.data() + offset + sizeof(int32_t)), blob_size);
          flat_container->blob.push_back( {new_tree_leaf, blob} );
        }
        buffer = buffer.subspan(blob_size);
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
            case ROS_TYPE_FLOAT:   value = double(CastFromBuffer<float>(buffer, member.offset_)); break;
            case ROS_TYPE_DOUBLE:  value = double(CastFromBuffer<double>(buffer, member.offset_)); break;

            case ROS_TYPE_INT64:   value = double(CastFromBuffer<int64_t>(buffer, member.offset_)); break;
            case ROS_TYPE_INT32:   value = double(CastFromBuffer<int32_t>(buffer, member.offset_)); break;
            case ROS_TYPE_INT16:   value = double(CastFromBuffer<int16_t>(buffer, member.offset_)); break;
            case ROS_TYPE_INT8:    value = double(CastFromBuffer<int8_t>(buffer, member.offset_)); break;

            case ROS_TYPE_UINT64:  value = double(CastFromBuffer<uint64_t>(buffer, member.offset_)); break;
            case ROS_TYPE_UINT32:  value = double(CastFromBuffer<uint32_t>(buffer, member.offset_)); break;
            case ROS_TYPE_UINT16:  value = double(CastFromBuffer<uint16_t>(buffer, member.offset_)); break;
            case ROS_TYPE_UINT8:   value = double(CastFromBuffer<uint8_t>(buffer, member.offset_)); break;

            case ROS_TYPE_BOOLEAN: value = double(CastFromBuffer<bool>(buffer, member.offset_)); break;
          }
          if( !skip_save )
          {
            flat_container->value.push_back( {new_tree_leaf, value} );
          }
        }
        else if(member.type_id_ == ROS_TYPE_STRING)
        {
          int32_t str_length = CastFromBuffer<int32_t>(buffer, member.offset_);
          if( str_length > buffer.size())
          {
            throw std::runtime_error("Buffer overrun");
          };
          if( !skip_save )
          {
            StringView str(reinterpret_cast<char*>(buffer.data()+offset+4), size_t(str_length));
            flat_container->string.push_back( {new_tree_leaf, str} );
          }
          buffer = buffer.subspan(str_length);
        }
        else if(member.type_id_ == ROS_TYPE_MESSAGE){
          recursiveParser( member.members_, buffer, new_tree_leaf, offset, skip_save );
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
  rootnode.node_ptr = message_info.field_tree.croot()->child(0);

  buffer = buffer.subspan(4);

  recursiveParser( message_info.type_support, buffer, rootnode, 0, false);

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

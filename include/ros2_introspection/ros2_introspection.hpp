/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright 2016-2017 Davide Faconti
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
* *******************************************************************/
#pragma once

#include <unordered_map>
#include <ros2_introspection/stringtree.hpp>
#include <rosbag2/typesupport_helpers.hpp>
#include <rosbag2/types/introspection_message.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>

namespace Ros2Introspection{

struct FlatMessage {

  /// Tree that the StringTreeLeaf(s) refer to.
  const StringTree* tree;

  /// List of all those parsed fields that can be represented by a builtin value different from "string".
  /// This list will be filled by the funtion buildRosFlatType.
  std::vector< std::pair<StringTreeLeaf, double> > values;

  /// List of all those parsed fields that can be represented by a builtin value equal to "string".
  /// This list will be filled by the funtion buildRosFlatType.
  std::vector< std::pair<StringTreeLeaf, std::string> > strings;

  /// Store "blobs", i.e all those fields which are vectors of BYTES (AKA uint8_t),
  /// where the vector size is greater than the argument [max_array_size]
  /// passed  to the function deserializeIntoFlatContainer
  std::vector< std::pair<StringTreeLeaf, BufferView> > blobs;
};

typedef std::vector< std::pair<std::string, double> > RenamedValues;

struct Ros2MessageInfo{
  const rosidl_message_type_support_t* type_support;
  StringTree field_tree;
};

class Parser{

public:
  Parser(): _discard_large_array(DISCARD_LARGE_ARRAYS)
 {}

  enum MaxArrayPolicy: bool {
    DISCARD_LARGE_ARRAYS = true,
    KEEP_LARGE_ARRAYS = false
  };

  enum{ MAX_ARRAY_SIZE = 999 };

  void setMaxArrayPolicy( MaxArrayPolicy discard_entire_array )
  {
      _discard_large_array = discard_entire_array;
  }

  void setMaxArrayPolicy( bool discard_entire_array )
  {
    _discard_large_array = static_cast<MaxArrayPolicy>(discard_entire_array);
  }

  MaxArrayPolicy maxArrayPolicy() const
  {
    return _discard_large_array;
  }


  void registerMessageType(const std::string& message_identifier,
                           const std::string& type_name);

  /**
   * @brief deserializeIntoFlatContainer takes a raw buffer of memory and extract information from it.
   *  This data is stored in two key/value vectors, FlatMessage::value and FlatMessage::name.
   * It must be noted that the key type is StringTreeLeaf. this type is not particularly user-friendly,
   * but allows a much faster post-processing.
   *
   * IMPORTANT: this approach is not meant to be used with use arrays such as maps, point clouds and images.
   * It would require a ridicoulous amount of memory and, franckly, make little sense.
   * For this reason the argument max_array_size is used.
   *
   * This funtion is almost always followed by applyNameTransform, which provide a more human-readable
   * key-value representation.
   *
   * @param msg_identifier   String ID to identify the registered message (use registerMessageDefinition first).
   * @param buffer           raw memory to be parsed.
   * @param flat_container_output  output to store the result. It is recommended to reuse the same object multiple times to
   *                               avoid memory allocations and speed up the parsing.
   * @param max_array_size   Usually we want to avoid special cases like maps and images, which contain very large arrays.
   *                         max_array_size is used to skip these arrays that are too large.
   *
   * return true if the entire message was parsed or false if parts of the message were
   * skipped because an array has (size > max_array_size)
   */
  bool deserializeIntoFlatMessage(const std::string& msg_identifier,
                                  const rcutils_uint8_array_t *msg,
                                  FlatMessage* flat_container_output,
                                  const uint32_t max_array_size ) const;


  const rosidl_message_type_support_t* getIntrospectionSupport(const std::string& msg_identifier)
  {
      const auto message_info_it = _registered_messages.find(msg_identifier);
      if(message_info_it == _registered_messages.end())
      {
          throw std::runtime_error("Message identifier not registered");
      }
      return message_info_it->second.type_support;
  }

private:

  std::unordered_map<std::string, Ros2MessageInfo> _registered_messages;

  MaxArrayPolicy _discard_large_array;
};

void ConvertFlatMessageToRenamedValues(const FlatMessage& flat, RenamedValues& renamed );

bool TypeHasHeader(const rosidl_message_type_support_t* type_support);

}

#ifndef RO2_SERIALIZE_UTILS_H
#define RO2_SERIALIZE_UTILS_H


#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/error_handling.h>


class RmwInterface
{
public:
  RmwInterface()
  {
    rcutils_allocator_ = rcutils_get_default_allocator();
  }

  ~RmwInterface() = default;

  template<typename T>
  std::shared_ptr<rmw_serialized_message_t> serialize_message(
    const T& message,
    const rosidl_message_type_support_t * type_support)
  {
    auto serialized_message = get_initialized_serialized_message(0);
    auto error = rmw_serialize(
      &message,
      type_support,
      serialized_message.get());
    if (error != RCL_RET_OK) {
      throw std::runtime_error("Failed to serialize");
    }
    return serialized_message;
  }

  template<typename T>
    std::shared_ptr<T> deserialize_message(std::shared_ptr<rmw_serialized_message_t> serialized_msg,
                       const rosidl_message_type_support_t * type_support)
  {
    auto message = std::make_shared<T>();
    auto error = rmw_deserialize(
      serialized_msg.get(),
      //get_message_typesupport(message),
      type_support,
      message.get());
    if (error != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rosbag2_test_common", "Leaking memory. Error: %s",
        rcutils_get_error_string().str);
    }
    return message;
  }


private:

  std::shared_ptr<rmw_serialized_message_t>
  get_initialized_serialized_message(size_t capacity)
  {
    auto msg = new rmw_serialized_message_t;
    *msg = rmw_get_zero_initialized_serialized_message();
    auto ret = rmw_serialized_message_init(msg, capacity, &rcutils_allocator_);
    if (ret != RCUTILS_RET_OK) {
      throw std::runtime_error(
        "Error allocating resources for serialized message: " +
        std::string(rcutils_get_error_string().str));
    }

    auto serialized_message = std::shared_ptr<rmw_serialized_message_t>(
      msg,
      [](rmw_serialized_message_t * msg) {
        int error = rmw_serialized_message_fini(msg);
        delete msg;
        if (error != RCUTILS_RET_OK) {
          RCUTILS_LOG_ERROR_NAMED(
            "rosbag2_test_common", "Leaking memory. Error: %s",
            rcutils_get_error_string().str);
        }
      });
    return serialized_message;
  }

  rcutils_allocator_t rcutils_allocator_;
};

#endif // UTILS_H

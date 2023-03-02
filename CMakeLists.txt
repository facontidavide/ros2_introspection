cmake_minimum_required(VERSION 3.5)
project(ros2_introspection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS container)

#------------------

add_library(${PROJECT_NAME} SHARED
    src/ros2_introspection.cpp
    src/stringtree.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  rclcpp
  rcpputils
  rmw_fastrtps_cpp
  rosbag2_cpp
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME}
  Boost::container
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

#------------------

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest_executable(ros2_parsing_test test/ros2_introspection_test.cpp)

  target_include_directories(ros2_parsing_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(ros2_parsing_test ${PROJECT_NAME})

endif()

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Boost
  geometry_msgs
  rclcpp
  rcpputils
  rmw_fastrtps_cpp
  rosbag2_cpp
  sensor_msgs
)
ament_package(CONFIG_EXTRAS "cmake/ConfigExtras.cmake")

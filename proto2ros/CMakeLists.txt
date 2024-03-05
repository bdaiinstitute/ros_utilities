# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)  # to accommodate rosidl pipeline
endif()
project(proto2ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Any.msg msg/AnyProto.msg msg/Bytes.msg msg/List.msg
  msg/Struct.msg msg/StructEntry.msg msg/Value.msg
)

include(cmake/rosidl_helpers.cmake)
rosidl_generated_python_package_add(
  ${PROJECT_NAME}_additional_modules
  PACKAGES ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)

ament_package(CONFIG_EXTRAS "proto2ros-extras.cmake")

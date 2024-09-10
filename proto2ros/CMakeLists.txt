# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
cmake_minimum_required(VERSION 3.9)
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
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Protobuf REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Any.msg msg/AnyProto.msg msg/Bytes.msg msg/List.msg
  msg/Struct.msg msg/StructEntry.msg msg/Value.msg
)

add_library(${PROJECT_NAME}_conversions SHARED src/conversions.cpp)
target_include_directories(${PROJECT_NAME}_conversions PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
rosidl_get_typesupport_target(${PROJECT_NAME}_cpp_msgs ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_conversions ${${PROJECT_NAME}_cpp_msgs} protobuf::libprotobuf)
ament_target_dependencies(${PROJECT_NAME}_conversions builtin_interfaces rclcpp std_msgs)

include(cmake/rosidl_helpers.cmake)
rosidl_generated_python_package_add(
  ${PROJECT_NAME}_additional_modules
  PACKAGES ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_conversions
  EXPORT  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(${PROJECT_NAME})

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)

ament_package(CONFIG_EXTRAS "proto2ros-extras.cmake")

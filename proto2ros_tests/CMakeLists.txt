# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
cmake_minimum_required(VERSION 3.12)
project(proto2ros_tests)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(proto2ros REQUIRED)

add_subdirectory(proto)

proto2ros_generate(
  ${PROJECT_NAME}_messages_gen
  PROTOS proto/test.proto
  CONFIG_OVERLAYS config/overlay.yaml
  INTERFACES_OUT_VAR ros_messages
  PYTHON_OUT_VAR py_sources
  APPEND_PYTHONPATH "${PROTO_OUT_DIR}"
)
add_dependencies(
  ${PROJECT_NAME}_messages_gen ${PROJECT_NAME}_proto_gen
)

rosidl_generate_interfaces(
  ${PROJECT_NAME} ${ros_messages}
  DEPENDENCIES geometry_msgs builtin_interfaces proto2ros
  SKIP_INSTALL
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_messages_gen)

rosidl_generated_python_package_add(
  ${PROJECT_NAME}_additional_modules
  MODULES ${proto_py_sources} ${py_sources}
  PACKAGES ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  get_rosidl_generated_interfaces_test_setup(
    LIBRARY_DIRS additional_library_dirs
    ENV additional_env
  )
  list(APPEND additional_env "PYTHONPATH=${PROTO_OUT_DIR}")

  ament_add_pytest_test(
    regression_tests test
    APPEND_LIBRARY_DIRS ${additional_library_dirs}
    APPEND_ENV ${additional_env}
    ENV CMAKE_BINARY_DIR=${CMAKE_BINARY_DIR}
    PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
  )
endif()

ament_package()

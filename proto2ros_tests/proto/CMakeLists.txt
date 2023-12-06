# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
find_package(Protobuf REQUIRED)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bosdyn/api")
file(TOUCH "${CMAKE_CURRENT_BINARY_DIR}/bosdyn/__init__.py")
file(TOUCH "${CMAKE_CURRENT_BINARY_DIR}/bosdyn/api/__init__.py")

protobuf_generate(
  LANGUAGE python
  OUT_VAR bosdyn_proto_py_sources
  PROTOS bosdyn/api/geometry.proto
)

protobuf_generate(
  LANGUAGE python
  OUT_VAR proto_py_sources
  PROTOS test.proto
)

add_custom_target(
  ${PROJECT_NAME}_proto_gen ALL DEPENDS
  ${proto_py_sources} ${bosdyn_proto_py_sources}
)

set(PROTO_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}" PARENT_SCOPE)

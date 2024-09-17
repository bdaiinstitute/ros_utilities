# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

# Vendors a package providing Protobuf messages, adding ROS 2 interoperability interfaces.
#
# :param target: the target name for the process, so it can be depended on.
# :param PACKAGE_NAME: name of the package that will host the generated interfaces.
#   Defaults to the current project name.
# :param PROTOS: Protobuf message files to generate interoperability interfaces for.
#   These are compiled to a single Protobuf descriptor file for further processing.
# :param IMPORT_DIRS: optional import paths to pass to protoc. Only used when PROTOS
#   are provided. If none is given, parent directories of PROTOS are used instead.
# :param CONFIG_OVERLAYS: optional configuration file overlays to be applied sequentially
#   over the default base configuration file.
#
macro(proto2ros_vendor_package target)
  set(options NO_LINT)
  set(one_value_keywords PACKAGE_NAME)
  set(multi_value_keywords PROTOS IMPORT_DIRS CONFIG_OVERLAYS ROS_DEPENDENCIES PYTHON_MODULES PYTHON_PACKAGES)
  cmake_parse_arguments(ARG "${options}" "${one_value_keywords}" "${multi_value_keywords}" ${ARGN})

  if(NOT ARG_PACKAGE_NAME)
    set(ARG_PACKAGE_NAME ${PROJECT_NAME})
  endif()

  set(proto2ros_generate_OPTIONS)
  if(ARG_NO_LINT)
    list(APPEND proto2ros_generate_OPTIONS NO_LINT)
  endif()

  proto2ros_generate(
    ${target}_messages_gen
    PROTOS ${ARG_PROTOS}
    IMPORT_DIRS ${ARG_IMPORT_DIRS}
    PACKAGE_NAME ${ARG_PACKAGE_NAME}
    CONFIG_OVERLAYS ${ARG_CONFIG_OVERLAYS}
    INTERFACES_OUT_VAR ros_messages
    PYTHON_OUT_VAR py_sources
    CPP_OUT_VAR cpp_sources
    ${proto2ros_generate_OPTIONS}
  )

  rosidl_generate_interfaces(
    ${target} ${ros_messages}
    DEPENDENCIES ${ARG_ROS_DEPENDENCIES} builtin_interfaces proto2ros
  )
  add_dependencies(${target} ${target}_messages_gen)

  get_filename_component(package_path "${ARG_PACKAGE_NAME}" ABSOLUTE)
  if(EXISTS "${package_path}/__init__.py")
    list(APPEND ARG_PYTHON_PACKAGES ${ARG_PACKAGE_NAME})
  endif()

  rosidl_generated_python_package_add(
    ${target}_additional_modules
    MODULES ${ARG_PYTHON_MODULES} ${py_sources}
    PACKAGES ${ARG_PYTHON_PACKAGES}
    DESTINATION ${target}
  )

  find_package(rclcpp REQUIRED)
  add_library(${target}_conversions ${cpp_sources})
  rosidl_get_typesupport_target(cpp_interfaces ${target} "rosidl_typesupport_cpp")
  target_link_libraries(${target}_conversions ${cpp_interfaces})
  ament_target_dependencies(${target}_conversions
    ${ARG_ROS_DEPENDENCIES} builtin_interfaces proto2ros rclcpp)

  set(header_files ${cpp_sources})
  list(FILTER header_files INCLUDE REGEX ".*\.hpp$")
  install(
    FILES ${header_files}
    DESTINATION include/${PROJECT_NAME}
  )
  ament_export_include_directories("include/${PROJECT_NAME}")

  install(
    TARGETS ${target}_conversions
    EXPORT export_${target}_conversions
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_targets(export_${target}_conversions)
endmacro()

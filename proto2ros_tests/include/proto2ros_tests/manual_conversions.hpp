// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#pragma once

#include <bosdyn/api/geometry.pb.h>
#include <test.pb.h>

#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <sensor_msgs/msg/temperature.hpp>

namespace proto2ros_tests {
namespace conversions {

void convert(const geometry_msgs::msg::Vector3& ros_msg, bosdyn::api::Vec3* proto_msg);

void convert(const bosdyn::api::Vec3& proto_msg, geometry_msgs::msg::Vector3* ros_msg);

void convert(const geometry_msgs::msg::Point& ros_msg, bosdyn::api::Vec3* proto_msg);

void convert(const bosdyn::api::Vec3& proto_msg, geometry_msgs::msg::Point* ros_msg);

void convert(const geometry_msgs::msg::Quaternion& ros_msg, bosdyn::api::Quaternion* proto_msg);

void convert(const bosdyn::api::Quaternion& proto_msg, geometry_msgs::msg::Quaternion* ros_msg);

void convert(const geometry_msgs::msg::Pose& ros_msg, bosdyn::api::SE3Pose* proto_msg);

void convert(const bosdyn::api::SE3Pose& proto_msg, geometry_msgs::msg::Pose* ros_msg);

void convert(const geometry_msgs::msg::Pose& ros_msg, bosdyn::api::SE2Pose* proto_msg);

void convert(const bosdyn::api::SE2Pose& proto_msg, geometry_msgs::msg::Pose* ros_msg);

void convert(const geometry_msgs::msg::Polygon& ros_msg, bosdyn::api::Polygon* proto_msg);

void convert(const bosdyn::api::Polygon& proto_msg, geometry_msgs::msg::Polygon* ros_msg);

void convert(const geometry_msgs::msg::Vector3& ros_msg, bosdyn::api::Circle* proto_msg);

void convert(const bosdyn::api::Circle& proto_msg, geometry_msgs::msg::Vector3* ros_msg);

void convert(const geometry_msgs::msg::Twist& ros_msg, bosdyn::api::SE3Velocity* proto_msg);

void convert(const bosdyn::api::SE3Velocity& proto_msg, geometry_msgs::msg::Twist* ros_msg);

void convert(const geometry_msgs::msg::Wrench& ros_msg, bosdyn::api::Wrench* proto_msg);

void convert(const bosdyn::api::Wrench& proto_msg, geometry_msgs::msg::Wrench* ros_msg);

void convert(const sensor_msgs::msg::Temperature& ros_msg, Temperature* proto_msg);

void convert(const Temperature& proto_msg, sensor_msgs::msg::Temperature* ros_msg);

}  // namespace conversions
}  // namespace proto2ros_tests

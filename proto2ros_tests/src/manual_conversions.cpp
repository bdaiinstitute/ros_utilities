// Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

#include "proto2ros_tests/manual_conversions.hpp"

#include <cmath>

namespace proto2ros_tests {
namespace conversions {

void convert(const geometry_msgs::msg::Vector3& ros_msg, bosdyn::api::Vec3* proto_msg) {
    proto_msg->set_x(ros_msg.x);
    proto_msg->set_y(ros_msg.y);
    proto_msg->set_z(ros_msg.z);
}

void convert(const bosdyn::api::Vec3& proto_msg, geometry_msgs::msg::Vector3* ros_msg) {
    ros_msg->x = proto_msg.x();
    ros_msg->y = proto_msg.y();
    ros_msg->z = proto_msg.z();
}

void convert(const geometry_msgs::msg::Point& ros_msg, bosdyn::api::Vec3* proto_msg) {
    proto_msg->set_x(ros_msg.x);
    proto_msg->set_y(ros_msg.y);
    proto_msg->set_z(ros_msg.z);
}

void convert(const bosdyn::api::Vec3& proto_msg, geometry_msgs::msg::Point* ros_msg) {
    ros_msg->x = proto_msg.x();
    ros_msg->y = proto_msg.y();
    ros_msg->z = proto_msg.z();
}

void convert(const geometry_msgs::msg::Quaternion& ros_msg, bosdyn::api::Quaternion* proto_msg) {
    proto_msg->set_x(ros_msg.x);
    proto_msg->set_y(ros_msg.y);
    proto_msg->set_z(ros_msg.z);
    proto_msg->set_w(ros_msg.w);
}

void convert(const bosdyn::api::Quaternion& proto_msg, geometry_msgs::msg::Quaternion* ros_msg) {
    ros_msg->x = proto_msg.x();
    ros_msg->y = proto_msg.y();
    ros_msg->z = proto_msg.z();
    ros_msg->w = proto_msg.w();
}

void convert(const geometry_msgs::msg::Pose& ros_msg, bosdyn::api::SE3Pose* proto_msg) {
    convert(ros_msg.position, proto_msg->mutable_position());
    convert(ros_msg.orientation, proto_msg->mutable_rotation());
}

void convert(const bosdyn::api::SE3Pose& proto_msg, geometry_msgs::msg::Pose* ros_msg) {
    convert(proto_msg.position(), &ros_msg->position);
    convert(proto_msg.rotation(), &ros_msg->orientation);
}

void convert(const geometry_msgs::msg::Pose& ros_msg, bosdyn::api::SE2Pose* proto_msg) {
    proto_msg->mutable_position()->set_x(ros_msg.position.x);
    proto_msg->mutable_position()->set_y(ros_msg.position.y);
    proto_msg->set_angle(2.0 * std::acos(ros_msg.orientation.w));
}

void convert(const bosdyn::api::SE2Pose& proto_msg, geometry_msgs::msg::Pose* ros_msg) {
    ros_msg->position.x = proto_msg.position().x();
    ros_msg->position.y = proto_msg.position().y();
    ros_msg->orientation.w = std::cos(proto_msg.angle() / 2.0);
    ros_msg->orientation.z = std::sin(proto_msg.angle() / 2.0);
}

void convert(const geometry_msgs::msg::Polygon& ros_msg, bosdyn::api::Polygon* proto_msg) {
    proto_msg->Clear();
    auto* vertices = proto_msg->mutable_vertexes();
    vertices->Reserve(ros_msg.points.size());
    for (const auto& point : ros_msg.points) {
        auto* vertex = vertices->Add();
        vertex->set_x(point.x);
        vertex->set_y(point.y);
    }
}

void convert(const bosdyn::api::Polygon& proto_msg, geometry_msgs::msg::Polygon* ros_msg) {
    ros_msg->points.clear();
    ros_msg->points.reserve(proto_msg.vertexes_size());
    for (const auto& vertex : proto_msg.vertexes()) {
        auto& point = ros_msg->points.emplace_back();
        point.x = vertex.x();
        point.y = vertex.y();
    }
}

void convert(const geometry_msgs::msg::Vector3& ros_msg, bosdyn::api::Circle* proto_msg) {
    auto* center = proto_msg->mutable_center_pt();
    center->set_x(ros_msg.x);
    center->set_y(ros_msg.y);
    proto_msg->set_radius(ros_msg.z);
}

void convert(const bosdyn::api::Circle& proto_msg, geometry_msgs::msg::Vector3* ros_msg) {
    const auto& center = proto_msg.center_pt();
    ros_msg->x = center.x();
    ros_msg->y = center.y();
    ros_msg->z = proto_msg.radius();
}

void convert(const geometry_msgs::msg::Twist& ros_msg, bosdyn::api::SE3Velocity* proto_msg) {
    convert(ros_msg.linear, proto_msg->mutable_linear());
    convert(ros_msg.angular, proto_msg->mutable_angular());
}

void convert(const bosdyn::api::SE3Velocity& proto_msg, geometry_msgs::msg::Twist* ros_msg) {
    convert(proto_msg.linear(), &ros_msg->linear);
    convert(proto_msg.angular(), &ros_msg->angular);
}

void convert(const geometry_msgs::msg::Wrench& ros_msg, bosdyn::api::Wrench* proto_msg) {
    convert(ros_msg.force, proto_msg->mutable_force());
    convert(ros_msg.torque, proto_msg->mutable_torque());
}

void convert(const bosdyn::api::Wrench& proto_msg, geometry_msgs::msg::Wrench* ros_msg) {
    convert(proto_msg.force(), &ros_msg->force);
    convert(proto_msg.torque(), &ros_msg->torque);
}

void convert(const sensor_msgs::msg::Temperature& ros_msg, Temperature* proto_msg) {
    proto_msg->set_scale(Temperature::CELSIUS);
    proto_msg->set_value(ros_msg.temperature);
}

void convert(const Temperature& proto_msg, sensor_msgs::msg::Temperature* ros_msg) {
    switch (proto_msg.scale()) {
        case Temperature::KELVIN:
            ros_msg->temperature = proto_msg.value() + 273.0;
            break;
        case Temperature::FAHRENHEIT:
            ros_msg->temperature = (proto_msg.value() - 32.0) * 5.0 / 9.0;
            break;
        case Temperature::CELSIUS:
        default:
            ros_msg->temperature = proto_msg.value();
            break;
    }
}

}  // namespace conversions
}  // namespace proto2ros_tests

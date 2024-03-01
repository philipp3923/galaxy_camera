#pragma once
#include <rclcpp/rclcpp.hpp>

inline rclcpp::QoS
FAST_PERFORMANCE() {
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    qos.history(rclcpp::HistoryPolicy::KeepLast);
    return qos;
}

inline rclcpp::QoS
HIGH_RELIABILITY() {
    rclcpp::QoS qos(1);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.history(rclcpp::HistoryPolicy::KeepLast);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    return qos;
}


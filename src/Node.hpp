#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "QoS.hpp"

class Node : public rclcpp::Node {
private:
    rclcpp::Clock::SharedPtr clock;
    rclcpp::Publisher<sensor_msgs::msg::Image> image_publisher;
};
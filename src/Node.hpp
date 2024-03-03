#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/detail/compressed_image__struct.hpp>
#include "QoS.hpp"
#include "Camera.hpp"

struct Parameters {
    std::string image_topic;
    bool enable_white_balance;
    std::vector<long> white_balance_roi;
    std::vector<long> statistical_area_roi;
    bool enable_auto_exposure;
    double manual_exposure;
    std::vector<double> auto_exposure;
    bool enable_auto_gain;
    double manual_gain;
    std::vector<double> auto_gain;
    bool enable_auto_gamma;
    double manual_gamma;
    long trigger_source;
};

class Node : public rclcpp::Node {
private:
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> image_publisher;
    Camera camera;
    std::shared_ptr<rclcpp::TimerBase> timer;
    Parameters params;

    void declare_params();

    void load_params();

    void configure_camera();

    void callback();

public:
    Node();
};
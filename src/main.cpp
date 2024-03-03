#include <optional>
#include <opencv2/opencv.hpp>
#include "Camera.hpp"
#include "Node.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Node>();

    rclcpp::spin_some(node);

    rclcpp::shutdown();
}

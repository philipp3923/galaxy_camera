#include <opencv2/opencv.hpp>
#include "Node.hpp"
#include <cv_bridge/cv_bridge.hpp>

#define DEBUG(...) \
    RCLCPP_DEBUG(this->get_logger(), __VA_ARGS__);

#define INFO(...) \
    RCLCPP_INFO(this->get_logger(), __VA_ARGS__);

#define WARN(...) \
    RCLCPP_WARN(this->get_logger(), __VA_ARGS__);

Node::Node() : rclcpp::Node("galaxy_camera"), camera(Camera()) {
    this->declare_params();
    this->load_params();
    this->configure_camera();

    image_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            params.image_topic, FAST_PERFORMANCE());

    timer = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration(0, 1), [this] { callback(); });
}

void Node::configure_camera() {
    camera.set_statistical_area_roi(params.statistical_area_roi[0], params.statistical_area_roi[1],
                                    params.statistical_area_roi[2], params.statistical_area_roi[3]);
    camera.set_white_balance_roi(params.white_balance_roi[0], params.white_balance_roi[1], params.white_balance_roi[2],
                                 params.white_balance_roi[3]);

    if (params.enable_auto_exposure) {
        camera.set_exposure_auto(params.auto_exposure[0], params.auto_exposure[1]);
    } else {
        camera.set_exposure_manual(params.manual_exposure);
    }

    camera.set_white_balance(params.enable_white_balance);

    if (params.enable_auto_gain) {
        camera.set_gain_auto(params.auto_gain[0], params.auto_gain[1]);
    } else {
        camera.set_gain_manual(params.manual_gain);
    }

    if (params.enable_auto_gamma) {
        camera.set_gamma_auto();
    } else {
        camera.set_gamma_manual(params.manual_gamma);
    }

    camera.set_trigger_source(static_cast<TriggerSource>(params.trigger_source));

    camera.set_stream_buffer_mode(NewestOnly);
}

void Node::callback() {
    timer->cancel();

    camera.start_capturing();

    while (true) {
        std::optional<Capture> opt_capture = camera.get_image(1000);

        if (!opt_capture.has_value()) {
            continue;
        }

        Capture capture = opt_capture.value();

        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8",
                                            capture.image).toCompressedImageMsg(cv_bridge::JPG);

        image_msg->header.stamp = capture.timestamp;

        image_publisher->publish(*image_msg.get());

        if (!rclcpp::ok()) {
            break;
        }
    }
}

void Node::declare_params() {
    this->declare_parameter("image_topic", "/camera/image");

    this->declare_parameter("enable_white_balance", true);
    this->declare_parameter("white_balance_roi", std::vector<int>({544, 148, 3024, 1788}));

    this->declare_parameter("statistical_area_roi", std::vector<int>({544, 148, 3024, 1788}));

    this->declare_parameter("enable_auto_exposure", true);
    this->declare_parameter("manual_exposure", 100.0);
    this->declare_parameter("auto_exposure", std::vector<double>({11.0, 5000.0}));

    this->declare_parameter("enable_auto_gain", true);
    this->declare_parameter("manual_gain", 0.0);
    this->declare_parameter("auto_gain", std::vector<double>({0.0, 16.0}));

    this->declare_parameter("enable_auto_gamma", true);
    this->declare_parameter("manual_gamma", 0.0);

    this->declare_parameter("trigger_source", 0);
}

void Node::load_params() {
    params.image_topic = this->get_parameter("image_topic").as_string();

    params.enable_white_balance = this->get_parameter("enable_white_balance").as_bool();
    params.white_balance_roi = this->get_parameter("white_balance_roi").as_integer_array();

    params.statistical_area_roi = this->get_parameter("statistical_area_roi").as_integer_array();

    params.enable_auto_exposure = this->get_parameter("enable_auto_exposure").as_bool();
    params.manual_exposure = this->get_parameter("manual_exposure").as_double();
    params.auto_exposure = this->get_parameter("auto_exposure").as_double_array();

    params.enable_auto_gain = this->get_parameter("enable_auto_gain").as_bool();
    params.manual_gain = this->get_parameter("manual_gain").as_double();
    params.auto_gain = this->get_parameter("auto_gain").as_double_array();

    params.enable_auto_gamma = this->get_parameter("enable_auto_gamma").as_bool();
    params.manual_gamma = this->get_parameter("manual_gamma").as_double();

    params.trigger_source = this->get_parameter("trigger_source").as_int();

    INFO("image_topic: %s", params.image_topic.c_str());
    INFO("enable_white_balance: %d", params.enable_white_balance);
    INFO("white_balance_roi: %ld, %ld, %ld, %ld", params.white_balance_roi[0], params.white_balance_roi[1],
         params.white_balance_roi[2], params.white_balance_roi[3]);
    INFO("statistical_area_roi: %ld, %ld, %ld, %ld", params.statistical_area_roi[0], params.statistical_area_roi[1],
         params.statistical_area_roi[2], params.statistical_area_roi[3]);
    INFO("enable_auto_exposure: %d", params.enable_auto_exposure);
    INFO("manual_exposure: %f", params.manual_exposure);
    INFO("auto_exposure: %f, %f", params.auto_exposure[0], params.auto_exposure[1]);
    INFO("enable_auto_gain: %d", params.enable_auto_gain);
    INFO("manual_gain: %f", params.manual_gain);
    INFO("auto_gain: %f, %f", params.auto_gain[0], params.auto_gain[1]);
    INFO("enable_auto_gamma: %d", params.enable_auto_gamma);
    INFO("manual_gamma: %f", params.manual_gamma);
    INFO("trigger_source: %u", static_cast<TriggerSource>(params.trigger_source));
}

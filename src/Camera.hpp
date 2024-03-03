#pragma once

#include <exception>
#include <GxIAPI.h>
#include <opencv2/core/mat.hpp>
#include <chrono>
#include <rclcpp/logger.hpp>
#include <rclcpp/clock.hpp>

enum TriggerSource {
    CONTINUOUS,
    SOFTWARE,
    LINE_0,
    LINE_2,
    LINE_3
};

enum StreamBufferMode {
    OldestFirst,
    OldestFirstOverwrite,
    NewestOnly
};

struct Capture {
    cv::Mat image;
    rclcpp::Time timestamp;
    int32_t width;
    int32_t height;
    uint64_t frame_id;
};

class Camera {
private:
    GX_STATUS status = GX_STATUS_SUCCESS;
    GX_DEV_HANDLE device = nullptr;
    PGX_FRAME_BUFFER frame_buffer = nullptr;
    int64_t payload_size = 0;
    int64_t color_filter = GX_COLOR_FILTER_NONE;
    unsigned char *rgb_image_buffer = nullptr;
    rclcpp::Logger logger;
    rclcpp::Clock clock;


public:
    Camera();

    virtual ~Camera();

    /**
     * set the exposure to manual
     * @param exposure to be set
     */
    void set_exposure_manual(double exposure);

    /**
     * set the exposure to be automatically determined by the camera
     * @param min_exposure to be set
     * @param max_exposure to be set
     */
    void set_exposure_auto(double min_exposure, double max_exposure);

    /**
     * set the region of interest of the camera frames
     * @param x
     * @param y
     * @param width
     * @param height
     */
    void set_roi(long x, long y, long width, long height);

    /**
     * set the region of interest which is used for statistical processing within the camera
     * @param x
     * @param y
     * @param width
     * @param height
     */
    void set_statistical_area_roi(long x, long y, long width, long height);

    /**
     * set the region of interest for automatic white balance determination
     * @param x
     * @param y
     * @param width
     * @param height
     */
    void set_white_balance_roi(long x, long y, long width, long height);

    /**
     * set the source of the trigger to be either software based or hardware based
     * @param source
     */
    void set_trigger_source(TriggerSource source);

    /**
     * set manual gain for the camera.
     * @param gain by which the brightness of the image is enhanced
     */
    void set_gain_manual(double gain);

    /**
     * set the gain to an automatic value
     * @param min_gain to be set
     * @param max_gain to be set
     */
    void set_gain_auto(double min_gain, double max_gain);

    /**
     * set a manual gamma value
     * @param gamma
     */
    void set_gamma_manual(double gamma);

    /**
     * deactivates the gamma feature
     */
    void set_gamma_manual();

    /**
     * activate automatic gamma detection
     */
    void set_gamma_auto();

    /**
     * activate automatic white balance
     * @param on
     */
    void set_white_balance(bool on);

    /**
     * set the stream buffer mode
     * @param mode
     */
    void set_stream_buffer_mode(StreamBufferMode mode);

    /**
     * starts capturing images
     */
    void start_capturing();

    /**
     * stops capturing images
     */
    void stop_capturing();

    /**
     * receives an image from the camera within the given timeout.
     * start_capturing() must be called in advance.
     * @param timeout
     * @return captured image
     */
    std::optional<Capture> get_image(int64_t timeout);
};

/**
 * Retrieves the error message to the corresponding status.
 * This method was copied from the GalaxySDK samples.
 * @param emErrSorStatus
 */
const char *GetErrorString(GX_STATUS emErrorStatus);

class CameraException : public std::exception {
private:
    std::string msg;
public:
    explicit CameraException(std::string msg) : msg(std::move(msg)) {}

    [[nodiscard]] const char *what() const noexcept override { return msg.data(); }
};


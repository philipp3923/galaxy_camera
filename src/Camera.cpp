#include <string>
#include <utility>
#include <optional>
#include <chrono>
#include <DxImageProc.h>
#include <opencv2/imgproc.hpp>
#include "Camera.hpp"
#include <rclcpp/rclcpp.hpp>

#define GX_VERIFY(status) \
    if (status != GX_STATUS_SUCCESS) \
    { \
         throw CameraException(GetErrorString(status)); \
    } \

#define DX_VERIFY(status) \
    if (status != DX_OK) \
    { \
        throw CameraException("DxRaw8toRGB24 Failed, Error Code: " + std::to_string(status) + "\n"); \
    } \

#define INFO(...) \
    RCLCPP_INFO(logger __VA_ARGS__); \


#define WARN(...) \
    RCLCPP_WARN(logger, __VA_ARGS__); \


Camera::Camera() : logger(rclcpp::get_logger("camera_api")) {
    status = GXInitLib();
    GX_VERIFY(status);

    uint32_t device_num = 0;
    status = GXUpdateAllDeviceList(&device_num, 1000);
    GX_VERIFY(status);

    if (device_num <= 0) {
        GXCloseLib();
        throw CameraException("No device found\n");
    }

    status = GXOpenDeviceByIndex(1, &device);
    GX_VERIFY(status);

    status = GXGetInt(device, GX_INT_PAYLOAD_SIZE, &payload_size);
    GX_VERIFY(status);

    rgb_image_buffer = new unsigned char[payload_size * 3];

    status = GXGetEnum(device, GX_ENUM_PIXEL_COLOR_FILTER, &color_filter);
    GX_VERIFY(status);
}

void Camera::set_exposure_manual(double exposure) {
    status = GXSetFloat(device, GX_FLOAT_EXPOSURE_TIME, exposure);
    GX_VERIFY(status);
}

void Camera::set_exposure_auto(double min_exposure, double max_exposure) {
    status = GXSetFloat(device, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, min_exposure);
    GX_VERIFY(status);
    status = GXSetFloat(device, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, max_exposure);
    GX_VERIFY(status);
}

void Camera::set_roi(long x, long y, long width, long height) {
    status = GXSetInt(device, GX_INT_HEIGHT, width);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_WIDTH, height);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_OFFSET_X, x);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_OFFSET_Y, y);
    GX_VERIFY(status);
}

void Camera::set_statistical_area_roi(long x, long y, long width, long height) {
    status = GXSetInt(device, GX_INT_AAROI_WIDTH, width);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_AAROI_HEIGHT, height);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_AAROI_OFFSETX, x);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_AAROI_OFFSETY, y);
    GX_VERIFY(status);
}

void Camera::set_white_balance_roi(long x, long y, long width, long height) {
    status = GXSetInt(device, GX_INT_AWBROI_WIDTH, width);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_AWBROI_HEIGHT, height);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_AWBROI_OFFSETX, x);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_AWBROI_OFFSETY, y);
    GX_VERIFY(status);
}

void Camera::set_trigger_source(TriggerSource source) {
    switch (source) {
        case CONTINUOUS:
            status = GXSetEnum(device, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
            GX_VERIFY(status);
            status = GXSetEnum(device, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
            GX_VERIFY(status);
            break;
        default:
            status = GXSetEnum(device, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
            GX_VERIFY(status);
            status = GXSetEnum(device, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_SINGLE_FRAME);
            GX_VERIFY(status);
        case SOFTWARE:
            status = GXSetEnum(device, GX_ENUM_TRIGGER_SOURCE,
                               GX_TRIGGER_SOURCE_SOFTWARE);
            GX_VERIFY(status);
            break;
        case LINE_0:
            status = GXSetEnum(device, GX_ENUM_TRIGGER_SOURCE,
                               GX_TRIGGER_SOURCE_LINE0);
            GX_VERIFY(status);
            break;
        case LINE_2:
            status = GXSetEnum(device, GX_ENUM_TRIGGER_SOURCE,
                               GX_TRIGGER_SOURCE_LINE2);
            GX_VERIFY(status);
            break;
        case LINE_3:
            status = GXSetEnum(device, GX_ENUM_TRIGGER_SOURCE,
                               GX_TRIGGER_SOURCE_LINE3);
            GX_VERIFY(status);
            break;
    }
}

void Camera::set_gain_manual(double gain) {
    status = GXSetEnum(device, GX_ENUM_GAIN_AUTO,
                       GX_GAIN_AUTO_OFF);
    GX_VERIFY(status);
    status = GXSetFloat(device, GX_FLOAT_GAIN, gain);
    GX_VERIFY(status);
}

void Camera::set_gain_auto(double min_gain, double max_gain) {
    status = GXSetFloat(device, GX_FLOAT_AUTO_GAIN_MIN, min_gain);
    GX_VERIFY(status);
    status = GXSetFloat(device, GX_FLOAT_AUTO_GAIN_MAX, max_gain);
    GX_VERIFY(status);
    status = GXSetEnum(device, GX_ENUM_GAIN_AUTO,
                       GX_GAIN_AUTO_CONTINUOUS);
    GX_VERIFY(status);
    status = GXSetEnum(device, GX_ENUM_GAIN_SELECTOR,
                       GX_GAIN_SELECTOR_ALL);
    GX_VERIFY(status);
}

void Camera::set_gamma_manual(double gamma) {
    status = GXSetBool(device, GX_BOOL_GAMMA_ENABLE, true);
    GX_VERIFY(status);
    status = GXSetEnum(device, GX_ENUM_GAMMA_MODE, GX_GAMMA_SELECTOR_USER);
    GX_VERIFY(status);
    status = GXSetFloat(device, GX_FLOAT_GAMMA, gamma);
    GX_VERIFY(status);
}

void Camera::set_gamma_auto() {
    status = GXSetBool(device, GX_BOOL_GAMMA_ENABLE, true);
    GX_VERIFY(status);
    status = GXSetEnum(device, GX_ENUM_GAMMA_MODE, GX_GAMMA_SELECTOR_SRGB);
    GX_VERIFY(status);
}

void Camera::set_gamma_manual() {
    status = GXSetBool(device, GX_BOOL_GAMMA_ENABLE, false);
    GX_VERIFY(status);
}

void Camera::set_white_balance(bool on) {
    if (on) {
        status = GXSetEnum(device, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    } else {
        status = GXSetEnum(device, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    }
    GX_VERIFY(status);
}

void Camera::set_stream_buffer_mode(StreamBufferMode mode) {
    switch (mode) {
        case OldestFirst:
            status = GXSetEnum(device, GX_DS_ENUM_STREAM_BUFFER_HANDLING_MODE,
                               GX_DS_STREAM_BUFFER_HANDLING_MODE_OLDEST_FIRST);
            break;
        case OldestFirstOverwrite:
            status = GXSetEnum(device, GX_DS_ENUM_STREAM_BUFFER_HANDLING_MODE,
                               GX_DS_STREAM_BUFFER_HANDLING_MODE_OLDEST_FIRST_OVERWRITE);
            break;
        case NewestOnly:
            status = GXSetEnum(device, GX_DS_ENUM_STREAM_BUFFER_HANDLING_MODE,
                               GX_DS_STREAM_BUFFER_HANDLING_MODE_NEWEST_ONLY);
            break;
    }
    GX_VERIFY(status);
}

std::optional<Capture> Camera::get_image(int64_t timeout) {
    //int64_t tick_frequency = 125000000;

    status = GXDQBuf(device, &frame_buffer, 1000);
    // TODO this timestamp is not accurate. Most functions for timestamp acquisition seem to not be implemented in our camera. Still this should be improved in the future
    auto ros2_time = clock.now();

    if (status == GX_STATUS_TIMEOUT) {
        WARN("Acquisition timed out");
        return std::nullopt;
    }

    if (frame_buffer->nStatus != GX_FRAME_STATUS_SUCCESS) {
        WARN("Abnormal Acquisition: Exception code: %d", frame_buffer->nStatus);
        status = GXQBuf(device, frame_buffer);
        GX_VERIFY(status);
        return std::nullopt;
    }

    GX_VERIFY(status);

    Capture cap;

    cap.height = frame_buffer->nHeight;
    cap.width = frame_buffer->nWidth;
    cap.frame_id = frame_buffer->nFrameID;
    cap.timestamp = ros2_time;
    cap.image = cv::Mat(cap.height, cap.width, CV_8UC3);

    // TODO this does not work with Raw16 images!
    status = DxRaw8toRGB24((unsigned char *) frame_buffer->pImgBuf, rgb_image_buffer, frame_buffer->nWidth,
                           frame_buffer->nHeight, RAW2RGB_NEIGHBOUR,
                           DX_PIXEL_COLOR_FILTER(color_filter), false);
    DX_VERIFY(status);

    memcpy(cap.image.data, rgb_image_buffer, frame_buffer->nWidth * frame_buffer->nHeight * 3);
    cv::cvtColor(cap.image, cap.image, cv::COLOR_RGB2BGR);

    status = GXQBuf(device, frame_buffer);
    GX_VERIFY(status);

    return cap;
}

void Camera::start_capturing() {
    status = GXStreamOn(device);
    GX_VERIFY(status);
}

void Camera::stop_capturing() {
    status = GXStreamOff(device);
    GX_VERIFY(status);
}

Camera::~Camera() {
    status = GXCloseDevice(device);
    GX_VERIFY(status);

    device = nullptr;
    GXCloseLib();
}

const char *GetErrorString(GX_STATUS emErrorStatus) {
    static std::string error_info;

    // Get length of error description
    size_t size = 0;
    GX_STATUS emStatus = GXGetLastError(&emErrorStatus, nullptr, &size);
    if (emStatus != GX_STATUS_SUCCESS) {
        return "<Error when calling GXGetLastError>\n";
    }

    // Resize the string to fit the error description
    error_info.resize(size);

    // Get error description
    emStatus = GXGetLastError(&emErrorStatus, &error_info[0], &size);
    if (emStatus != GX_STATUS_SUCCESS) {
        return "<Error when calling GXGetLastError>\n";
    }

    return error_info.c_str();
}
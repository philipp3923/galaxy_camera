#include <string>
#include <utility>
#include <iostream>
#include "Camera.hpp"
#include "../include/GxIAPI.h"

#define GX_VERIFY(status) \
    if (status != GX_STATUS_SUCCESS) \
    { \
         throw CameraException(GetErrorString(status)); \
    } \


Camera::Camera() {
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

void Camera::set_roi(int x, int y, int width, int height) {
    status = GXSetInt(device, GX_INT_OFFSET_X, x);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_OFFSET_Y, y);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_HEIGHT, width);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_WIDTH, height);
    GX_VERIFY(status);
}

void Camera::set_statistical_area_roi(int x, int y, int width, int height) {
    status = GXSetInt(device, GX_INT_AAROI_OFFSETX, x);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_AAROI_OFFSETY, y);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_AAROI_WIDTH, width);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_AAROI_HEIGHT, height);
    GX_VERIFY(status);
}

void Camera::set_white_balance_roi(int x, int y, int width, int height) {
    status = GXSetInt(device, GX_INT_AWBROI_OFFSETX, x);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_AWBROI_OFFSETY, y);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_AWBROI_WIDTH, width);
    GX_VERIFY(status);
    status = GXSetInt(device, GX_INT_AWBROI_HEIGHT, height);
    GX_VERIFY(status);
}

void Camera::set_trigger_source(TriggerSource source) {
    switch (source) {
        case CONTINUOUS:
            status = GXSetEnum(device, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
            GX_VERIFY(status);
            break;
        default:
            status = GXSetEnum(device, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_SINGLE_FRAME);
            GX_VERIFY(status);
        case SOFTWARE:
            status = GXSetEnum(device, GX_ENUM_COUNTER_TRIGGER_SOURCE,
                               GX_COUNTER_TRIGGER_SOURCE_SOFTWARE);
            GX_VERIFY(status);
            break;
        case LINE_0:
            status = GXSetEnum(device, GX_ENUM_COUNTER_TRIGGER_SOURCE,
                               GX_COUNTER_TRIGGER_SOURCE_LINE0);
            GX_VERIFY(status);
            break;
        case LINE_2:
            status = GXSetEnum(device, GX_ENUM_COUNTER_TRIGGER_SOURCE,
                               GX_COUNTER_TRIGGER_SOURCE_LINE2);
            GX_VERIFY(status);
            break;
        case LINE_3:
            status = GXSetEnum(device, GX_ENUM_COUNTER_TRIGGER_SOURCE,
                               GX_COUNTER_TRIGGER_SOURCE_LINE3);
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
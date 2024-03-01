#include <cstdio>
#include <DxImageProc.h>
#include <opencv2/opencv.hpp>
#include "../include/GxIAPI.h"
#include "Camera.hpp"
#include "Node.hpp"

#include <rclcpp/rclcpp.hpp>

//Show error message
#define GX_VERIFY(status) \
    if (status != GX_STATUS_SUCCESS) \
    { \
        GetErrorString(status); \
        return status; \
    } \
//Show error message
#define DX_VERIFY(status) \
    if (status != DX_OK) \
    { \
        printf("DxRaw8toRGB24 Failed, Error Code: %d\n", status); \
        return status; \
    } \


//Get description of error
void GetErrorString(GX_STATUS);

int main() {
    GX_STATUS status = GX_STATUS_SUCCESS;

    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS) {
        GetErrorString(status);
        return status;
    }

    uint32_t device_num = 0;

    status = GXUpdateAllDeviceList(&device_num, 1000);
    GX_VERIFY(status);

    if (device_num <= 0) {
        printf("<No device found>\n");
        GXCloseLib();
        return status;
    }

    GX_DEV_HANDLE device = NULL;

    status = GXOpenDeviceByIndex(1, &device);
    GX_VERIFY(status);

    status = GXSetEnum(device, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    GX_VERIFY(status);

    bool bStreamTransferSize = false;
    status = GXIsImplemented(device, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
    GX_VERIFY(status);

    if (bStreamTransferSize) {
        //Set size of data transfer block
        status = GXSetInt(device, GX_DS_INT_STREAM_TRANSFER_SIZE, (64 * 1024));
        GX_VERIFY(status);
    }

    bool bStreamTransferNumberUrb = false;
    status = GXIsImplemented(device, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
    GX_VERIFY(status);

    int64_t color_filter = GX_COLOR_FILTER_NONE;
    status = GXGetEnum(device, GX_ENUM_PIXEL_COLOR_FILTER, &color_filter);
    GX_VERIFY(status);

    if (bStreamTransferNumberUrb) {
        //Set qty. of data transfer block
        status = GXSetInt(device, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, 64);
        GX_VERIFY(status);
    }

    status = GXStreamOn(device);
    GX_VERIFY(status);

    int64_t payload_size = 0;

    status = GXGetInt(device, GX_INT_PAYLOAD_SIZE, &payload_size);
    GX_VERIFY(status);


    auto *rgb_image_buffer = new unsigned char[payload_size * 3];
    auto *raw8_image_buffer = new unsigned char[payload_size];

    PGX_FRAME_BUFFER frame_buffer = nullptr;

    cv::namedWindow("image", cv::WINDOW_NORMAL);

    while (true) {
        status = GXDQBuf(device, &frame_buffer, 1000);
        if (status == GX_STATUS_TIMEOUT) {
            continue;
        }
        GX_VERIFY(status);

        if (frame_buffer->nStatus != GX_FRAME_STATUS_SUCCESS) {
            printf("<Abnormal Acquisition: Exception code: %d>\n", frame_buffer->nStatus);
        } else {
            cv::Mat image(frame_buffer->nHeight, frame_buffer->nWidth, CV_8UC3);

            //status = DxRaw16toRaw8(frame_buffer->pImgBuf, raw8_image_buffer, frame_buffer->nWidth, frame_buffer->nHeight, DX_BIT_2_9);
            //DX_VERIFY(status);

            printf("pixel_format: %d\n", frame_buffer->nPixelFormat);

            status = DxRaw8toRGB24((unsigned char *) frame_buffer->pImgBuf, rgb_image_buffer, frame_buffer->nWidth,
                                   frame_buffer->nHeight, RAW2RGB_NEIGHBOUR,
                                   DX_PIXEL_COLOR_FILTER(color_filter), false);
            DX_VERIFY(status);

            FILE *image_file = fopen("/home/philipp/Desktop/test.ppm", "wb");
            fprintf(image_file, "P6\n%u %u 255\n", frame_buffer->nWidth, frame_buffer->nHeight);
            fwrite(rgb_image_buffer, 1, payload_size * 3, image_file);
            fclose(image_file);

            memcpy(image.data, rgb_image_buffer, frame_buffer->nWidth * frame_buffer->nHeight * 3);

            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

            cv::imshow("image", image);
        }

        if (cv::waitKey(1) == 27) {
            break;
        }

        status = GXQBuf(device, frame_buffer);
        GX_VERIFY(status);

    }


    return 0;

}

void GetErrorString(GX_STATUS emErrorStatus) {
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // Get length of error description
    emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
    if (emStatus != GX_STATUS_SUCCESS) {
        printf("<Error when calling GXGetLastError>\n");
        return;
    }

    // Alloc error resources
    error_info = new char[size];
    if (error_info == NULL) {
        printf("<Failed to allocate memory>\n");
        return;
    }

    // Get error description
    emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
    if (emStatus != GX_STATUS_SUCCESS) {
        printf("<Error when calling GXGetLastError>\n");
    } else {
        printf("%s\n", error_info);
    }

    // Realease error resources
    if (error_info != NULL) {
        delete[]error_info;
        error_info = NULL;
    }
}
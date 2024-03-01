#pragma once

#include <exception>
#include <GxIAPI.h>

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

class Camera {
private:
    GX_STATUS status = GX_STATUS_SUCCESS;
    GX_DEV_HANDLE device = nullptr;
public:
    Camera();

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
    void set_roi(int x, int y, int width, int height);

    /**
     * set the region of interest which is used for statistical processing within the camera
     * @param x
     * @param y
     * @param width
     * @param height
     */
    void set_statistical_area_roi(int x, int y, int width, int height);

    /**
     * set the region of interest for automatic white balance determination
     * @param x
     * @param y
     * @param width
     * @param height
     */
    void set_white_balance_roi(int x, int y, int width, int height);

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
     * deativates the gamma feature
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


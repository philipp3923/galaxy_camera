#include <optional>
#include <opencv2/opencv.hpp>
#include "Camera.hpp"

int main() {
    Camera cam = Camera();
    cam.set_statistical_area_roi(544, 148, 3024, 1788);
    cam.set_white_balance_roi(544, 148, 3024, 1788);
    cam.set_exposure_auto(11, 1 * 1000); // 1ms
    cam.set_white_balance(true);
    cam.set_gain_auto(0, 16);
    cam.set_gamma_auto();

    cam.set_trigger_source(CONTINUOUS);

    cam.start_capturing();

    cv::namedWindow("image", cv::WINDOW_NORMAL);

    while (true) {
        std::optional<Capture> image = cam.get_image(1000);

        if (image.has_value()) {
            cv::imshow("image", image.value().image);
        }

        if (cv::waitKey(1) == 27) {
            break;
        }
    }


}

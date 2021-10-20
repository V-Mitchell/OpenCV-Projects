/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*/
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

std::string gstreamerPipelineStr(int cap_width, int cap_height, int frame_rate, int cap_id) {
    return "nvarguscamerasrc sensor-id=" + std::to_string(cap_id)
        + " exposuretimerange='" + std::to_string(13000) + " " + std::to_string(100000) + "' aelock=false"
        + " gainrange='" + std::to_string(1) + " " + std::to_string(1) + "'"
        + " ispdigitalgainrange='" + std::to_string(1) + " " + std::to_string(1) + "'"
        + " aeantibanding=" + std::to_string(0) + " tnr-mode=" + std::to_string(1)
        + " tnr-strength=" + std::to_string(0.5) + " ee-mode=" + std::to_string(0)
        + " ee-strength=" +std::to_string(0.5)
        + " ! video/x-raw(memory:NVMM), width=" + std::to_string(cap_width)
        + ", height=" + std::to_string(cap_height) + ", format=(string)NV12, "
        + "framerate=(fraction)" + std::to_string(frame_rate) + "/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! "
        + "videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char* argv[]) {
    // Camera feed setup
    std::string pipeline = gstreamerPipelineStr(1280, 720, 20, 0);
    std::cout << pipeline << std::endl;
    cv::VideoCapture left_cam(pipeline, cv::CAP_GSTREAMER);
    pipeline = gstreamerPipelineStr(1280, 720, 20, 1);
    std::cout << pipeline << std::endl;
    cv::VideoCapture right_cam(pipeline, cv::CAP_GSTREAMER);

    if (!left_cam.isOpened()) {
        std::cout << "Couldn't open left camera" << std::endl;
        return -1;
    }
    if (!right_cam.isOpened()) {
        std::cout << "Couldn't open right camera" << std::endl;
        return -1;
    }

    while (true) {
        cv::Mat left_frame;
        cv::Mat right_frame;
        bool left_success = left_cam.read(left_frame);
        if (!left_success) {
            std::cout << "Left Camera Disconnected - Cannot read frame" << std::endl;
            break;
        }
        bool right_success = right_cam.read(right_frame);
        if (!right_success) {
            std::cout << "Right Camera Disconnected - Cannot read frame" << std::endl;
            break;
        }





        cv::imshow("left", left_frame);
        cv::imshow("right", right_frame);
        if (cv::waitKey(30) == 27) {
            std::cout << "Exiting..." << std::endl;
            break;
        }
    }

    return 0;
}
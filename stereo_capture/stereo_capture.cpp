/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*   Notes:  - Program for capturing calibration images from stereo camera
*           - Press 'Space' to capture an image and 'Esc' to exit program
*           - Captured image names are saved to xml file in path passed as cmd line argument
*/

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <stdio.h>

std::string gstreamerPipelineStr(int cap_width, int cap_height, int frame_rate, int cap_id) {
    return "nvarguscamerasrc sensor-id=" + std::to_string(cap_id)
        + " exposuretimerange='" + std::to_string(13000) + " " + std::to_string(100000000) + "' aelock=false"
        + " gainrange='" + std::to_string(1) + " " + std::to_string(2.5) + "'"
        + " ispdigitalgainrange='" + std::to_string(1) + " " + std::to_string(1) + "'"
        + " aeantibanding=" + std::to_string(0) + " tnr-mode=" + std::to_string(1)
        + " tnr-strength=" + std::to_string(0.5) + " ee-mode=" + std::to_string(0)
        + " ee-strength=" + std::to_string(0.5)
        + " ! video/x-raw(memory:NVMM), width=" + std::to_string(cap_width)
        + ", height=" + std::to_string(cap_height) + ", format=(string)NV12, "
        + "framerate=(fraction)" + std::to_string(frame_rate) + "/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! "
        + "videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char* argv[]) {
     // Print OpenCv build info
    std::cout << cv::getBuildInformation() << "\n" << std::endl;

    // camera feed setup
    std::string pipeline = gstreamerPipelineStr(1280, 720, 20, 0);
    std::cout << "// Left Cam Pipeline // " << pipeline << " //" << std::endl;
    cv::VideoCapture left_cam(pipeline, cv::CAP_GSTREAMER);
    pipeline = gstreamerPipelineStr(1280, 720, 20, 1);
    std::cout << "// Right Cam Pipeline // " << pipeline << " //" << std::endl;
    cv::VideoCapture right_cam(pipeline, cv::CAP_GSTREAMER);

    if (!left_cam.isOpened()) {
        std::cout << "Couldn't open left camera" << std::endl;
        return -1;
    }
    if (!right_cam.isOpened()) {
        std::cout << "Couldn't open right camera" << std::endl;
        return -1;
    }

    std::string xml_file = argv[1];
    xml_file += "calibration_images.xml";
    cv::FileStorage fs(xml_file, cv::FileStorage::WRITE);
    std::vector<std::string> l_file, r_file;

    cv::Mat left_frame, right_frame;
    int i = 0;
    while(true) {

        bool left_success = left_cam.read(left_frame);
        if (!left_success) {
            std::cout << "Left Camera Disconnected - Cannot read frame" << std::endl;
        }
        bool right_success = right_cam.read(right_frame);
        if (!right_success) {
            std::cout << "Right Camera Disconnected - Cannot read frame" << std::endl;
        }

        cv::imshow("Left Frame", left_frame);
        cv::imshow("Right Frame", right_frame);
        if (cv::waitKey(5) == 32) { // Press 'Space' to capture image
            i++;
            std::string file = argv[1];
            file += "left/";
            file += std::to_string(i);
            file += "_left_frame.png";
            std::cout << "Saving " << file << std::endl;
            cv::imwrite(file, left_frame);
            l_file.push_back(file);
            file = argv[1];
            file += "right/";
            file += std::to_string(i);
            file += "_right_frame.png";
            std::cout << "Saving " << file << std::endl;
            cv::imwrite(file, right_frame);
            r_file.push_back(file);
        } else if (cv::waitKey(5) == 27) { // Press 'Esc' to exit
            std::cout << "Exiting..." << std::endl;
            break;
        }
    }

    fs << "left_images" << l_file;
    fs << "right_images" << r_file;
    fs.release();

    return 0;
}
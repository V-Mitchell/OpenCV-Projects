/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*   Notes:  - Calculates a disparity map given calibration values passed via xml file
*           passed from cmd line argument
*           - StereoBM settings can be modifies in real time with track bars
*/

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <stdio.h>

cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();

// StereoBM parameters
int num_disp = 8;
int block_size = 5;
int pre_filter_type = 1;
int pre_filter_size = 1;
int pre_filter_cap = 31;
int min_disp = 0;
int texture_thresh = 10;
int uniqueness_ratio = 15;
int speckle_range = 0;
int speckle_window_size = 0;
int disp12_max_diff = -1;
int disp_type = CV_16S;

static void trackbar1(int, void*) {
    stereo->setNumDisparities(num_disp*16);
    num_disp = num_disp*16;
}
static void trackbar2(int, void*) {
    stereo->setBlockSize(block_size*2+5);
    block_size = block_size*2+5;
}
static void trackbar3(int, void*) {
    stereo->setPreFilterType(pre_filter_type);
}
static void trackbar4(int, void*) {
    stereo->setPreFilterSize(pre_filter_size*2+5);
    pre_filter_size = pre_filter_size*2+5;
}
static void trackbar5(int, void*) {
    stereo->setPreFilterCap(pre_filter_cap);
}
static void trackbar6(int, void*) {
    stereo->setTextureThreshold(texture_thresh);
}
static void trackbar7(int, void*) {
    stereo->setUniquenessRatio(uniqueness_ratio);
}
static void trackbar8(int, void*) {
    stereo->setSpeckleRange(speckle_range);
}
static void trackbar9(int, void*) {
    stereo->setSpeckleWindowSize(speckle_window_size*2);
    speckle_window_size = speckle_window_size*2;
}
static void trackbar10(int, void*) {
    stereo->setDisp12MaxDiff(disp12_max_diff);
}
static void trackbar11(int, void*) {
    stereo->setMinDisparity(min_disp);
}


std::string gstreamerPipelineStr(int cap_width, int cap_height, int frame_rate, int cap_id) {
    return "nvarguscamerasrc sensor-id=" + std::to_string(cap_id)
        + " exposuretimerange='" + std::to_string(13000) + " " + std::to_string(100000) + "' aelock=false"
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

// Stores relevent calibration values
class StereoCalibration {
private:
    cv::Mat l_cameraMat, l_distCoeffs;
    cv::Mat r_cameraMat, r_distCoeffs;
    cv::Mat Q;

public:
    StereoCalibration(std::string& xml_calib) {
        cv::FileStorage fs(xml_calib, cv::FileStorage::READ);
        fs["left_camera_matrix"] >> l_cameraMat;
        fs["right_camera_matrix"] >> r_cameraMat;
        fs["left_dist_coeffs"] >> l_distCoeffs;
        fs["right_dist_coeffs"] >> r_distCoeffs;
        fs["disparity_to_depth_matrix"] >> Q;
        fs.release();
    }

    void applyCalibration(cv::Mat& left_img, cv::Mat& right_img, cv::Mat& left_dst, cv::Mat& right_dst) {
        cv::undistort(left_img, left_dst, l_cameraMat, l_distCoeffs);
        cv::undistort(right_img, right_dst, r_cameraMat, r_distCoeffs);
    }

    void getQ(cv::Mat& q) {
        q = Q;
    }
};


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

    // Get stereo calibration data
    std::string xml_calib = argv[1];
    StereoCalibration calibrate(xml_calib);

    cv::namedWindow("settings", cv::WINDOW_NORMAL);
    // trackbar for configuring StereoBM
    cv::createTrackbar("numDisparities", "settings", &num_disp, 18, trackbar1);
    cv::createTrackbar("blockSize", "settings", &block_size, 50, trackbar2);
    cv::createTrackbar("preFilterType", "settings", &pre_filter_type, 1, trackbar3);
    cv::createTrackbar("preFilterSize", "settings", &pre_filter_size, 25, trackbar4);
    cv::createTrackbar("preFilterCap", "settings", &pre_filter_cap, 62, trackbar5);
    cv::createTrackbar("textureThreshold", "settings", &texture_thresh, 100, trackbar6);
    cv::createTrackbar("uniquenessRatio", "settings", &uniqueness_ratio, 100, trackbar7);
    cv::createTrackbar("speckleRange", "settings", &speckle_range, 100, trackbar8);
    cv::createTrackbar("speckleWindowSize", "settings", &speckle_window_size, 25, trackbar9);
    cv::createTrackbar("disp12MaxDiff", "settings", &disp12_max_diff, 25, trackbar10);
    cv::createTrackbar("minDisparity", "settings", &min_disp, 25, trackbar11);

    cv::Mat left_frame, right_frame;
    cv::Mat left_rFrame, right_rFrame;
    cv::Mat disparity, depth;
    cv::Mat Q;
    calibrate.getQ(Q);


    while (true) {
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

        cv::cvtColor(left_frame, left_frame, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_frame, right_frame, cv::COLOR_BGR2GRAY);
        calibrate.applyCalibration(left_frame, right_frame, left_rFrame, right_rFrame);

        // calculate disparity with StereoBM
        stereo->compute(left_rFrame, right_rFrame, disparity);
        cv::reprojectImageTo3D(disparity, depth, Q); // project disparity map to 3D image

        // conversions before display?
        //disparity.convertTo(disparity32F, CV_8U, 1.0f);

        double min, max;
        cv::minMaxIdx(disparity, &min, &max);
        cv::Mat adjDisparity;
        cv::convertScaleAbs(disparity, adjDisparity, 255 / max);

        cv::imshow("disparity", adjDisparity);
        if (cv::waitKey(30) == 27) {
            std::cout << "Exiting..." << std::endl;
            break;
        }
    }

    return 0;
}
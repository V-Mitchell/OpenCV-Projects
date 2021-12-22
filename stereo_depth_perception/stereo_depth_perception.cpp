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

std::string gstreamerPipelineStr(int cap_width, int cap_height, int frame_rate, int cap_id) {
    return "nvarguscamerasrc sensor-id=" + std::to_string(cap_id)
        + " exposuretimerange='" + std::to_string(13000) + " " + std::to_string(100000) + "' aelock=false"
        + " gainrange='" + std::to_string(1) + " " + std::to_string(5) + "'"
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
    cv::Mat l_rtR, l_rtP;
    cv::Mat r_rtR, r_rtP;
    cv::Mat Q;
    cv::Mat l_map1, l_map2;
    cv::Mat r_map1, r_map2;

public:
    StereoCalibration(std::string& xml_calib, cv::Size& size) {
        cv::FileStorage fs(xml_calib, cv::FileStorage::READ);
        fs["left_camera_matrix"] >> l_cameraMat;
        fs["right_camera_matrix"] >> r_cameraMat;
        fs["left_dist_coeffs"] >> l_distCoeffs;
        fs["right_dist_coeffs"] >> r_distCoeffs;
        fs["left_rectification_rotation_matrix"] >> l_rtR;
        fs["right_rectification_rotation_matrix"] >> r_rtR;
        fs["left_rectification_projection_matrix"] >> l_rtP;
        fs["right_rectification_projection_matrix"] >> r_rtP;
        fs["disparity_to_depth_matrix"] >> Q;
        fs.release();

        cv::Size new_size(size.height, size.width);
        cv::initUndistortRectifyMap(l_cameraMat, l_distCoeffs, l_rtR, l_rtP,
                                        new_size, CV_16SC2, l_map1, l_map2);
        cv::initUndistortRectifyMap(r_cameraMat, r_distCoeffs, r_rtR, r_rtP,
                                        new_size, CV_16SC2, r_map1, r_map2);
    }

    void applyCalibration(cv::Mat& left_img, cv::Mat& right_img, cv::Mat& left_dst, cv::Mat& right_dst) {
        //cv::undistort(left_img, left_dst, l_cameraMat, l_distCoeffs);
        //cv::undistort(right_img, right_dst, r_cameraMat, r_distCoeffs);

        cv::remap(left_img, left_dst, l_map1, l_map2, cv::INTER_LINEAR);
        cv::remap(right_img, right_dst, r_map1, r_map2, cv::INTER_LINEAR);
    }

    void getQ(cv::Mat& q) {
        q = Q;
    }
};


int main(int argc, char* argv[]) {
    // Print OpenCv build info
    std::cout << cv::getBuildInformation() << "\n" << std::endl;
    int width = 1280;
    int height = 720;
    cv::Size size(height, width);

    // camera feed setup
    std::string pipeline = gstreamerPipelineStr(width, height, 20, 0);
    std::cout << "// Left Cam Pipeline // " << pipeline << " //" << std::endl;
    cv::VideoCapture left_cam(pipeline, cv::CAP_GSTREAMER);
    pipeline = gstreamerPipelineStr(width, height, 20, 1);
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
    StereoCalibration calibrate(xml_calib, size);

    // Disparity Algorithm Parameters
    int block_size = 30;
    int min_disp = 0;
    int max_disp = 16;
    int num_disp = max_disp - min_disp;
    int uniqueness_ratio = 5;
    int speckle_range = 2;
    int speckle_window_size = 5;
    int disp12_max_diff = 2;
    
    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(min_disp, num_disp, block_size);
    //stereo->setUniquenessRatio(uniqueness_ratio);
    //stereo->setSpeckleWindowSize(speckle_window_size);
    //stereo->setDisp12MaxDiff(disp12_max_diff);
    //stereo->setP1(8 * 3 * block_size * block_size);
    //stereo->setP2(32 * 3 * block_size * block_size);


    cv::Mat left_frame, right_frame;
    cv::Mat left_rFrame, right_rFrame;
    cv::Mat disparity, depth;
    cv::Mat Q;
    calibrate.getQ(Q);


    while (true) {
        bool left_success = left_cam.read(left_frame);
        bool right_success = right_cam.read(right_frame);
        if (!left_success) {
            std::cout << "Left Camera Disconnected - Cannot read frame" << std::endl;
            break;
        }
        if (!right_success) {
            std::cout << "Right Camera Disconnected - Cannot read frame" << std::endl;
            break;
        }

        cv::cvtColor(left_frame, left_frame, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_frame, right_frame, cv::COLOR_BGR2GRAY);
        calibrate.applyCalibration(left_frame, right_frame, left_rFrame, right_rFrame);

        // calculate disparity with StereoBM
        stereo->compute(left_rFrame, right_rFrame, disparity);
        //cv::reprojectImageTo3D(disparity, depth, Q); // project disparity map to 3D image

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
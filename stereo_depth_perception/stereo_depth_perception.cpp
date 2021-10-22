/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*/
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/aruco.hpp>
#include <iostream>
#include <stdio.h>

cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();

cv::Mat left_frame, right_frame;
cv::Mat left_gray, right_gray;
// cv::Mat left_stereo_map1, left_stereo_map2, right_stereo_map1, right_stereo_map2;
cv::Mat disparity, disparity32F;
cv::Mat homography_left, homography_right;

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
        + " gainrange='" + std::to_string(1) + " " + std::to_string(1) + "'"
        + " ispdigitalgainrange='" + std::to_string(1) + " " + std::to_string(1) + "'"
        + " aeantibanding=" + std::to_string(0) + " tnr-mode=" + std::to_string(1)
        + " tnr-strength=" + std::to_string(0.5) + " ee-mode=" + std::to_string(0)
        + " ee-strength=" + std::to_string(0.5)
        + " ! video/x-raw(memory:NVMM), width=" + std::to_string(cap_width)
        + ", height=" + std::to_string(cap_height) + ", format=(string)NV12, "
        + "framerate=(fraction)" + std::to_string(frame_rate) + "/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! "
        + "videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

void stereoRectification(cv::Mat left_input, cv::Mat right_input, cv::Mat& left_rect, cv::Mat& right_rect) {
    cv::Ptr<cv::SIFT> sift_detector = cv::SIFT::create();
    std::vector<cv::KeyPoint> left_keypoints, right_keypoints;
    cv::Mat descriptor_left, descriptor_right;

    // convert input to gray
    cv::cvtColor(left_input, left_input, cv::COLOR_BGR2GRAY);
    cv::cvtColor(right_input, right_input, cv::COLOR_BGR2GRAY);

    // detect keypoints with sift detector
    sift_detector->detectAndCompute(left_input, cv::noArray(), left_keypoints, descriptor_left);
    sift_detector->detectAndCompute(right_input, cv::noArray(), right_keypoints, descriptor_right);

    // FLANN matcher
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector<std::vector<cv::DMatch> > knn_matches;
    matcher->knnMatch(descriptor_left, descriptor_right, knn_matches, 2);

    // filter matches using lowe's ratio
    const float ratio_thresh = 0.7f;
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::Point2f> pts_left;
    std::vector<cv::Point2f> pts_right;
    for (std::size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][0].distance) {
            good_matches.push_back(knn_matches[i][0]);
            pts_left.push_back(left_keypoints[knn_matches[i][0].queryIdx].pt);
            pts_right.push_back(right_keypoints[knn_matches[i][0].trainIdx].pt);
        }
    }

    // convert pts to int32?

    // get fundamental matrix
    std::vector<uchar> inliers;
    cv::Mat fundamental_mat = cv::findFundamentalMat(pts_left, pts_right, inliers, cv::FM_RANSAC);
    std::vector<cv::Point2f> inl_pts_left;
    std::vector<cv::Point2f> inl_pts_right;

    // select only inlier points
    for(std::size_t i = 0; i < pts_left.size(); i++) {
        if(inliers[i]) {
            inl_pts_left.push_back(pts_left[i]);
            inl_pts_right.push_back(pts_right[i]);
        }
    }

    cv::Mat H1;
    cv::Mat H2;
    cv::stereoRectifyUncalibrated(inl_pts_left, inl_pts_right, fundamental_mat, left_input.size(), H1, H2);
    homography_left = H1;
    homography_right = H2;
    cv::warpPerspective(left_input, left_rect, H1, left_input.size());
    cv::warpPerspective(right_input, right_rect, H2, right_input.size());
}



int main(int argc, char* argv[]) {
    // camera feed setup
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

    cv::namedWindow("disparity", cv::WINDOW_NORMAL);
    // trackbar for configuring StereoBM
    cv::createTrackbar("numDisparities", "disparity", &num_disp, 18, trackbar1);
    cv::createTrackbar("blockSize", "disparity", &block_size, 50, trackbar2);
    cv::createTrackbar("preFilterType", "disparity", &pre_filter_type, 1, trackbar3);
    cv::createTrackbar("preFilterSize", "disparity", &pre_filter_size, 25, trackbar4);
    cv::createTrackbar("preFilterCap", "disparity", &pre_filter_cap, 62, trackbar5);
    cv::createTrackbar("textureThreshold", "disparity", &texture_thresh, 100, trackbar6);
    cv::createTrackbar("uniquenessRatio", "disparity", &uniqueness_ratio, 100, trackbar7);
    cv::createTrackbar("speckleRange", "disparity", &speckle_range, 100, trackbar8);
    cv::createTrackbar("speckleWindowSize", "disparity", &speckle_window_size, 25, trackbar9);
    cv::createTrackbar("disp12MaxDiff", "disparity", &disp12_max_diff, 25, trackbar10);
    cv::createTrackbar("minDisparity", "disparity", &min_disp, 25, trackbar11);


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

        cv::cvtColor(left_frame, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_frame, right_gray, cv::COLOR_BGR2GRAY);

        cv::Mat left_rect, right_rect;

        // Apply rectification to both cameras (maybe only need to do this once and use homography for rectification)
        stereoRectification(left_gray, right_gray, left_rect, right_rect);
        
        // calculate disparition with StereoBM
        stereo->compute(left_rect, right_rect, disparity);

        // conversions before display?
        disparity.convertTo(disparity32F, CV_32F, 1.0f);

        //cv::imshow("left", left_frame);
        //cv::imshow("right", right_frame);
        cv::imshow("disparity", disparity32F);
        if (cv::waitKey(30) == 27) {
            std::cout << "Exiting..." << std::endl;
            break;
        }
    }

    return 0;
}
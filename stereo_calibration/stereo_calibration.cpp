/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
    Notes:  - Calculates camera calibration from stereo camera image pairs
            - File names of images given via xml file passed in third cmd line argument
            - Board size is passed in first two cmd line agruments
            - Calibration values saved to file passed in last cmd line argument
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

void imgCalibrationPts( cv::Mat& img, cv::Size board_size,
                        std::vector<cv::Point3f>& obj_pt,
                        std::vector<std::vector<cv::Point3f> >& obj_pts,
                        std::vector<std::vector<cv::Point2f> >& img_pts) {

    std::vector<cv::Point2f> corners; // stores detected corners of checkerboard
    bool found = cv::findChessboardCorners(img, board_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if (found) {
        cv::cornerSubPix(   img, corners, cv::Size(11, 11), cv::Size(-1, -1), 
                            cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

        cv::Mat draw = img;
        cv::drawChessboardCorners(draw, board_size, cv::Mat(corners), found);
        if (img.empty()) {
            std::cout << "Image Empty" << std::endl;
            return;
        }
        //cv::imshow("detected corners", draw);
        //cv::waitKey(); // wait till user presses key

        obj_pts.push_back(obj_pt);
        img_pts.push_back(corners);
    }

}

void applyStereoCalibration(std::string& xml_file, std::string& xml_calib, cv::Size& img_size) {
    cv::Mat l_map1, l_map2;
    cv::Mat r_map1, r_map2;
    //cv::Size new_size(img_size.height, img_size.width);

    cv::FileStorage fs(xml_calib, cv::FileStorage::READ);
    cv::Mat l_cameraMat, l_distCoeffs;
    cv::Mat r_cameraMat, r_distCoeffs;
    cv::Mat l_rtR, l_rtP;
    cv::Mat r_rtR, r_rtP;
    fs["left_camera_matrix"] >> l_cameraMat;
    fs["right_camera_matrix"] >> r_cameraMat;
    fs["left_dist_coeffs"] >> l_distCoeffs;
    fs["right_dist_coeffs"] >> r_distCoeffs;
    fs["left_rectification_rotation_matrix"] >> l_rtR;
    fs["right_rectification_rotation_matrix"] >> r_rtR;
    fs["left_rectification_projection_matrix"] >> l_rtP;
    fs["right_rectification_projection_matrix"] >> r_rtP;
    fs.release();

    cv::initUndistortRectifyMap(l_cameraMat, l_distCoeffs, l_rtR, l_rtP,
                                    img_size, CV_16SC2, l_map1, l_map2);
    cv::initUndistortRectifyMap(r_cameraMat, r_distCoeffs, r_rtR, r_rtP,
                                    img_size, CV_16SC2, r_map1, r_map2);

    fs.open(xml_file, cv::FileStorage::READ);
    std::vector<std::string> l_files, r_files;
    fs["left_images"] >> l_files;
    fs["right_images"] >> r_files;
    fs.release();
    cv::destroyAllWindows();

    for (std::size_t i = 0; i < l_files.size(); i++) {

        // Read stereo images
        cv::Mat left_img, right_img;
        cv::Mat left_rimg, right_rimg;
        left_img = cv::imread(l_files[i], cv::IMREAD_GRAYSCALE);
        right_img = cv::imread(r_files[i], cv::IMREAD_GRAYSCALE);

        // Check that images aren't empty
        if (left_img.empty()) {
            std::cout << "Image: " << l_files[i] << " Not Found" << std::endl;
            return;
        }
        if (right_img.empty()) {
            std::cout << "Image: " << r_files[i] << " Not Found" << std::endl;
            return;
        }
        // Check that image sizes match eachother
        if (left_img.size().width != right_img.size().width) {
            std::cout << "Images have unmacthed size." << std::endl;
            return;
        }
        if (left_img.size().height != right_img.size().height) {
            std::cout << "Images have unmacthed size." << std::endl;
            return;
        }

        cv::remap(left_img, left_rimg, l_map1, l_map2, cv::INTER_LINEAR);
        cv::remap(right_img, right_rimg, r_map1, r_map2, cv::INTER_LINEAR);

        cv::imshow("original", left_img);
        cv::imshow("remapped", left_rimg);
        cv::waitKey();
        cv::imshow("original", right_img);
        cv::imshow("remapped", right_rimg);
        cv::waitKey();
    }
}

void calibrateStereoCamera(std::string& xml_file, cv::Size& board_size, std::string& xml_output) {
    //Calibration points detected from calibration checkerboard
    std::vector<std::vector<cv::Point3f> > l_obj_pts, r_obj_pts;
    std::vector<std::vector<cv::Point2f> > l_img_pts, r_img_pts;

    std::vector<cv::Point3f> obj_pt;
    for(int i = 0; i < board_size.height; i++) {
        for(int j = 0; j < board_size.width; j++) {
            obj_pt.push_back(cv::Point3f(j, i, 0));
        }
    }

    cv::Size img_size; // Stores size of images being analyzed
    std::vector<std::string> l_files, r_files;
    // Read list of images from xml file
    cv::FileStorage fs(xml_file, cv::FileStorage::READ);
    fs["image_size"] >> img_size;
    fs["left_images"] >> l_files;
    fs["right_images"] >> r_files;
    fs.release();

    for (std::size_t i = 0; i < l_files.size(); i++) {

        // Read stereo images
        cv::Mat left_img, right_img;
        left_img = cv::imread(l_files[i], cv::IMREAD_GRAYSCALE);
        right_img = cv::imread(r_files[i], cv::IMREAD_GRAYSCALE);

        // Check that images aren't empty
        if (left_img.empty()) {
            std::cout << "Image: " << l_files[i] << " Not Found" << std::endl;
            return;
        }
        if (right_img.empty()) {
            std::cout << "Image: " << r_files[i] << " Not Found" << std::endl;
            return;
        }
        // Check that image sizes match eachother
        if (left_img.size().width != right_img.size().width) {
            std::cout << "Images have unmacthed size." << std::endl;
            return;
        }
        if (left_img.size().height != right_img.size().height) {
            std::cout << "Images have unmacthed size." << std::endl;
            return;
        }

        std::cout << "Analyzing " << l_files[i] << std::endl;
        std::cout << "Analyzing " << r_files[i] << std::endl;

        imgCalibrationPts(left_img, board_size, obj_pt, l_obj_pts, l_img_pts);
        imgCalibrationPts(right_img, board_size, obj_pt, r_obj_pts, r_img_pts);
    }

    if ((l_obj_pts.size() != r_obj_pts.size()) || (l_img_pts.size() != r_img_pts.size())) {
        std::cout << "Detected points aren't matching" << std::endl;
    }

    std::cout << "Calculating Individual Camera Calibration Data..." << std::endl;
    // Individual Camera Calibration //

    cv::Mat l_cameraMat, l_distCoeffs, l_R, l_T, l_stdDevInt, l_stdDevExt, l_viewErrors;
    cv::Mat r_cameraMat, r_distCoeffs, r_R, r_T, r_stdDevInt, r_stdDevExt, r_viewErrors;
    cv::calibrateCamera(l_obj_pts, l_img_pts, img_size, l_cameraMat, l_distCoeffs, l_R, l_T,
                        l_stdDevInt, l_stdDevExt, l_viewErrors);
    cv::calibrateCamera(r_obj_pts, r_img_pts, img_size, r_cameraMat, r_distCoeffs, r_R, r_T,
                        r_stdDevInt, r_stdDevExt, r_viewErrors);
    

    std::cout << "Calculating Stereo Camera Calibration Data..." << std::endl;
    // Stereo Rectification //

    // l_obj_pts = r_obj_pts
    cv::Mat R, T, E, F, viewErrors;
    cv::stereoCalibrate(l_obj_pts, l_img_pts, r_img_pts, l_cameraMat, l_distCoeffs, r_cameraMat, r_distCoeffs,
                        img_size, R, T, E, F, viewErrors, cv::CALIB_USE_INTRINSIC_GUESS +
                        cv::CALIB_FIX_PRINCIPAL_POINT + cv::CALIB_FIX_ASPECT_RATIO,
                        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));

    cv::Mat l_rtR, l_rtP;
    cv::Mat r_rtR, r_rtP;
    cv::Mat Q;
    cv::stereoRectify(l_cameraMat, l_distCoeffs, r_cameraMat, r_distCoeffs, img_size, R, T, l_rtR, r_rtR,
                        l_rtP, r_rtP, Q);

    std::cout << "Writing to " << xml_output << std::endl;
    // Save calibration values to xml file
    fs.open(xml_output, cv::FileStorage::WRITE);
    fs << "image_size" << img_size;
    fs << "left_camera_matrix" << l_cameraMat;
    fs << "right_camera_matrix" << r_cameraMat;
    fs << "left_dist_coeffs" << l_distCoeffs;
    fs << "right_dist_coeffs" << r_distCoeffs;
    fs << "left_rotation_matrix" << l_R;
    fs << "right_rotation_matrix" << r_R;
    fs << "left_translation_matrix" << l_T;
    fs << "right_translation_matrix" << r_T;
    fs << "left_stddev_intrinsic" << l_stdDevInt;
    fs << "right_stddev_intrinsic" << r_stdDevInt;
    fs << "left_stddev_extrinsic" << l_stdDevExt;
    fs << "right_stddev_extrinsic" << r_stdDevExt;
    fs << "left_per_view_errors" << l_viewErrors;
    fs << "right_per_view_errors" << r_viewErrors;
    fs << "stereo_rotation_matrix" << R;
    fs << "stereo_translation_matrix" << T;
    fs << "stereo_essential_matrix" << E;
    fs << "stereo_fundamental_matrix" << F;
    fs << "stereo_per_view_errors" << viewErrors;
    fs << "left_rectification_rotation_matrix" << l_rtR;
    fs << "right_rectification_rotation_matrix" << r_rtR;
    fs << "left_rectification_projection_matrix" << l_rtP;
    fs << "right_rectification_projection_matrix" << r_rtP;
    fs << "disparity_to_depth_matrix" << Q;
    fs.release();

    std::cout << "Applying Calibration Data..." << std::endl;
    applyStereoCalibration(xml_file, xml_output, img_size);
}

int main(int argc, char* argv[]) {
    // Print OpenCv build info
    std::cout << cv::getBuildInformation() << "\n" << std::endl;
    cv::Size board_size(std::stoi(argv[1]), std::stoi(argv[2]));
    std::string xml_file = argv[3];
    std::string xml_output = argv[4];
    
    calibrateStereoCamera(xml_file, board_size, xml_output);

    cv::waitKey();
    std::cout << "Exiting... " << std::endl;
    return 0;
}
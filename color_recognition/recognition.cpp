/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

int main(int argc, char* argv[]) {

    cv::VideoCapture capture(2);

    if (!capture.isOpened()) {
        std::cout << "Couldn't open camera" << std::endl;
        return -1;
    }

    double dWidth = capture.get(cv::CAP_PROP_FRAME_WIDTH); // get video frame width
    double dHeight = capture.get(cv::CAP_PROP_FRAME_HEIGHT); // get video frame height

    std::cout << "Video Resolution: " << dWidth << " x " << dHeight << std::endl;

    cv::namedWindow("Control", cv::WINDOW_AUTOSIZE); // Window called control

    int iLowH = 0;
    int iHighH = 179;
    int iLowS = 0;
    int iHighS = 255;
    int iLowV = 0;
    int iHighV = 255;

    cv::createTrackbar("Low H", "Control", &iLowH, 179);
    cv::createTrackbar("High H", "Control", &iHighH, 179);

    cv::createTrackbar("Low S", "Control", &iLowS, 255);
    cv::createTrackbar("High S", "Control", &iHighS, 255);

    cv::createTrackbar("Low V", "Control", &iLowV, 255);
    cv::createTrackbar("High V", "Control", &iHighV, 255);

    int iLastX = -1;
    int iLastY = -1;

    // Capture temporary image from camera
    cv::Mat imgTmp;
    capture.read(imgTmp);

    // Create black image with the size as the camera output
    cv::Mat imgLines = cv::Mat::zeros(imgTmp.size(), CV_8UC3);

    while(true) {
        cv::Mat imgOriginal;
        bool bSuccess = capture.read(imgOriginal);
        if (!bSuccess) {
            std::cout << "Camera Disconnected - Cannot read frame" << std::endl;
            break;
        }

        cv::Mat imgHSV;
        cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); // Convert capture frame from BGR to HSV

        cv::Mat imgThresholded;
        // Threshold the image
        cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);

        // morphological opening (remove small object from forwground)
        cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        // morphological closing (fill small holes from forwground)
        cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

        cv::Moments oMoments = cv::moments(imgThresholded);

        double dM01 = oMoments.m01;
        double dM10 = oMoments.m10;
        double dArea = oMoments.m00;

        if (dArea > 30000) {
            int posX = dM10/dArea;
            int posY = dM01/dArea;

            if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0) {
                // Draw line from point to last point
                cv::line(imgLines, cv::Point(posX, posY), cv::Point(iLastX, iLastY), cv::Scalar(0, 0, 255), 2);
            }
            std::cout << "XY: " << posX << " - " << posY << std::endl;
            iLastX = posX;
            iLastY = posY;
        }

        cv::imshow("Thresholded Image", imgThresholded);

        imgOriginal = imgOriginal + imgLines;
        cv::imshow("Original", imgOriginal);

        if (cv::waitKey(30) == 27) {
            std::cout << "Exiting..." << std::endl;
            break;
        }
    }

    return 0;
}
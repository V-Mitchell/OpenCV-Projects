/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*/

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

void setLabel(cv::Mat& im, const std::string label, const std::vector<cv::Point2f>& bounds) {
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.6;
    int thickness = 1;
    int baseline = 0;

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::Rect r = cv::boundingRect(bounds);

    cv::rectangle(im, r.tl(), r.br(), CV_RGB(0, 206, 209), thickness);
    cv::putText(im, label, r.br(), fontface, scale, CV_RGB(255,0,0), thickness, 8);
}

int main(int argc, char* argv[]) {
    // Load Aruco Dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // Initialize detector parameters (default)
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    // Vectors for containing detected and rejected markers
    std::vector<std::vector<cv::Point2f> > markers, rejectedMarkers;
    // IDs of detected markers
    std::vector<int> markerIDs;

    // Camera feed setup
    cv::VideoCapture capture(0);
    if (!capture.isOpened()) {
        std::cout << "Couldn't open camera" << std::endl;
        return -1;
    }
    double camWidth = capture.get(cv::CAP_PROP_FRAME_WIDTH); // get video frame width
    double camHeight = capture.get(cv::CAP_PROP_FRAME_HEIGHT); // get video frame height
    std::cout << "Video Resolution: " << camWidth << " x " << camHeight << std::endl;

    while (true) {
        cv::Mat frame;
        bool bSuccess = capture.read(frame);
        if (!bSuccess) {
            std::cout << "Camera Disconnected - Cannot read frame" << std::endl;
            break;
        }
        cv::Mat labeled = frame.clone();

        cv::aruco::detectMarkers(frame, dictionary, markers, markerIDs, parameters, rejectedMarkers);

        for (std::size_t i = 0; i < markers.size(); i++) {
            setLabel(labeled, std::to_string(markerIDs[i]), markers[i]);
        }

        cv::imshow("frame", frame);
        cv::imshow("labeled frame", labeled);
        if (cv::waitKey(30) == 27) {
            std::cout << "Exiting..." << std::endl;
            break;
        }
    }



    return 0;
}
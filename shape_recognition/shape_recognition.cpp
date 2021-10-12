/*  Author: Victor M
*   Email: victorcmitchell@gmail.com
*/

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour) {
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.4;
    int thickness = 1;
    int baseline = 0;

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::Rect r = cv::boundingRect(contour);

    cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
    cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), cv::FILLED);
    cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}


int main (int argc, char* argv[]) {

    cv::Mat img = cv::imread("/home/victor-m/Pictures/shapes.png");
    if (img.empty()) {
        std::cout << "Can't Read Image" << std::endl;
        return 0;
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // Use canny to catch squares with gradient shading
    cv::Mat bw;
    cv::Canny(gray, bw, 0, 50, 5);

    // Find Contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(bw.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> approx;
    cv::Mat dst = img.clone();

    for (std::size_t i = 0; i < contours.size(); i++) {
        // Approximate contour with accuracy proportional to contour perimeter
        cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(contours[i], true)*0.02, true);

        // skip small & non-convex objects
        if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx)) continue;

        if (approx.size() == 3) {
            setLabel(dst, "TRI", contours[i]);
        } else if (approx.size() >= 4 && approx.size() <= 6) {
            int vtc = approx.size(); // Number of verticies

            if (vtc == 4)
                setLabel(dst, "QUAD", contours[i]);
            else if (vtc == 5)
                setLabel(dst, "PENTA", contours[i]);
            else if (vtc ==6) 
                setLabel(dst, "HEXA", contours[i]);

        } else {
            // Detect circular objects
            double area = cv::contourArea(contours[i]);
            cv::Rect r = cv::boundingRect(contours[i]);
            int radius = r.width / 2;

            if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 && std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
                setLabel(dst, "CIR", contours[i]);

        }

        

    }
    cv::imshow("src", img);
    cv::imshow("dst", dst);
    cv::waitKey(0);
    return 0;
}
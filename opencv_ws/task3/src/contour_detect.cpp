#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv) {
    const std::string input = argc > 1 ? argv[1] : "images/official_input.jpg";
    const std::string output = argc > 2 ? argv[2] : "output/contours.png";
    const double canny_low = argc > 3 ? std::atof(argv[3]) : 35.0;
    const double canny_high = argc > 4 ? std::atof(argv[4]) : 105.0;

    const cv::Mat color = cv::imread(input, cv::IMREAD_COLOR);
    if (color.empty()) {
        std::cerr << "读不到图片: " << input << '\n';
        return 1;
    }

    cv::Mat gray;
    cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 1.5);

    cv::Mat edges;
    cv::Canny(gray, edges, canny_low, canny_high);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat canvas(color.size(), CV_8UC3, cv::Scalar(255, 255, 255));
    cv::drawContours(canvas, contours, -1, cv::Scalar(0, 0, 0), 1);

    cv::imwrite(output, canvas);

    std::cout << input << " -> " << output << "  轮廓 " << contours.size() << " 条\n";
    return 0;
}

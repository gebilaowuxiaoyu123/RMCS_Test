#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {

cv::Mat to_bgr(const cv::Mat& image) {
    cv::Mat out;
    if (image.channels() == 1)
        cv::cvtColor(image, out, cv::COLOR_GRAY2BGR);
    else
        out = image.clone();
    return out;
}

void put_label(cv::Mat& image, const std::string& text) {
    const cv::Point origin{12, 34};
    cv::putText(
        image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 0), 5, cv::LINE_AA);
    cv::putText(
        image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255, 255, 255), 2,
        cv::LINE_AA);
}

cv::Mat draw_contours_on(cv::Mat canvas, const std::vector<std::vector<cv::Point>>& contours) {
    canvas = canvas.clone();
    for (std::size_t i = 0; i < contours.size(); ++i) {
        const int blue = static_cast<int>((i * 37 + 60) % 256);
        const int green = static_cast<int>((i * 91 + 140) % 256);
        const int red = static_cast<int>((i * 53 + 220) % 256);
        cv::drawContours(
            canvas, contours, static_cast<int>(i), cv::Scalar(blue, green, red), 2, cv::LINE_AA);
    }
    return canvas;
}

cv::Mat stack_panel(
    const cv::Mat& top_left, const cv::Mat& top_right, const cv::Mat& bottom_left,
    const cv::Mat& bottom_right) {
    cv::Mat top, bottom, panel;
    cv::hconcat(top_left, top_right, top);
    cv::hconcat(bottom_left, bottom_right, bottom);
    cv::vconcat(top, bottom, panel);
    return panel;
}

} // namespace

int main(int argc, char** argv) {
    const std::string input_path = argc > 1 ? argv[1] : "images/fruits.jpg";
    const std::string output_prefix = argc > 2 ? argv[2] : "output/result";
    const double min_length = argc > 3 ? std::atof(argv[3]) : 40.0;

    const cv::Mat image = cv::imread(input_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "无法读取图片: " << input_path << '\n';
        return 1;
    }

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size{5, 5}, 1.5);

    cv::Mat edges;
    cv::Canny(blurred, edges, 60, 180);
    cv::morphologyEx(
        edges, edges, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edges, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    std::vector<std::vector<cv::Point>> kept;
    for (const auto& contour : contours) {
        if (cv::arcLength(contour, false) >= min_length)
            kept.push_back(contour);
    }

    cv::Mat contour_view =
        draw_contours_on(cv::Mat(image.size(), CV_8UC3, cv::Scalar(255, 255, 255)), kept);
    cv::Mat contour_overlay = draw_contours_on(image, kept);

    cv::Mat original_view = image.clone();
    cv::Mat gray_view = to_bgr(gray);
    cv::Mat edge_view = to_bgr(edges);

    put_label(original_view, "original");
    put_label(gray_view, "gray");
    put_label(edge_view, "canny edges");
    put_label(contour_view, "contours");

    cv::imwrite(output_prefix + "_gray.png", gray);
    cv::imwrite(output_prefix + "_edges.png", edges);
    cv::imwrite(output_prefix + "_contours.png", contour_view);
    cv::imwrite(output_prefix + "_overlay.png", contour_overlay);
    cv::imwrite(
        output_prefix + "_compare.png",
        stack_panel(original_view, gray_view, edge_view, contour_view));

    std::cout << "图片: " << input_path << " (" << image.cols << "x" << image.rows << ")\n";
    std::cout << "检出轮廓: " << contours.size() << " 条, 周长 > " << min_length << " 的保留 "
              << kept.size() << " 条\n";
    std::cout << "结果已写入: " << output_prefix << "_{gray,edges,contours,overlay,compare}.png\n";
    return 0;
}

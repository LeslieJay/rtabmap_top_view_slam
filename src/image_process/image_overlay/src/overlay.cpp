/*
 * @Autor: wei.canming
 * @Version: 1.0
 * @Date: 2025-03-14 15:04:26
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-04-01 15:19:52
 * @Description: 读取彩色图和对应的深度图，处理后再水平拼接
 */
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 读取彩色图和深度图
    cv::Mat color_image = cv::imread("/home/CameraTuyang_ws/src/image_saver/saved_images/color/color_20250212_052732_927.png");  // 彩色图（3通道）
    cv::Mat depth_image = cv::imread("/home/CameraTuyang_ws/src/image_saver/saved_images/depth/depth_20250212_052732_898.png", cv::IMREAD_UNCHANGED);  // 深度图（单通道）

    double maxVal;
    cv::minMaxLoc(depth_image, nullptr, &maxVal);
    std::cout<< maxVal << std::endl;
    // 检查图像是否成功加载
    if (color_image.empty() || depth_image.empty()) {
        std::cerr << "Error: Could not load images!" << std::endl;
        return -1;
    }
    cv::Mat resize_img;
    int target_width_ = 1920;
    int target_height_ = 1080;
    cv::resize(depth_image, resize_img, cv::Size(target_width_, target_height_));
    cv::imwrite("depth_image.png", resize_img);
    cv::minMaxLoc(depth_image, nullptr, &maxVal);
    std::cout<< maxVal << std::endl;

    // 确保两张图像尺寸相同
    if (color_image.size() != depth_image.size()) {
        std::cerr << "Error: Image sizes do not match!" << std::endl;
        return -1;
    }

    // 将深度图归一化到 0-255 范围
    cv::Mat depth_normalized;
    cv::normalize(depth_image, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::minMaxLoc(depth_normalized, nullptr, &maxVal);
    std::cout<< maxVal << std::endl;

    // 将深度图转换为伪彩色图（使用颜色映射）
    cv::Mat depth_colormap;
    cv::applyColorMap(depth_normalized, depth_colormap, cv::COLORMAP_JET);

    // 设置权重（alpha 和 beta）
    double alpha = 0.5;  // 彩色图的权重
    double beta = 0.5;    // 深度图的权重
    double gamma = 0;     // 可选的标量值

    // 合并彩色图和深度图
    cv::Mat overlay;
    cv::addWeighted(color_image, alpha, depth_colormap, beta, gamma, overlay);

    // 水平拼接彩色图、深度图的伪彩色图和合并后的图像
    cv::Mat result;
    cv::hconcat(std::vector<cv::Mat>{color_image, depth_colormap, overlay}, result);

    // 显示结果
    cv::imshow("Color Image | Depth Colormap | Overlay", result);
    cv::waitKey(0);

    // 保存结果
    cv::imwrite("overlay_result.jpg", result);

    return 0;
}
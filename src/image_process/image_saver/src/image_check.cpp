/*
 * @Autor: wei.canming
 * @Version: 1.0
 * @Date: 2025-01-20 11:26:47
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-04-01 14:54:53
 * @Description: 将深度图叠加到彩色图上，观察两者重合的部分
 */
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 加载彩色图和深度图
    cv::Mat colorImage = cv::imread("color.jpg");
    cv::Mat depthImage = cv::imread("depth.png", cv::IMREAD_GRAYSCALE);

    if (colorImage.empty() || depthImage.empty()) {
        std::cerr << "Error: Could not load images." << std::endl;
        return -1;
    }

    // 归一化深度图并应用颜色映射
    cv::Mat depthNormalized;
    cv::normalize(depthImage, depthNormalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::Mat depthColormap;
    cv::applyColorMap(depthNormalized, depthColormap, cv::COLORMAP_JET); // 热力图颜色

    // 合并彩色图和深度图
    double alpha = 0.6; // 控制叠加比例
    cv::Mat blendedImage;
    cv::addWeighted(colorImage, 1 - alpha, depthColormap, alpha, 0, blendedImage);

    // 显示或保存结果
    cv::imshow("Blended Image", blendedImage);
    cv::waitKey(0);
    cv::imwrite("blended_output.jpg", blendedImage);

    return 0;
}
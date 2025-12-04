#include <iostream>
#include <vector>
#include<opencv2/opencv.hpp>

int main() {
    // 创建一个存储cv::Mat的vector
    std::vector<cv::Mat> imageVector;
    
    // 添加几个图像到vector中
    cv::Mat img1 = cv::imread("image1.png", cv::IMREAD_COLOR);
    cv::Mat img2 = cv::imread("image2.png", cv::IMREAD_COLOR);
    
    if(img1.empty() || img2.empty()) {
        std::cerr << "Error loading images" << std::endl;
        return -1;
    }
    
    imageVector.push_back(img1);
    imageVector.push_back(img2);
    
    // 访问vector中的元素
    std::cout << "Vector size: " << imageVector.size() << std::endl;
    std::cout << "First image: " << imageVector[1].size()<< std::endl;
    
    // 遍历vector
    for(const auto& img : imageVector) {
        std::cout << "Image type: " << img.type() << std::endl;
    }
    
    return 0;
}
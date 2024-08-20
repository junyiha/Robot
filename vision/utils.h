#ifndef UTILS_H
#define UTILS_H
#include<string>
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>


//平滑边缘，去除锯齿
cv::Mat padding_hollow(cv::Mat mask, int ksize = 10);

// 计算两点之间的距离
double calculatePointsDistance(const cv::Point2f A, const cv::Point2f B);

//拟合直线
std::vector<float> fiting_line(std::vector<cv::Point2f> points);

// 计算多个点之间的欧式距离
double calculateDistance(const std::vector<float> A, const std::vector<float> B);

// 判断直线是否为板界线
bool check_is_boundary(cv::Mat img, cv::Vec4f line);


// 计算两条直线之间的距离
double calculateDistanceBetweenParallelLines(double slope, double intercept1, double intercept2);


//绘制直线
void draw_line(std::vector<cv::Vec4f> lines, cv::Mat& img);

// 绘制直线v2
void draw_line(std::vector<std::vector<float>> lines, cv::Mat& img);

cv::Mat rotateImage(const cv::Mat src);

//计算直方图相似度
double calculateHistogramSimilarity(const cv::Mat& img1, const cv::Mat& img2);
//比较直方图相似度
double compareHistograms(const cv::Mat& hist1, const cv::Mat& hist2) ;

// 检查直线是否为墨水线
bool check_is_ink_line(cv::Mat img, cv::Vec4f line);
//获取时间字符串
std::string getCurrentTimeAsString();

std::string getCurrentTimestampString();
#endif // UTILS_H

#ifndef UTILS_H
#define UTILS_H
#include <string>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <windows.h>
#include <filesystem>
using namespace std;

cv::Mat rotateImage(const cv::Mat src);

// 平滑边缘，去除锯齿
cv::Mat padding_hollow(cv::Mat mask, int ksize = 10);

// 计算两点之间的距离
double calculatePointsDistance(const cv::Point2f A, const cv::Point2f B);

// 拟合直线
std::vector<float> fiting_line(std::vector<cv::Point2f> points);

void sort_lines(std::vector<std::vector<float>> &lines, bool ascending = true); //  针对直线进行排序, 默认情况下，自上到下

void sort_lines(std::vector<cv::Vec4f> &lines, bool ascending = true); //  针对直线进行排序, 默认情况下，自上到下

bool compareLines(const std::vector<float> &a, const std::vector<float> &b, bool ascending = true); // 根据直线的y坐标，比较两条直线在上下次序

bool lineIsOverlapping(cv::Vec4f a, cv::Vec4f b, double tolerance = 30);

std::vector<std::vector<float>> get_line_by_lsd(cv::Mat bin_img);
std::vector<std::vector<float>> linesAggregation(std::vector<std::vector<float>> lines);
bool check_is_border_line_by_hist(cv::Mat img, cv::Vec4f line, float thr);

std::string getCurrentTimestampString();

// 计算多个点之间的欧式距离
double calculateDistance(const std::vector<float> A, const std::vector<float> B);

// 判断直线是否为板界线
bool check_is_boundary(cv::Mat img, cv::Vec4f line);

// 绘制直线
void draw_line(std::vector<cv::Vec4f> lines, cv::Mat &img);

// 绘制直线v2
void draw_line(std::vector<std::vector<float>> lines, cv::Mat &img);

cv::Mat rotateImage(const cv::Mat src);

// 获取时间字符串
std::string getCurrentTimeAsString();

std::string generateFilename();

int get_index_by_string(std::string key_str);

void save_images(cv::Mat img, std::string data_root);

bool saveMatWithCurrentTime(const cv::Mat &mat, const std::string goc_name, const std::string &folderPath = "./gocator");

bool check_is_border_line(cv::Mat img, cv::Vec4f line, float threshold = 0.6);
// 计算直方图相似度
double calculateHistogramSimilarity(const cv::Mat &img1, const cv::Mat &img2);
// 比较直方图相似度
double compareHistograms(const cv::Mat &hist1, const cv::Mat &hist2);
double perpendicularDistance(double A, double B, double C, double x1, double y1);
double computePointToLineDistance(std::vector<float> line_1, std::vector<float> line_2);

void image_correction(cv::Mat &img, unsigned rotate_type);

#endif // UTILS_H

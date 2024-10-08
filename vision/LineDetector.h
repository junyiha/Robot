//
// Created by csh_i on 2024/7/5.
//

#ifndef CMAKE_TEST_LINEDETECTOR_H
#define CMAKE_TEST_LINEDETECTOR_H
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "mlsd.h"
#include "utils.h"
#include "Parameters.h"







class LineDetector {

public:

    unsigned int resize_w = 512;
    unsigned int resize_h = 512;


    MLSD mlsd;
    LineDetector();
    ~LineDetector();


    // 获取参考线
    MLine getReferenceLine(cv::Mat img, std::vector<MLine> lines);
    // 获取墨线
    MLine getInkLine(cv::Mat img, std::vector<MLine> lines, MLine refLine);
    // 获取参考线与墨线的距离
    LineResult getLineDistance(cv::Mat img);
    // 获取所有可能的直线
    std::vector<MLine> getAllPossibleLines(cv::Mat img);
    // 获取参考线 LSD算法
    MLine LineDetector::getReferenceLineLSD(cv::Mat img, cv::Mat bin_img);

    MLine getReferenceLineByContours(cv::Mat bin_img);




    unsigned  int  min_len_seg;

    unsigned  int min_pos_valid;
    unsigned  int max_pos_valid ;

    unsigned  int min_dist_lines ;
    unsigned  int max_dist_lines ;
    unsigned  int max_angle_degrees ;






};


#endif //CMAKE_TEST_LINEDETECTOR_H

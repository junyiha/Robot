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


    // ��ȡ�ο���
    MLine getReferenceLine(cv::Mat img, std::vector<MLine> lines);
    // ��ȡī��
    MLine getInkLine(cv::Mat img, std::vector<MLine> lines, MLine refLine);
    // ��ȡ�ο�����ī�ߵľ���
    LineResult getLineDistance(cv::Mat img);
    // ��ȡ���п��ܵ�ֱ��
    std::vector<MLine> getAllPossibleLines(cv::Mat img);
    // ��ȡ�ο��� LSD�㷨
    MLine LineDetector::getReferenceLineLSD(cv::Mat img, cv::Mat bin_img);

    MLine getReferenceLineByContours(cv::Mat bin_img);

    MLine getReferenceLineByHKCU60(cv::Mat img, std::vector<MLine> lines);




    unsigned  int  min_len_seg;

    unsigned  int min_pos_valid;
    unsigned  int max_pos_valid ;

    unsigned  int min_dist_lines ;
    unsigned  int max_dist_lines ;
    unsigned  int max_angle_degrees ;






};


#endif //CMAKE_TEST_LINEDETECTOR_H

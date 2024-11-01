//
// Created by csh_i on 2024/10/31.
//

#ifndef PDROBOT_MASTER_CAMERACALIBRATIONLINE_H
#define PDROBOT_MASTER_CAMERACALIBRATIONLINE_H
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>

struct CalibrationLine {
    bool status;
    std::vector<cv::Point2f> line;
    cv::Mat imageDrawd;
    double scale;
};

class CameraCalibrationLine {

public:
    CameraCalibrationLine();
    ~CameraCalibrationLine();

    bool checkDataIsValid(cv::Mat image);

    CalibrationLine findCalibrationLine(cv::Mat image);
private:
    const int RectangleWidth = 50; //ToDo: 根据实际长度修改；


};


#endif //PDROBOT_MASTER_CAMERACALIBRATIONLINE_H

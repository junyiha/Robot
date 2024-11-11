//
// Created by csh_i on 2024/10/21.
//

#ifndef MLSD_TEST_LINEDETECTOR_H
#define MLSD_TEST_LINEDETECTOR_H
//#include "LineSegmenationBase.h"
#include "LineSegmenationBaseTensorRT.h"
#include "utils.h"

struct LineSpaceResult {
    float dist;
    bool status;
    bool ink_line_status;
    bool border_line_status;
    std::string error_info; //错误信息
    std::vector<float> ink_res;
    std::vector<float> border_res;
    cv::Mat img_drawed;
};




class LineDetector {
public:
    LineSegmenationBaseTensorRT lineSegModel;
    LineDetector();
    ~LineDetector();

    int imgInputW = 768;
    int imgInputH = 768;

    // get masks of ink and border
    std::vector<cv::Mat> getMasks(cv::Mat img);
    std::vector<float> getRerferenceLine(cv::Mat refImg, cv::Mat inputImage);
    std::vector<float> getInkLine(cv::Mat img, std::vector<float> referenceLine);
    LineSpaceResult getLinesDistance(cv::Mat img);
    void getPossibleLinesFromMask(cv::Mat mask, vector<std::vector<cv::Point2f>> &possibleLines);
    std::vector<float> LineDetector::getReferenceLineLSD(cv::Mat img, cv::Mat bin_img);
    std::vector<float> getReferenceLineFromMask(cv::Mat mask,cv::Mat bin_img);

};


#endif //MLSD_TEST_LINEDETECTOR_H

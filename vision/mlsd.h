#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <string>
#include <random>
#include <vector>
#include <onnxruntime_cxx_api.h>
#include <QDebug>
// using namespace Ort;

// 定义网络结构体
struct Net_config
{
    float score_thr;        //  直线得分阈值
    float dist_thr;         //  距离得分阈值
    std::string model_path; // onnx 文件存放地址
};

// 定义MLSD模型类
class MLSD
{

public:
    MLSD();                                              // 模型初始化
    std::vector<std::vector<float>> detect(cv::Mat img); // MLSD检测方法入口

private:
    int inpWidth;
    int inpHeight;
    float score_thr;
    float dist_thr;

    cv::Mat img_processing(cv::Mat img);                          // 图像预处理类
    std::vector<std::vector<float>> post_processing(cv::Mat res); // 预测结果后处理

    // ONNXRUNTIME 模型配置文件
    Ort::Env env = Ort::Env(ORT_LOGGING_LEVEL_ERROR, "MLSD");
    Ort::Session *ort_session = nullptr;
    Ort::SessionOptions sessionOptions = Ort::SessionOptions();
    std::vector<char *> input_names; // 输入的节点名称
    std::vector<char *> output_names;
    std::vector<std::vector<int64_t>> input_node_dims;  // >=1 outputs
    std::vector<std::vector<int64_t>> output_node_dims; // >=1 outputs
};

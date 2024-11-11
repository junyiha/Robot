//
// Created by csh_i on 2024/8/15.
//


#pragma once
#include <opencv2/opencv.hpp>
#include<opencv2/dnn.hpp>
#include <iostream>
#include <string>
#include <random>
#include<vector>
#include <onnxruntime_cxx_api.h>


namespace OnnxProviders {
    inline const std::string CPU = "cpu";
    inline const std::string CUDA = "cuda";
}


class LineSegmenationBase {
    // 初始化模型
public:
    LineSegmenationBase();
    ~LineSegmenationBase();

    void init_model();
    cv::Mat get_input_tensor(cv::Mat img);
    cv::Mat predict(cv::Mat img);
    cv::Mat post_process(cv::Mat res);
    void imageEnhance(cv::Mat &img);




private:
    int inpWidth;
    int inpHeight;
    int outputWidth;
    int outputHeight;
    int outputChannels;
    //ONNXRUNTIME 模型配置文件
    Ort::Env env = Ort::Env(ORT_LOGGING_LEVEL_ERROR, "PP-LiteSeg");
    const std::string &onnx_provider = OnnxProviders::CPU;
    Ort::Session *ort_session = nullptr;
    Ort::SessionOptions sessionOptions = Ort::SessionOptions();
    std::vector<char*> input_names; // 输入的节点名称
    std::vector<char*> output_names;
    std::vector<std::vector<int64_t>> input_node_dims; // >=1 outputs
    std::vector<std::vector<int64_t>> output_node_dims; // >=1 outputs

    cv::Mat LineSegmenationBase::computeArgmax(const cv::Mat& input);

    void Normalize(cv::Mat src, cv::Mat &dst);
};



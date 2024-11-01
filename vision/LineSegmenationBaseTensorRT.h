//
// Created by Administrator on 2024/10/29.
//

#ifndef MLSD_TEST_LINESEGMENATIONBASETENSORRT_H
#define MLSD_TEST_LINESEGMENATIONBASETENSORRT_H
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "logging.h"

#define DEVICE 0  // GPU id
#define CHECK(status) \
    do\
    {\
        auto ret = (status);\
        if (ret != 0)\
        {\
            std::cerr << "Cuda failure: " << ret << std::endl;\
            abort();\
        }\
    } while (0)




class LineSegmenationBaseTensorRT {

public:

    LineSegmenationBaseTensorRT();
    ~LineSegmenationBaseTensorRT();

    nvinfer1::ICudaEngine* loadEngine(const std::string& engineFilePath);

    nvinfer1::IExecutionContext* createExecutionContext(nvinfer1::ICudaEngine* engine);

    float* preprocessImage(cv::Mat& image, int inputHeight, int inputWidth);

    void infer(nvinfer1::IExecutionContext* context, float* inputBuffer, float* outputBuffer,\
               int batchSize, int inputHeight, int inputWidth, int outputSize);

    void doInference(nvinfer1::IExecutionContext& context, float* input, float* output, const int output_size, cv::Size input_shape);

    cv::Mat predict(cv::Mat inputImg);

    cv::Mat LineSegmenationBaseTensorRT::computeArgmax(const cv::Mat& input);

    cv::Mat LineSegmenationBaseTensorRT::post_process(cv::Mat res);

    float* blobFromImage(cv::Mat& img);



private:
//    const std::string engineFilePath= "../models/pp_liteseg_stdc1_softmax_20241021.engine";
    const std::string engineFilePath= VISION_ENGINE_PATH;
    Logger gLogger;
    int inputHeight;
    int inputWidth;
    int outputHeight;
    int outputWidth;
    int outputSize;


    const char* INPUT_BLOB_NAME = "x";
    const char* OUTPUT_BLOB_NAME = "save_infer_model/scale_0.tmp_0";
    nvinfer1::ICudaEngine* cudaEngine;
    nvinfer1::IExecutionContext * context;

};


#endif //MLSD_TEST_LINESEGMENATIONBASETENSORRT_H

/**
 *
*/
#pragma once

#include "common.hpp"

#ifdef GPU_FLAG

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>

#include "vision/logging.h"

namespace Infer
{
    class DetectorOnGPU
    {
    public:
        DetectorOnGPU();
        ~DetectorOnGPU();
        void InferOnce(std::wstring model_path, cv::Mat image);

    private:
        nvinfer1::ICudaEngine* LoadEngine(std::string engine_path);
        std::vector<float> Preprocess(cv::Mat& image, int input_height, int input_width);
        cv::Mat Predict(nvinfer1::Dims output_dims, nvinfer1::IExecutionContext* context, std::vector<float>& input, std::vector<float>& blob, cv::Size input_shape, int output_height, int output_width, int output_size);
        std::vector<std::vector<cv::Point2f>> GetPossibleLines(const cv::Mat& mask);
        bool ProcessReferMask(cv::Mat referMask, cv::Mat image_gray, std::vector<float>& line_res);
        bool ProcessInkMask(cv::Mat inkMask, cv::Mat image_gray, const std::vector<float>& refer_line, std::vector<float>& line_res);
        bool Postprocess(cv::Mat res, cv::Mat image_gray, std::vector<float>& refer_line, std::vector<float>& ink_line, int output_height, int output_width);

    private:
        Logger gLogger;
        const char* INPUT_BLOB_NAME = "x";
        const char* OUTPUT_BLOB_NAME = "save_infer_model/scale_0.tmp_0";
        int m_width;
        int m_height;
    };
}  // namespace Infer

#endif // GPU_FLAG
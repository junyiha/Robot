/**
 * @file onnxruntime.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-12-30
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "common.hpp"

namespace Infer
{
    class Detector
    {
    public:
        Detector();
        ~Detector();
        void InferOnce(std::wstring model_path, cv::Mat image);

    private:
        void Init(const std::wstring& model_path);
        void Preprocess(const cv::Mat& img, const std::vector<int64_t>& input_shape, cv::Mat& rgb_img, cv::Mat& gray_img, cv::Mat& blob);
        bool Infer(cv::Mat blob, std::vector<Ort::Value>& output_tensors);
        bool ProcessReferMask(cv::Mat referMask, cv::Mat image_gray, std::vector<float>& line_res);
        bool ProcessInkMask(cv::Mat inkMask, cv::Mat image_gray, const std::vector<float>& refer_line, std::vector<float>& line_res);
        bool Postprocess(std::vector<Ort::Value>& output_tensors, cv::Mat image_gray, std::vector<float>& refer_line, std::vector<float>& ink_line);
        std::vector<std::vector<cv::Point2f>> GetPossibleLines(const cv::Mat& mask);

    private:
        Context_t m_ctx;
        int m_height{ 0 };
        int m_width{ 0 };
        std::map<std::string, Context_t> m_detector_map;
    };
}

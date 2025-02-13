/**
 * @file common.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-02-13
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "utils/basic_header.hpp"
#include "onnxruntime_cxx_api.h"

namespace Infer
{
    struct Context_t
    {
        Ort::Env* env;
        Ort::Session* session;
        std::vector<char*> input_names;
        std::vector<char*> output_names;
        std::vector<int64_t> input_shape;
        std::vector<int64_t> output_shape;
    };

    std::vector<float> FitLine(std::vector<cv::Point2f> points);

    double CaculateDistance(const cv::Point2f& point1, const cv::Point2f& point2);

}  // namespace Infer
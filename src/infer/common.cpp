#include "common.hpp"

namespace Infer
{
    std::vector<float> FitLine(std::vector<cv::Point2f> points)
    {
        cv::Vec4f lines;
        cv::fitLine(points, lines, cv::DIST_L2, 0, 0.01, 0.01);
        float k = lines[1] / lines[0];
        float lefty = (-lines[2] * k) + lines[3];
        float righty = ((768 - 1 - lines[2]) * k) + lines[3];

        return { 0.0, lefty, 768.0, righty, k };
    }

    double CaculateDistance(const cv::Point2f& point1, const cv::Point2f& point2)
    {
        return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
    }
}  // namespace Infer
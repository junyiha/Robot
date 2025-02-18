#include "onnxruntime.hpp"

namespace Infer
{
    DetectorOnCPU::DetectorOnCPU()
    {

    }

    DetectorOnCPU::~DetectorOnCPU()
    {

    }

    std::vector<std::vector<cv::Point2f>> DetectorOnCPU::GetPossibleLines(const cv::Mat& mask)
    {
        std::vector<std::vector<cv::Point2f>> possible_lines;
        cv::GaussianBlur(mask, mask, cv::Size(5, 5), 0);
        cv::threshold(mask, mask, 0, 255, cv::THRESH_OTSU);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty())
        {
            std::cerr << "contours empty\n";
            return possible_lines;
        }
        for (int i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours.at(i));
            if (area < 768 * 0.2)
            {
                continue;
            }

            cv::RotatedRect min_rect = cv::minAreaRect(contours.at(i));
            cv::Mat points;
            std::vector<cv::Point2f> rect_points;
            cv::boxPoints(min_rect, points);

            for (int j = 0; j < points.rows; j++)
            {
                rect_points.push_back(points.at<cv::Point2f>(j, 0));
            }

            for (int j = 0; j < rect_points.size() - 1; j++)
            {
                for (int k = 0; k < rect_points.size() - j - 1; k++)
                {
                    if (rect_points.at(k).x > rect_points.at(k + 1).x)
                    {
                        cv::Point temp = rect_points.at(k);
                        rect_points.at(k) = rect_points.at(k + 1);
                        rect_points.at(k + 1) = temp;
                    }
                }
            }

            cv::Point2f point_left = cv::Point((rect_points.at(0).x + rect_points.at(1).x) / 2, (rect_points.at(0).y + rect_points.at(1).y) / 2);
            cv::Point2f point_right = cv::Point((rect_points.at(2).x + rect_points.at(3).x) / 2, (rect_points.at(2).y + rect_points.at(3).y) / 2);
            possible_lines.push_back({ point_left, point_right });
        }

        return possible_lines;
    }

    void DetectorOnCPU::Init(const std::wstring& model_path)
    {
        Ort::Env* env = new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "test");
        Ort::SessionOptions session_options;
        Ort::Session* session = new Ort::Session(*env, model_path.c_str(), session_options);

        m_ctx.env = env;
        m_ctx.session = session;
        m_ctx.input_shape = session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        m_ctx.output_shape = session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        Ort::AllocatorWithDefaultOptions allocator;
        m_ctx.input_names.push_back(session->GetInputName(0, allocator));
        m_ctx.output_names.push_back(session->GetOutputName(0, allocator));
    }

    void DetectorOnCPU::Preprocess(const cv::Mat& img, const std::vector<int64_t>& input_shape, cv::Mat& rgb_img, cv::Mat& gray_img, cv::Mat& blob)
    {
        if (img.channels() != 3)
        {
            cv::cvtColor(img, rgb_img, cv::COLOR_GRAY2BGR);
            gray_img = img.clone();
        }
        else
        {
            rgb_img = img.clone();
            cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
        }

        m_height = input_shape.at(2);
        m_width = input_shape.at(3);
        cv::resize(rgb_img, rgb_img, cv::Size(m_width, m_height));
        cv::resize(gray_img, gray_img, cv::Size(m_width, m_height));
        rgb_img.convertTo(rgb_img, CV_32F);
        rgb_img = ((rgb_img / 255.0f) - 0.5f) / 0.5f;

        blob = cv::dnn::blobFromImage(rgb_img, 1.0, cv::Size(m_width, m_height), cv::Scalar(0, 0, 0), true, false);
    }

    bool DetectorOnCPU::Infer(cv::Mat blob, std::vector<Ort::Value>& output_tensors)
    {
        std::vector<Ort::Value> input_tensors;
        auto allocator_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        input_tensors.emplace_back(Ort::Value::CreateTensor<float>(allocator_info, blob.ptr<float>(), blob.total(), m_ctx.input_shape.data(), m_ctx.input_shape.size()));

        output_tensors = m_ctx.session->Run(Ort::RunOptions{ nullptr }, &m_ctx.input_names.at(0), input_tensors.data(), input_tensors.size(), m_ctx.output_names.data(), m_ctx.output_names.size());
        if (output_tensors.size() != m_ctx.session->GetOutputCount())
        {
            std::cerr << "output tensors size not equal to num_output_nodes\n";
            return false;
        }
        if (!output_tensors.front().IsTensor())
        {
            std::cerr << "output tensor is not a tensor\n";
            return false;
        }

        return true;
    }

    bool DetectorOnCPU::ProcessReferMask(cv::Mat referMask, cv::Mat image_gray, std::vector<float>& line_res)
    {
        auto possible_lines = GetPossibleLines(referMask);

        if (possible_lines.empty())
        {
            std::cerr << "possibleLines empty\n";
            return false;
        }

        double max_dist = 0;
        int max_index = 0;
        for (int i = 0; i < possible_lines.size(); i++)
        {
            double dist_i = possible_lines.at(i).at(0).y > possible_lines.at(i).at(1).y ? possible_lines.at(i).at(0).y : possible_lines.at(i).at(1).y;

            CaculateDistance(possible_lines.at(i).at(0), possible_lines.at(i).at(1));

            if (dist_i < 768 * 0.15)
            {
                continue;
            }

            if (possible_lines.at(i).at(0).y < 768 * 0.5)
            {
                continue;
            }

            cv::Point2f p1((possible_lines.at(i).at(0).x + possible_lines.at(i).at(1).x) / 2, (possible_lines.at(i).at(0).y + possible_lines.at(i).at(1).y) / 2);
            if (p1.x + 30 > m_width || p1.y + 30 > m_height)
            {
                continue;
            }

            cv::Rect rect(p1.x, p1.y, 30, 30);
            cv::Mat roi = image_gray(rect);
            double mean_color = cv::mean(roi)[0];
            if (mean_color > 180)
            {
                continue;
            }

            if (dist_i > max_dist)
            {
                max_dist = dist_i;
                max_index = i;
            }
        }

        if (max_index == -1)
        {
            return false;
        }

        line_res = FitLine(possible_lines.at(max_index));

        if (line_res.empty())
        {
            std::cerr << "line_res empty\n";
            return false;
        }

        bool result = std::all_of(line_res.begin(), line_res.end() - 1, [](float val) { return val >= 0.0; });
        if (!result)
        {
            std::cerr << "line_res invalid\n";
            return false;
        }

        return true;
    }

    bool DetectorOnCPU::ProcessInkMask(cv::Mat inkMask, cv::Mat image_gray, const std::vector<float>& refer_line, std::vector<float>& line_res)
    {
        auto possible_lines = GetPossibleLines(inkMask);
        double dgree_ref = atan((refer_line.at(3) - refer_line.at(1)) / (refer_line.at(2) - refer_line.at(0))) * 180 / CV_PI;

        double min_dist = 100000;
        int min_index = -1;
        double max_dist = 0;
        int max_index = 0;
        for (int i = 0; i < possible_lines.size(); i++)
        {
            double dist_i = CaculateDistance(possible_lines.at(i).at(0), possible_lines.at(i).at(1));
            if (dist_i < 768 * 0.15)
            {
                continue;
            }

            double max_ref_y = refer_line.at(1) > refer_line.at(3) ? refer_line.at(1) : refer_line.at(3);
            double max_ink_y = possible_lines.at(i).at(0).y < possible_lines.at(i).at(1).y ? possible_lines.at(i).at(1).y : possible_lines.at(i).at(0).y;
            if (max_ink_y > max_ref_y)
            {
                continue;
            }

            double slope_i = (possible_lines.at(i).at(1).y - possible_lines.at(i).at(0).y) / (possible_lines.at(i).at(1).x - possible_lines.at(i).at(0).x);
            double dgree_i = atan(slope_i) * 180 / CV_PI;
            if (std::abs(dgree_i - dgree_ref) > 10)
            {
                continue;
            }

            if (dist_i > max_dist)
            {
                max_dist = dist_i;
                max_index = i;
            }

            double ref_ink_dist_y = std::abs(max_ink_y - max_ref_y);
            if (min_dist > ref_ink_dist_y)
            {
                min_dist = ref_ink_dist_y;
                min_index = i;
            }
        }

        if (min_index == -1)
        {
            std::cerr << "min_index invalid\n";
            return false;

        }

        line_res = FitLine(possible_lines.at(min_index));

        if (line_res.empty())
        {
            std::cerr << "line_res empty\n";
            return false;
        }

        bool result = std::all_of(line_res.begin(), line_res.end() - 1, [](float val) { return val >= 0.0; });
        if (!result)
        {
            std::cerr << "line_res invalid\n";
            return false;
        }

        return true;
    }

    bool DetectorOnCPU::Postprocess(std::vector<Ort::Value>& output_tensors, cv::Mat image_gray, std::vector<float>& refer_line, std::vector<float>& ink_line)
    {
        auto output_shape = output_tensors.front().GetTensorTypeAndShapeInfo().GetShape();
        std::vector<int> mask_sz(output_shape.begin(), output_shape.end());
        cv::Mat res(mask_sz, CV_32F, output_tensors.front().GetTensorMutableData<float>());

        res = res.reshape(0, { static_cast<int>(output_shape.at(1)), static_cast<int>(output_shape.at(2) * output_shape.at(3)) }).t();
        res = res.reshape(static_cast<int>(output_shape.at(1)), { static_cast<int>(output_shape.at(2)), static_cast<int>(output_shape.at(3)) });

        cv::Mat argmax_image(res.rows, res.cols, CV_8UC1);

        for (int row = 0; row < res.rows; row++)
        {
            for (int col = 0; col < res.cols; col++)
            {
                float* ptr = const_cast<float*>(res.ptr<float>(row, col));
                int max_idx{ 0 };
                float max_value = ptr[0];
                for (int i = 1; i < res.channels(); i++)
                {
                    if (ptr[i] > max_value)
                    {
                        max_value = ptr[i];
                        max_idx = i;
                    }
                }
                argmax_image.at<uchar>(row, col) = max_idx;
            }
        }

        double mask_sum = cv::sum(argmax_image)[0];
        if (mask_sum < 100)
        {
            std::cerr << "mask sum less than 100\n";
            return false;
        }

        cv::Mat inkMask = cv::Mat::zeros(argmax_image.size(), CV_8UC1);
        cv::Mat referMask = cv::Mat::zeros(argmax_image.size(), CV_8UC1);
        for (int row = 0; row < argmax_image.rows; row++)
        {
            for (int col = 0; col < argmax_image.cols; col++)
            {
                if (argmax_image.at<uchar>(row, col) == 1)
                {
                    inkMask.at<uchar>(row, col) = 255;
                }
                else if (argmax_image.at<uchar>(row, col) == 2)
                {
                    referMask.at<uchar>(row, col) = 255;
                }
            }
        }
        bool result{ false };
        result = ProcessReferMask(referMask, image_gray, refer_line);
        if (!result)
        {
            std::cerr << "ProcessReferMask failed\n";
            return false;
        }

        result = ProcessInkMask(inkMask, image_gray, refer_line, ink_line);
        if (!result)
        {
            std::cerr << "ProcessInkMask failed\n";
            return false;
        }

        return true;
    }

    void DetectorOnCPU::InferOnce(std::wstring model_path, cv::Mat image)
    {
        Init(model_path);

        cv::Mat image_gray, image_rgb, blob;
        auto begin = std::chrono::steady_clock::now();
        Preprocess(image, m_ctx.input_shape, image_rgb, image_gray, blob);
        image = image_rgb;

        std::vector<Ort::Value> output_tensors;
        Infer(blob, output_tensors);

        std::vector<float> ink_line;
        std::vector<float> refer_line;
        bool result = Postprocess(output_tensors, image_gray, refer_line, ink_line);
        if (!result)
        {
            std::cerr << "Postprocess failed\n";
            return;
        }
        auto end = std::chrono::steady_clock::now();
        std::cerr << "infer time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms\n";

        cv::line(image, cv::Point(refer_line.at(0), refer_line.at(1)), cv::Point(refer_line.at(2), refer_line.at(3)), cv::Scalar(0, 0, 255), 2);
        cv::line(image, cv::Point(ink_line.at(0), ink_line.at(1)), cv::Point(ink_line.at(2), ink_line.at(3)), cv::Scalar(0, 255, 0), 2);

        cv::imshow("image", image);
        cv::waitKey(0);
    }
}

#include "tensorrt.hpp"

#ifdef GPU_FLAG
namespace Infer
{
    static int DEVICE{ 0 };
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


    DetectorOnGPU::DetectorOnGPU()
    {

    }

    DetectorOnGPU::~DetectorOnGPU()
    {

    }

    nvinfer1::ICudaEngine* DetectorOnGPU::LoadEngine(std::string engine_path)
    {
        nvinfer1::ICudaEngine* cuda_engine{ nullptr };

        std::ifstream engine_file(engine_path, std::ios::binary);
        if (!engine_file)
        {
            std::cerr << "Failed to open engine file: " << engine_path << "\n";
            return cuda_engine;
        }

        engine_file.seekg(0, engine_file.end);
        size_t size = engine_file.tellg();
        engine_file.seekg(0, engine_file.beg);

        std::vector<char> engine_data(size);
        engine_file.read(engine_data.data(), engine_data.size());

        nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(gLogger);
        cuda_engine = runtime->deserializeCudaEngine(engine_data.data(), engine_data.size(), nullptr);

        return cuda_engine;
    }

    std::vector<float> DetectorOnGPU::Preprocess(cv::Mat& image, int input_height, int input_width)
    {
        cv::Mat img;
        std::vector<float> blob(image.total() * 3);

        if (image.channels() == 1)
            cv::cvtColor(image, img, cv::COLOR_GRAY2BGR);
        else
            img = image;

        cv::Size float_image_size{ input_width, input_height };

        if (img.cols != input_width || img.rows != input_height)
            cv::resize(img, img, float_image_size);

        cv::Mat float_image;
        img.convertTo(float_image, CV_32F, 1.0f / 255.0);
        float_image = (float_image - 0.5) / 0.5;

        std::vector<cv::Mat> chw(3);
        for (int i = 0; i < chw.size(); i++)
        {
            chw[i] = cv::Mat(float_image_size, CV_32FC1, blob.data() + i * float_image_size.width * float_image_size.height);
        }

        cv::split(float_image, chw);

        return blob;
    }

    cv::Mat DetectorOnGPU::Predict(nvinfer1::Dims output_dims, nvinfer1::IExecutionContext* context, std::vector<float>& input, std::vector<float>& blob, cv::Size input_shape, int output_height, int output_width, int output_size)
    {
        const nvinfer1::ICudaEngine& engine = context->getEngine();

        void* buffers[2];

        const int inputIndex = engine.getBindingIndex(INPUT_BLOB_NAME);
        assert(engine.getBindingDataType(inputIndex) == nvinfer1::DataType::kFLOAT);
        const int outputIndex = engine.getBindingIndex(OUTPUT_BLOB_NAME);
        assert(engine.getBindingDataType(outputIndex) == nvinfer1::DataType::kFLOAT);

        CHECK(cudaMalloc(&buffers[inputIndex], 3 * input_shape.height * input_shape.width * sizeof(float)));
        CHECK(cudaMalloc(&buffers[outputIndex], output_size * sizeof(float)));

        // Create stream
        cudaStream_t stream;
        CHECK(cudaStreamCreate(&stream));

        // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
        CHECK(cudaMemcpyAsync(buffers[inputIndex], input.data(), 3 * input_shape.height * input_shape.width * sizeof(float), cudaMemcpyHostToDevice, stream));
        //    context.enqueue(1, buffers, stream, nullptr);
        context->enqueueV2((void**)buffers, stream, nullptr);
        CHECK(cudaMemcpyAsync(blob.data(), buffers[outputIndex], output_size * sizeof(float), cudaMemcpyDeviceToHost, stream));
        cudaStreamSynchronize(stream);

        // Release stream and buffers
        cudaStreamDestroy(stream);
        CHECK(cudaFree(buffers[inputIndex]));
        CHECK(cudaFree(buffers[outputIndex]));

        std::vector<int> mask_size{ 1, 3, output_height, output_width };
        cv::Mat res = cv::Mat(mask_size, CV_32F, blob.data());

        return res;
    }

    std::vector<std::vector<cv::Point2f>> DetectorOnGPU::GetPossibleLines(const cv::Mat& mask)
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

    bool DetectorOnGPU::ProcessReferMask(cv::Mat referMask, cv::Mat image_gray, std::vector<float>& line_res)
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

    bool DetectorOnGPU::ProcessInkMask(cv::Mat inkMask, cv::Mat image_gray, const std::vector<float>& refer_line, std::vector<float>& line_res)
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

    bool DetectorOnGPU::Postprocess(cv::Mat res, cv::Mat image_gray, std::vector<float>& refer_line, std::vector<float>& ink_line, int output_height, int output_width)
    {
        cv::Mat argmax;

        res = res.reshape(0, { 3, output_height * output_width }).t();
        res = res.reshape(3, { output_height, output_width });

        int rows = res.rows;
        int cols = res.cols;
        int channels = res.channels();

        cv::Mat argmax_image(rows, cols, CV_8UC1);

        for (int row = 0; row < rows; row++)
        {
            for (int col = 0; col < cols; col++)
            {
                float* ptr = const_cast<float*>(res.ptr<float>(row, col));
                int max_idx{ 0 };
                float max_value = ptr[0];
                for (int channel = 1; channel < channels; channel++)
                {
                    if (ptr[channel] > max_value)
                    {
                        max_value = ptr[channel];
                        max_idx = channel;
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

    void DetectorOnGPU::InferOnce(std::string model_path, std::string image_path)
    {
        cudaSetDevice(DEVICE);
        auto cuda_engine = LoadEngine(model_path);
        auto context = cuda_engine->createExecutionContext();

        auto input_dimension = cuda_engine->getBindingDimensions(0);
        auto output_dimension = cuda_engine->getBindingDimensions(1);

        int input_height = input_dimension.d[2];
        int input_width = input_dimension.d[3];
        int output_height = output_dimension.d[2];
        int output_width = output_dimension.d[3];
        int output_size = output_height * output_width;
        m_width = input_width;
        m_height = input_height;
        std::cerr << "input_height: " << input_height << ", input width: " << input_width << "\n"
            << "output_height: " << output_height << ", output_width: " << output_width << "\n";

        cv::Mat img = cv::imread(image_path);
        if (img.empty())
        {
            std::cerr << "invalid image path: " << image_path << "\n";
            return;
        }

        std::vector<float> input_buffer = Preprocess(img, input_height, input_width);
        cv::Size input_shape{ input_height, input_width };

        size_t temp_size{ 1 };
        for (int i = 0; i < output_dimension.nbDims; i++)
            temp_size *= output_dimension.d[i];

        std::vector<float> blob(temp_size);

        auto begin = std::chrono::steady_clock::now();

        cv::Mat res = Predict(output_dimension, context, input_buffer, blob, input_shape, output_height, output_width, temp_size);
        if (res.empty())
        {
            std::cerr << "failed to predict...";
            return;
        }
        auto end = std::chrono::steady_clock::now();
        std::cerr << "duration: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms\n";

        cv::Mat image_gray = img.clone();
        std::vector<float> ink_line;
        std::vector<float> refer_line;
        cv::cvtColor(image_gray, image_gray, cv::COLOR_RGB2GRAY);

        bool flag = Postprocess(res, image_gray, refer_line, ink_line, output_height, output_width);
        if (!flag)
        {
            std::cerr << "Postprocess failed\n";
            return;
        }
        cv::line(img, cv::Point(refer_line.at(0), refer_line.at(1)), cv::Point(refer_line.at(2), refer_line.at(3)), cv::Scalar(0, 0, 255));
        cv::line(img, cv::Point(ink_line.at(0), ink_line.at(1)), cv::Point(ink_line.at(2), ink_line.at(3)), cv::Scalar(0, 255, 0), 2);

        cv::imshow("img", img);
        cv::waitKey(0);
    }
}  // namespace Infer

#endif // GPU_FLAG

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


    static const char* INPUT_BLOB_NAME = "x";
    static const char* OUTPUT_BLOB_NAME = "save_infer_model/scale_0.tmp_0";

    Logger gLogger;

    nvinfer1::ICudaEngine* LoadEngine(std::string engine_path)
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

    std::vector<float> Preprocess(cv::Mat& image, int input_height, int input_width)
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

    void Predict(nvinfer1::Dims output_dims, nvinfer1::IExecutionContext& context, std::vector<float>& input, std::vector<float>& output, cv::Size input_shape)
    {
        size_t output_size{ 1 };
        for (int i = 0; i < output_dims.nbDims; i++)
            output_size *= output_dims.d[i];

        std::vector<float> blob(output_size);

        const nvinfer1::ICudaEngine& engine = context.getEngine();

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
        context.enqueueV2((void**)buffers, stream, nullptr);
        CHECK(cudaMemcpyAsync(output.data(), buffers[outputIndex], output_size * sizeof(float), cudaMemcpyDeviceToHost, stream));
        cudaStreamSynchronize(stream);

        // Release stream and buffers
        cudaStreamDestroy(stream);
        CHECK(cudaFree(buffers[inputIndex]));
        CHECK(cudaFree(buffers[outputIndex]));


    }

    void Init()
    {
        std::string engine_path{ VISION_MODEL_PATH };
        cudaSetDevice(DEVICE);
        auto cuda_engine = LoadEngine(engine_path);
        auto context = cuda_engine->createExecutionContext();

        auto input_dimension = cuda_engine->getBindingDimensions(0);
        auto output_dimension = cuda_engine->getBindingDimensions(1);

        int input_height = input_dimension.d[2];
        int input_width = input_dimension.d[3];
        int output_height = output_dimension.d[2];
        int output_width = output_dimension.d[3];
        int output_size = output_height * output_width;
        std::cerr << "input_height: " << input_height << ", input width: " << input_width << "\n"
            << "output_height: " << output_height << ", output_width: " << output_width << "\n";
    }



}  // namespace Infer

#endif // GPU_FLAG

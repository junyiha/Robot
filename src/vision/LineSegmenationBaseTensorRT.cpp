//
// Created by Administrator on 2024/10/29.
//

#include "LineSegmenationBaseTensorRT.h"

#ifdef GPU_FLAG
LineSegmenationBaseTensorRT::LineSegmenationBaseTensorRT()
{

    cudaSetDevice(DEVICE);
    cudaEngine = loadEngine(engineFilePath);
    context = createExecutionContext(cudaEngine);

    inputHeight = cudaEngine->getBindingDimensions(0).d[2];
    inputWidth = cudaEngine->getBindingDimensions(0).d[3];
    outputHeight = cudaEngine->getBindingDimensions(1).d[2];
    outputWidth = cudaEngine->getBindingDimensions(1).d[3];
    outputSize = outputHeight * outputWidth;
}

LineSegmenationBaseTensorRT::~LineSegmenationBaseTensorRT()
{
    delete cudaEngine;
    delete context;
}

nvinfer1::ICudaEngine* LineSegmenationBaseTensorRT::loadEngine(const std::string& engineFilePath)
{

    std::ifstream engineFile(engineFilePath, std::ios::binary);
    if (!engineFile)
    {
        std::cerr << "Failed to open engine file: " << engineFilePath << std::endl;
        return nullptr;
    }

    engineFile.seekg(0, engineFile.end);
    size_t size = engineFile.tellg();
    engineFile.seekg(0, engineFile.beg);

    std::vector<char> engineData(size);
    engineFile.read(engineData.data(), size);
    engineFile.close();

    nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(gLogger);
    nvinfer1::ICudaEngine* engine = runtime->deserializeCudaEngine(engineData.data(), size, nullptr);
    runtime->destroy();
    return engine;
}

nvinfer1::IExecutionContext* LineSegmenationBaseTensorRT::createExecutionContext(nvinfer1::ICudaEngine* engine)
{
    return engine->createExecutionContext();
}

float* LineSegmenationBaseTensorRT::preprocessImage(cv::Mat& image, int inputHeight, int inputWidth)
{

    float* blob = new float[image.total() * 3];
    // 图像预处理
    cv::Mat img_resize, img;


    // 通道处理
    if (image.channels() == 1)
    {
        cv::cvtColor(image, img, cv::COLOR_GRAY2BGR);
    }
    else
    {
        img = image;
    }

    int imgW = img.cols;
    int imgH = img.rows;
    if (imgW == this->inputWidth && imgH == this->inputHeight)
    {
        img_resize = img;
    }
    else
    {
        cv::resize(img, img_resize, cv::Size(inputWidth, inputHeight));
    }

    cv::Mat floatImage;
    img_resize.convertTo(floatImage, CV_32F, 1.0f / 255.0);
    floatImage = (floatImage - 0.5) / 0.5;

    cv::Size floatImageSize{ img_resize.cols, img_resize.rows };
    // hwc -> chw
    std::vector<cv::Mat> chw(3);
    for (int i = 0; i < 3; ++i)
    {
        chw[i] = cv::Mat(floatImageSize, CV_32FC1, blob + i * floatImageSize.width * floatImageSize.height);
    }
    cv::split(floatImage, chw);

    return blob;
}

void LineSegmenationBaseTensorRT::infer(nvinfer1::IExecutionContext* context, float* inputBuffer, float* outputBuffer,
                                        int batchSize, int inputHeight, int inputWidth, int outputSize)
{

    void* buffers[2];
    cudaMalloc(&buffers[0], batchSize * inputHeight * inputWidth * 3 * sizeof(float));
    cudaMalloc(&buffers[1], batchSize * outputSize * sizeof(float));

    cudaMemcpy(buffers[0], inputBuffer, batchSize * inputHeight * inputWidth * 3 * sizeof(float), cudaMemcpyHostToDevice);

    context->execute(batchSize, buffers);

    cudaMemcpy(outputBuffer, buffers[1], batchSize * outputSize * sizeof(float), cudaMemcpyDeviceToHost);

    cudaFree(buffers[0]);
    cudaFree(buffers[1]);

}

void LineSegmenationBaseTensorRT::doInference(nvinfer1::IExecutionContext& context, float* input, float* output,
                                              const int output_size, cv::Size input_shape)
{

    const nvinfer1::ICudaEngine& engine = context.getEngine();
    // Pointers to input and output device buffers to pass to engine.
    // Engine requires exactly IEngine::getNbBindings() number of buffers.
    assert(engine.getNbBindings() == 2);
    void* buffers[2];

    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine.getBindingIndex(INPUT_BLOB_NAME);

    assert(engine.getBindingDataType(inputIndex) == nvinfer1::DataType::kFLOAT);
    const int outputIndex = engine.getBindingIndex(OUTPUT_BLOB_NAME);
    assert(engine.getBindingDataType(outputIndex) == nvinfer1::DataType::kFLOAT);
    //    int mBatchSize = engine.getMaxBatchSize();

        // Create GPU buffers on device
    CHECK(cudaMalloc(&buffers[inputIndex], 3 * input_shape.height * input_shape.width * sizeof(float)));
    CHECK(cudaMalloc(&buffers[outputIndex], output_size * sizeof(float)));

    // Create stream
    cudaStream_t stream;
    CHECK(cudaStreamCreate(&stream));

    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CHECK(cudaMemcpyAsync(buffers[inputIndex], input, 3 * input_shape.height * input_shape.width * sizeof(float), cudaMemcpyHostToDevice, stream));
    //    context.enqueue(1, buffers, stream, nullptr);
    context.enqueueV2((void**)buffers, stream, nullptr);
    CHECK(cudaMemcpyAsync(output, buffers[outputIndex], output_size * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);

    // Release stream and buffers
    cudaStreamDestroy(stream);
    CHECK(cudaFree(buffers[inputIndex]));
    CHECK(cudaFree(buffers[outputIndex]));
}

cv::Mat LineSegmenationBaseTensorRT::predict(cv::Mat inputImg)
{

    //图像预处理

    auto out_dims = cudaEngine->getBindingDimensions(1);
    // 分配输入，输出缓存
    auto output_size = 1;
    for (int j = 0; j < out_dims.nbDims; j++)
    {
        output_size *= out_dims.d[j];
    }

    float* prob = new float[output_size];
    //    float* inputBuffer = new float[inputHeight * inputWidth * 3];
    auto start = std::chrono::system_clock::now();
    // 图像预处理
    float* inputBuffer = preprocessImage(inputImg, inputWidth, inputHeight);


    doInference(*context, inputBuffer, prob, output_size, cv::Size(768, 768));
    auto end = std::chrono::system_clock::now();
    //    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    std::vector<int> mask_sz = { 1,3, outputHeight,outputWidth };
    cv::Mat res = cv::Mat(mask_sz, CV_32F, prob);
    cv::Mat mask_res = post_process(res);
    // 释放指针
    delete prob;
    delete inputBuffer;

    return mask_res;

}

cv::Mat LineSegmenationBaseTensorRT::computeArgmax(const cv::Mat& input)
{   //softmax 函数实现

    int rows = input.rows;
    int cols = input.cols;
    int channels = input.channels();

    cv::Mat argmaxImage(rows, cols, CV_8UC1); // 创建一个用于存储每像素最大值通道索引的矩阵，类型为32位有符号整数

    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            float* ptr_ = const_cast<float*>(input.ptr<float>(row, col));
            int maxIdx = 0;
            float maxValue = ptr_[0];
            for (int channel = 1; channel < channels; ++channel)
            {
                if (ptr_[channel] > maxValue)
                {
                    maxValue = ptr_[channel];
                    maxIdx = channel;
                }
            }
            argmaxImage.at<uchar>(row, col) = maxIdx; // 存储最大值对应的通道索引
        }
    }
    return argmaxImage;
}

cv::Mat LineSegmenationBaseTensorRT::post_process(cv::Mat res)
{
    cv::Mat argmax;
    res = res.reshape(0, { 3, outputHeight * outputWidth }).t();
    res = res.reshape(3, { outputHeight, outputWidth });
    argmax = computeArgmax(res);
    return argmax;
}

float* LineSegmenationBaseTensorRT::blobFromImage(cv::Mat& img)
{

    float* blob = new float[img.total() * 3];
    int channels = 3;
    int img_h = img.rows;
    int img_w = img.cols;
    for (size_t c = 0; c < channels; c++)
    {
        for (size_t h = 0; h < img_h; h++)
        {
            for (size_t w = 0; w < img_w; w++)
            {
                blob[c * img_w * img_h + h * img_w + w] =
                    ((((float)img.at<cv::Vec3b>(h, w)[c]) / 255.0) - 0.5) / 0.5;
            }
        }
    }
    return blob;

}

#endif // GPU_FLAG
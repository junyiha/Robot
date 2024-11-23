//
// Created by csh_i on 2024/8/15.
//

#ifndef GPU_FLAG
#include "LineSegmenationBase.h"
//#include<cuda_provider_factory.h>
using namespace std;
using namespace Ort;

LineSegmenationBase::LineSegmenationBase() {

    // std::string model_path = "../models/pp_liteseg_stdc1_softmax_20241021.onnx";
    std::string model_path = "E:/projects/zjy/Robot-zb/models/pp_liteseg_stdc1_softmax_20241021.onnx";
    model_path = VISION_MODEL_PATH;
//    std::string model_path = "../models/model_ocrnet-20241020.onnx";
    std::wstring widestr = std::wstring(model_path.begin(), model_path.end());

    // ONNXRUNTIME 会话相关配置

    sessionOptions.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    sessionOptions.SetIntraOpNumThreads(3);
    sessionOptions.SetExecutionMode(ORT_SEQUENTIAL);

    std::vector<std::string> availableProviders = Ort::GetAvailableProviders();
    auto cudaAvailable = std::find(availableProviders.begin(), availableProviders.end(), "CUDAExecutionProvider");
    if (onnx_provider == OnnxProviders::CUDA.c_str()) {  // strcmp(provider, OnnxProviders::CUDA.c_str()) == true strcmp(provider, "cuda") // (providerStr == "cuda")
        if (cudaAvailable == availableProviders.end()) {
            std::cout << "CUDA is not supported by your ONNXRuntime build. Fallback to CPU." << std::endl;
            //std::cout << "Inference device: CPU" << std::endl;
        }
        else {
            std::cout << "Inference device: GPU" << std::endl;
#ifdef GPU_FLAG
            OrtCUDAProviderOptions cuda_options{0};
            cuda_options.gpu_mem_limit = 10*1024*1024*1024; // 显存限制在10G
            cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearch::EXHAUSTIVE;
            cuda_options.do_copy_in_default_stream = true;
            cuda_options.arena_extend_strategy = 0;
            sessionOptions.AppendExecutionProvider_CUDA(cuda_options);
#endif
        }
    }
    else if (onnx_provider == OnnxProviders::CPU.c_str()) {  // strcmp(provider, OnnxProviders::CPU.c_str()) == true) (providerStr == "cpu") {
        // "cpu" by default
    }
    else
    {
        throw std::runtime_error("NotImplemented provider=" + std::string(onnx_provider));
    }


    ort_session = new Session(env, widestr.c_str(), sessionOptions);
    size_t numInputNodes = ort_session->GetInputCount(); //输入节点数
    size_t numOutputNodes = ort_session->GetOutputCount();//输出节点数
    //获取ONNX模型输入和输出节点信息
    AllocatorWithDefaultOptions allocator;
    for (int i = 0; i < numInputNodes; i++)
    {
        input_names.push_back(ort_session->GetInputName(i, allocator));
        Ort::TypeInfo input_type_info = ort_session->GetInputTypeInfo(i);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        auto input_dims = input_tensor_info.GetShape();
        input_node_dims.push_back(input_dims);
    }
    for (int i = 0; i < numOutputNodes; i++)
    {
        output_names.push_back(ort_session->GetOutputName(i, allocator));
        Ort::TypeInfo output_type_info = ort_session->GetOutputTypeInfo(i);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        auto output_dims = output_tensor_info.GetShape();
        output_node_dims.push_back(output_dims);
    }
    this->inpHeight = input_node_dims[0][2];
    this->inpWidth = input_node_dims[0][3];
}

void LineSegmenationBase::init_model() {

    std::cout << "init model" << std::endl;
    std::cout << "input_names: " << input_names[0] << std::endl;
    for(int i = 0; i < output_node_dims.size(); i++){
        std::cout << "input_node_dims: " << output_node_dims[i][0] << " " << output_node_dims[i][1] << " " << output_node_dims[i][2] << std::endl;
    }
}

// 批量对图像进行推理
LineSegmenationBase::~LineSegmenationBase() {

}

cv::Mat LineSegmenationBase::get_input_tensor(cv::Mat inputImg) {

    // 图像预处理
    cv::Mat img_resize, img;
    cv::Mat blob;

    if(inputImg.channels() == 1){
        cv::cvtColor(inputImg, img, cv::COLOR_GRAY2RGB);
    }else{
        img = inputImg;
    }

    int imgW = img.cols;
    int imgH = img.rows;
    if(imgW == this->inpWidth && imgH == this->inpHeight){
        img_resize = img;
    }else{
        cv::resize(img, img_resize, cv::Size(this->inpWidth, this->inpHeight));
    }

    img_resize.convertTo(img_resize, CV_32F);
    img_resize = ((img_resize/255.0)-0.5)/0.5;
    blob = cv::dnn::blobFromImage(img_resize, 1.0, cv::Size(this->inpHeight, this->inpWidth), cv::Scalar(0,0,0), true, false);
    return blob;
}



void LineSegmenationBase::Normalize(cv::Mat src, cv::Mat &dst) {

//    vector<float> mean_value{0.406, 0.456, 0.485};
//    vector<float> std_value{0.225, 0.224, 0.229};

    vector<float> mean_value{0.5, 0.5, 0.5};
    vector<float> std_value{0.5,  0.5,  0.5};

    vector<cv::Mat> bgrChannels(3);
    cv::split(src, bgrChannels);
    for (auto i = 0; i < bgrChannels.size(); i++)
    {
        bgrChannels[i].convertTo(bgrChannels[i], CV_32FC1, 1.0 / std_value[i], (0.0 - mean_value[i]) / std_value[i]);
    }
    cv::merge(bgrChannels, dst);
}


cv::Mat LineSegmenationBase::computeArgmax(const cv::Mat& input) {   //softmax 函数实现

    int rows = input.rows;
    int cols = input.cols;
    int channels = input.channels();

    cv::Mat argmaxImage(rows, cols, CV_8UC1); // 创建一个用于存储每像素最大值通道索引的矩阵，类型为32位有符号整数

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            float* ptr_ = const_cast<float *>(input.ptr<float>(row, col));
            int maxIdx = 0;
            float maxValue = ptr_[0];
            for (int channel = 1; channel < channels; ++channel) {
                if (ptr_[channel] > maxValue) {
                    maxValue = ptr_[channel];
                    maxIdx = channel;
                }
            }
            argmaxImage.at<uchar>(row, col) = maxIdx; // 存储最大值对应的通道索引
        }
    }
    return argmaxImage;
}


cv::Mat LineSegmenationBase::predict(cv::Mat img) {

    cv::Mat input_img = this->get_input_tensor(img);

    array<int64_t, 4> input_shape_{ 1, 3, this->inpHeight, this->inpWidth };
    auto allocator_info = MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

    std::vector<Ort::Value> input_tensor_;
    input_tensor_.emplace_back(Value::CreateTensor<float>(allocator_info, input_img.ptr<float>(), input_img.total(), input_shape_.data(), input_shape_.size()));

    std::vector<Value> ort_outputs;
    ort_outputs = ort_session->Run(RunOptions{ nullptr }, &input_names[0], input_tensor_.data(), 1, output_names.data(), output_names.size());
    assert(ort_outputs.size() == 1 && ort_outputs.front().IsTensor());
    auto data_shape = ort_outputs.front().GetTensorTypeAndShapeInfo().GetShape();
    this->outputHeight = data_shape[2];
    this->outputWidth = data_shape[3];
    this->outputChannels = data_shape[1];
    //解析模型推理结果
    vector<int> mask_sz = { 1,outputChannels, outputHeight,outputWidth};
    cv::Mat res = cv::Mat(mask_sz, CV_32F, ort_outputs.front().GetTensorMutableData<float>());
    cv::Mat mask_index = post_process(res);
    return mask_index;
}

cv::Mat LineSegmenationBase::post_process(cv::Mat res) {
    cv::Mat argmax;
    res = res.reshape(0, {outputChannels, outputHeight*outputWidth}).t();
    res = res.reshape(outputChannels, {outputHeight, outputWidth});
    argmax = computeArgmax(res);
    return argmax;
}

void LineSegmenationBase::imageEnhance(cv::Mat &img) {

     // 直方图均衡化
     cv::equalizeHist(img, img);

}
#endif  // GPU_FLAG
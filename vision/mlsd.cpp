#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/dnn.hpp>
#include <onnxruntime_cxx_api.h>
#include "mlsd.h"
#include<vector>
#include<chrono>
#include<filesystem>

using namespace std;
using namespace Ort;



float getMax(cv::Mat matrix, int kernel_size, int x, int y) {
    float max_value = matrix.at<float>(x, y);
    for (int i = 0; i < kernel_size; i++) {
        for (int j = 0; j < kernel_size; j++) {
            if (max_value < matrix.at<float>(x + i, y + j))
            {
                max_value = matrix.at<float>(x + i, y + j);
            }
        }
    }
    return max_value;
}

cv::Mat maxPool(cv::Mat image, int poolSize, int stride) {

    //计算输出图像的大小
    cv::Size outputSize((image.cols - poolSize) / stride + 1,
                        (image.rows - poolSize) / stride + 1);
    // 创建输出图像
    cv::Mat pooledImage(outputSize, image.type());
    for (int y = 0; y < outputSize.height; ++y) {
        for (int x = 0; x < outputSize.width; ++x) {
            cv::Mat roi(image, cv::Rect(x * stride, y * stride, poolSize, poolSize));
            double temp;
            cv::minMaxLoc(roi, nullptr, &temp, nullptr, nullptr);
            pooledImage.at<float>(y, x) = temp;
        }
    }
    return pooledImage;
}

//最大池化
cv::Mat Pool(cv::Mat feat_mat, int kernel_size = 3, int strides = 2)
{
    int src_img_h = feat_mat.rows;
    int src_img_w = feat_mat.cols;
    int new_h = int((feat_mat.rows - kernel_size) / strides + 1);
    int new_w = int((feat_mat.cols - kernel_size) / strides + 1);

    cv::Mat new_mat = cv::Mat(new_w, new_h, CV_32F);
    for (int i = 0; i < new_mat.rows; i++)
    {
        for (int j = 0; j < new_mat.cols; j++)
        {

            float gray_value = getMax(feat_mat, kernel_size, i*strides, j*strides);
            new_mat.at<float>(i, j) = gray_value;

        }
    }
    return new_mat;
}
//获取topk最大值和索引
void getTopKMaxValuesAndIndices(const cv::Mat& inputArray, int k, std::vector<double>& maxValues, std::vector<int>& maxIndices) {
    // 对输入数组进行排序，返回排序后的索引
    cv::Mat sortedIndices;
    cv::sortIdx(inputArray, sortedIndices, cv::SORT_EVERY_COLUMN + cv::SORT_DESCENDING);

    // 获取前k个最大值的索引和根据索引从原始数组中提取前k个最大值
    for (int i = 0; i < k; ++i) {
        maxIndices.push_back(sortedIndices.at<int>(i));
        maxValues.push_back(inputArray.at<float>(sortedIndices.at<int>(i)));
    }

}

// MLSD对象构造函数
MLSD::MLSD()
{
//    std::filesystem::path current_path = std::filesystem::current_path();
//    string model_path = "D:\\_Project\\Ship\\program\\ZBRobot\\ZBRobotV24\\models\\mlsd_large.onnx";
//    string model_path = "E:\\project\\qt\\vision_test\\vision_test_v3\\vision_test_v3\\models\\mlsd_large.onnx";
    //string model_path = "../models/mlsd_model_tiny_20240721.onnx";
    std::string model_path = "D:/PDRobot-master/models/mlsd_model_tiny_20240721.onnx";
    std::wstring widestr = std::wstring(model_path.begin(), model_path.end());

    this->score_thr = 0.1;
    this-> dist_thr = 20.0;

    // ONNXRUNTIME 会话相关配置
    sessionOptions.SetGraphOptimizationLevel(ORT_ENABLE_BASIC);
    sessionOptions.SetIntraOpNumThreads(3);
    sessionOptions.SetExecutionMode(ORT_SEQUENTIAL);
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

//模型输入数据处理
cv::Mat MLSD::img_processing(cv::Mat img) {

    cv::Mat blob;
    cv::Mat img_512;
    cv::Mat img_merge;
    if (img.rows!=512 || img.cols!=512) {
        cv::resize(img, img_512, cv::Size(this->inpWidth, this->inpHeight));
    }
    else {
        img_512 = img.clone();
    }

    //    cvtColor(img_512, img_512, cv::COLOR_BGR2RGB); //色彩空间转换

    //    //构造1x4x512x512模型输入
    //    cv::Mat img_one = cv::Mat::ones(this->inpHeight, this->inpWidth, CV_32F);
    //    img_512.convertTo(img_512, CV_32F);
    //    vector<cv::Mat> channels;
    //    cv::split(img_512, channels);
    //    channels.push_back(img_one);
    //    cv::merge(channels, img_merge);
    //    img_merge = (img_merge / 127.5) - 1;
    //    blob = cv::dnn::blobFromImage(img_merge, 1.0, cv::Size(this->inpHeight, this->inpWidth), cv::Scalar(0), false, false);
    //    return blob;
       cv::Mat img_gray, img_new;
       cvtColor(img_512, img_gray, cv::COLOR_BGR2GRAY);
       std::vector<cv::Mat> img_input;
       img_input.push_back(img_gray);
       img_input.push_back(img_gray);
       img_input.push_back(img_gray);
       cv::merge(img_input, img_new);


           //构造1x4x512x512模型输入
       //    Mat img_one = cv::Mat::ones(this->inpHeight, this->inpWidth, CV_32F);

       img_new.convertTo(img_new, CV_32F);
       //    vector<cv::Mat> channels;
       //    cv::split(img_512, channels);
       //    channels.push_back(img_one);
       //    cv::merge(channels, img_merge);
       img_merge = (img_new / 127.5) - 1;
       blob = cv::dnn::blobFromImage(img_merge, 1.0, cv::Size(this->inpHeight, this->inpWidth), cv::Scalar(0), false, false);
       return blob;
}

// 模型推理结果后处理s
vector<vector<float>> MLSD::post_processing(cv::Mat res) {
    // 模型预测结果进行后处理，获取真实直线坐标信息

    int rows = res.rows;
    int cols = res.cols;

    vector<cv::Mat> displacements, channels; //预测偏置结果
    cv::Mat center; //预测中心结果
    split(res, channels);
    //获取中心图预测
    center = channels[0];
    //处理displacemont
    for (int i = 1; i < 5; i++) {
        displacements.push_back(channels[i]);
    }

    //对center执行sigmoid操作
    cv::Mat heat, hmax, dst, heat_padd, heat_roi, temp;
    exp(-center, heat);
    heat = 1.0 / (1.0 + heat);
    //对heat进行重采样操作
    cv::copyMakeBorder(heat, heat_padd, 1, 1, 1, 1, 0, cv::Scalar(0.0)); // 池化操作存在问题 
    hmax = Pool(heat_padd, 3, 1);
    /*hmax = maxPool(heat_padd, 3, 1);*/
    cv::compare(heat, hmax, dst, cv::CMP_EQ);

    dst.convertTo(dst, CV_32F);
    dst = dst / 255.0;
    cv::multiply(heat, dst, heat_roi);
    heat_roi = heat.reshape(0, { heat.cols*heat.rows,1 });

    std::vector<double> maxValues; //topk个最大值
    std::vector<int> maxIndices;   //topk各最大值的索引
    getTopKMaxValuesAndIndices(heat_roi, 200, maxValues, maxIndices);

    // 坐标转换
    cv::Mat max_index(maxIndices);
    cv::Mat yy(maxIndices.size(), 1, CV_32S), xx(maxIndices.size(), 1, CV_32S);
    for (int i = 0; i < max_index.rows; i++) {
        int value = maxIndices[i];
        yy.at<int>(i, 0) = cvFloor( static_cast<float>(value) / static_cast<float>(cols));
        xx.at<int>(i, 0) = value % cols;
    }

    vector<cv::Mat> ptss(2); // 存储yy，xx坐标
    ptss.push_back(yy);
    ptss.push_back(xx);

    vector<cv::Mat> start, end;
    cv::Mat start_mat, end_mat;
    start = { displacements[0],displacements[1]};  //起点坐标
    end =   { displacements[2],displacements[3] };   //终点坐标
    merge(start, start_mat);
    merge(end, end_mat);
    //计算dist_map
    cv::Mat xx_dist, yy_dist, dis_map;
    cv::pow((displacements[0] - displacements[2]), 2, xx_dist);
    cv::pow((displacements[1] - displacements[3]), 2, yy_dist);
    cv::sqrt(xx_dist+yy_dist, dis_map);

    // 构建distance map
    vector<vector<float>> points;
    for (int i = 0; i < yy.rows; i++) {
        int y = yy.at<int>(i, 0);
        int x = xx.at<int>(i, 0);
        float distance = dis_map.at<float>(y, x);
        double score = maxValues[i];
        if (score >this->score_thr && distance > this->dist_thr) {
            float disp_x_start = start_mat.at<cv::Vec2f>(y, x)[0];
            float disp_y_start = start_mat.at<cv::Vec2f>(y, x)[1];
            float disp_x_end = end_mat.at<cv::Vec2f>(y, x)[0];
            float disp_y_end = end_mat.at<cv::Vec2f>(y, x)[1];
            float x_start = x + disp_x_start;
            float y_start = y + disp_y_start;
            float x_end = x + disp_x_end;
            float y_end = y + disp_y_end;
            points.push_back({x_start * 2, y_start * 2, x_end  * 2,y_end  * 2});
        }
    }
    return points;
}

//检测图像中所有的直线
vector<vector<float>> MLSD::detect(cv::Mat img)
{
    cv::Mat blob = this->img_processing(img);
    array<int64_t, 4> input_shape_{ 1, 3, this->inpHeight, this->inpWidth };
    auto allocator_info = MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

    std::vector<Ort::Value> input_tensor_;
    input_tensor_.emplace_back(Value::CreateTensor<float>(allocator_info, blob.ptr<float>(), blob.total(), input_shape_.data(), input_shape_.size()));

    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<Value> ort_outputs;
    try{
        ort_outputs = ort_session->Run(RunOptions{ nullptr }, &input_names[0], input_tensor_.data(), 1, output_names.data(), output_names.size());
    }catch(...){
        qDebug()<<"\n**************直线检测模型推理失败***************\n";
    };
    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "model inference: " << duration.count() << " milliseconds." << std::endl;
    assert(ort_outputs.size() == 1 && ort_outputs.front().IsTensor());
    auto data_shape = ort_outputs.front().GetTensorTypeAndShapeInfo().GetShape();

    //解析模型推理结果
    vector<int> mask_sz = { 1,9,256,256 };
    cv::Mat res = cv::Mat(mask_sz, CV_32F, ort_outputs.front().GetTensorMutableData<float>());
    /*cv::Mat res(results);*/
    res = res.reshape(0, { 9,256 * 256 }).t();
    res = res.reshape(9, { 256,256});
    vector<vector<float>> result =this->post_processing(res);
    return result;
}

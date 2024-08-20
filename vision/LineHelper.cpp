#include"LineHelper.h"
#include<string>
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui.hpp>
#include "utils.h"
#include<cmath>
#include<chrono>

#define PI acos(-1)


std::vector<cv::Vec4f> detectEDLines(const cv::Mat& binaryImage) {
    std::vector<cv::Vec4f> lines;
    cv::Mat grad;
    // 使用Canny边缘检测器检测边缘
    Canny(binaryImage, grad, 50, 150);

    // 提取非零像素点
    std::vector<cv::Point> points;
    for (int y = 0; y < grad.rows; ++y) {
        for (int x = 0; x < grad.cols; ++x) {
            if (grad.at<uchar>(y, x) != 0) {
                points.push_back(cv::Point(x, y));
            }
        }
    }

    // 对边缘点进行分组以形成直线段（这里省略了分组逻辑）
    // ...

    return lines;
}


/* 直线检测算法类方法实现 */
// LineHelper 构造函数
LineHelper::LineHelper() {
    this->resize_h = 512;
    this->resize_w = 512;

    this->min_pos_valid = 50;
    this->max_pos_valid = 512 - 50;

    this->min_dist_lines = 20;
    this->max_dist_lines = 200;

    this->max_angle_degrees = 10;
    this->min_len_seg = static_cast<unsigned>(512 * 0.1);
}

// 输入图像预处理
void LineHelper::img_processing(cv::Mat img, cv::Mat& img_gray, cv::Mat& bin_img) {
    /**
     * @brief 图像预处理
     *
     * 对输入图像进行预处理，包括调整大小、直方图均衡化、去除边缘锯齿和高斯模糊。
     *
     * @param img 输入图像
     * @param img_gray 灰度图像
     * @param bin_img 二值化图像
     */
    cv::Mat img_eq;
    cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(img_gray, img_gray, cv::Size(5, 5), 0);
    //cv::equalizeHist(img_gray, img_eq); // 不要使用直方图均衡化
    cv::threshold(img_gray, bin_img, 0, 255,  cv::THRESH_OTSU);//OSTU算法二值化分割
//    bin_img = padding_hollow(bin_img);// 边缘去锯齿
}

InkInfo LineHelper::get_ink_rough_property(cv::Mat bin_img, std::vector<float> line) {

    InkInfo ink_info;
    int height = bin_img.rows;
    int width = bin_img.cols;

    int half_height = height / 2;

    // 创建两个Mat对象来存储图像的上下半部分
    cv::Mat upper_half(height / 2, width, bin_img.type());
    cv::Mat lower_half(height - half_height, width, bin_img.type());

    // 提取图像的上半部分
    cv::Rect upper_rect(0, 0, width, half_height);
    bin_img(upper_rect).copyTo(upper_half);

    // 提取图像的下半部分
    cv::Rect lower_rect(0, half_height, width, height - half_height);
    bin_img(lower_rect).copyTo(lower_half);

    cv::Scalar upper_mean = cv::mean(upper_half);
    cv::Scalar lower_mean = cv::mean(lower_half);

    if (upper_mean.val[0] <lower_mean.val[0]) {
        ink_info.is_upper = true;
        float min_value = line[1] < line[3] ? line[1] : line[3];
        ink_info.base_y = min_value;
    }
    else {
        ink_info.is_upper = false;
        float max_value = line[1] > line[3] ? line[1] : line[3];
        ink_info.base_y = max_value;
    }
    return ink_info;
}

//LSD算法进行直线检测
std::vector<cv::Vec4f> LineHelper::line_detect_lsd(cv::Mat binary_img) {
    /**
     * @brief 对二值化分割图像进行直线检测
     *
     * 使用 LSD 算法对二值化分割图像进行直线检测，并返回检测到的直线信息。
     *
     * @param binary_img 二值化分割图像
     *
     * @return 检测到的直线信息
     */
    cv::Ptr<cv::LineSegmentDetector> detector = createLineSegmentDetector(cv::LSD_REFINE_STD);
    std::vector<cv::Vec4f> lines;
    detector->detect(binary_img, lines);
    return lines;
}

// 获取板界线
void LineHelper::get_border_line(cv::Mat img, cv::Mat bin_img, std::vector<std::vector<float>> pred_lines, LineRes& res) {

    /**
     * @brief 获取板线
     *
     * 基于给定的图像和预测直线，获取板线信息并存储到 LineRes 结构体中。
     *
     * @param img 输入的图像
     * @param bin_img 二值化后的图像
     * @param pred_lines 预测直线集合
     * @param res 存储板线信息的 LineRes 结构体引用
     */

    int img_h, img_w;
    img_h = bin_img.rows;
    img_w = bin_img.cols;
    float ratio = 0.3;
    // 基于LSD算法进行直线检测

    auto start_time = std::chrono::high_resolution_clock::now();

    std::vector<cv::Vec4f> lines = line_detect_lsd(bin_img);
    //vector<Vec4f> lines = detectEDLines(bin_img);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);


    //基于LSD算法获取线检测点
    std::vector<cv::Point2f> points;
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4f line = lines[i];
        line[0] = round(line[0]);
        line[1] = round(line[1]);
        line[2] = round(line[2]);
        line[3] = round(line[3]);
        float distance = calculatePointsDistance(cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]));
        // 如何判别检测的线属于板线的范畴？
        // 长度大小、外观纹理、位置特点
        if ((distance > img_w*0.2) && check_is_boundary(bin_img, line)) { //满足直线的长度要求，且直线上下区域像素相似度低
            points.push_back(cv::Point2f(line[0], line[1]));
            points.push_back(cv::Point2f(line[2], line[3]));
            break;
        }
    }


    std::vector<float> line_new; // LSD算法获取的板界线
    // MLSD检测算法进行兜底或者LSD算法结果进行修正
    if (points.size() == 0) {
        for (const auto& row : pred_lines) {
            float start_x = row[0];
            float start_y = row[1];
            float end_x = row[2];
            float end_y = row[3];
            if (start_y < this->min_pos_valid || end_y > max_pos_valid) { continue; }

            float distance = calculatePointsDistance(cv::Point2f(start_x, start_y), cv::Point2f(end_x, end_y));
            cv::Vec4f line = { start_x ,start_y ,end_x,end_y };
            if ((distance > this->min_len_seg) && check_is_boundary(bin_img, line)) {
                points.push_back(cv::Point2f(start_x, start_y));
                points.push_back(cv::Point2f(end_x, end_y));
                break;
            }
        }
    }

    // 对结果进行校验
    if (points.size()>0) {
        // 直线重新拟合
        line_new = fiting_line(points);
        if (line_new.size()>0) {
            res.border_line_status = true;
            res.border_res = line_new;
            cv::line(res.img_drawed,
                     cv::Point( static_cast<int>(line_new[0]), static_cast<int>(line_new[1])),
                     cv::Point(static_cast<int>(line_new[2]), static_cast<int>(line_new[3])),
                     cv::Scalar(0, 0, 255), 2);
        }
        else {
            res.error_info = "板线直线拟和失败!";
        }
    }
    else {
        res.error_info = "板线检测失败,请重新确认输入图像是否符合要求!!!";
    }

}

//// 函数用于计算两条平行直线之间的距离
//double calculateDistanceBetweenParallelLines(double slope, double intercept1, double intercept2) {
//    double denominator = std::sqrt(1 + slope * slope);
//    return std::abs(intercept2 - intercept1) / denominator;
//}

// 获取墨迹线
void LineHelper::get_ink_line(cv::Mat bin_img, std::vector<std::vector<float>> pred_lines, LineRes& res) {
    /**
     * @brief 获取墨迹线
     *
     * 根据 MLSD 预测结果，筛选墨迹线条并存储到 LineRes 结构体中。
     *
     * @param pred_lines MLSD 预测直线集合
     * @param res 存储板线信息的 LineRes 结构体引用
     */
    std::vector<float> border_line_info = res.border_res;
    float slope = border_line_info[4]; //板线斜率
    std::vector<float> ink_line;

    InkInfo ink_info = get_ink_rough_property(bin_img, border_line_info);
    for (const auto& row : pred_lines) {
        float start_x = row[0];
        float start_y = row[1];
        float end_x = row[2];
        float end_y = row[3];
        // 非工作区间内线段过滤
        if (start_y < this->min_pos_valid || end_y > this->max_pos_valid) { continue; }

        // 斜率相同且保持一定的间距、一定的长度
        float distance = calculatePointsDistance(cv::Point2f(start_x, start_y), cv::Point2f(end_x, end_y));
        // 直线拟合
        std::vector<cv::Point2f> points;
        points.push_back(cv::Point2f(start_x, start_y));
        points.push_back(cv::Point2f(end_x, end_y));
        std::vector<float> line_new = fiting_line(points);
        float slope_ink = line_new[4];
        float line_dist = abs(border_line_info[1] - line_new[1]);

        // 计算两条之间角度偏差
        float slope_offset = slope - slope_ink;
        float angle = atan(slope_offset);
        float angle_degrees = abs(angle*(180.0 / PI));

        if (line_dist>this->min_dist_lines &&
            line_dist<this->max_dist_lines&&
            angle_degrees < this->max_angle_degrees &&
            distance>this->min_len_seg ) {
            ink_line = line_new;
            break;
        }
    }

    if (ink_line.size()>0) {
        res.ink_line_status = true;
        res.ink_res = ink_line;
        res.status = true;
        cv::line(res.img_drawed, cv::Point(static_cast<int>(ink_line[0]), static_cast<int>(ink_line[1])),
                 cv::Point(static_cast<int>(ink_line[2]), static_cast<int>(ink_line[3])),
                 cv::Scalar(0,255,0), 2);
    }
    else {
        res.error_info = "墨迹检测失败!";
    }

}

// 获取板线与墨迹线之间的距离
LineRes LineHelper::compute_lines_distance(cv::Mat img) {

    /**
        * @brief 计算直线之间的距离
        *
        * 根据给定的图像，计算板线和墨迹线之间的距离。
        *
        * @param img 输入的图像
        *
        * @return 返回包含直线信息的结构体
        */

    //数据合法性校验
    LineRes res;
    res.status = false;
    res.ink_line_status = false;
    res.border_line_status = false;

    if (img.empty()) {
        res.error_info = "Invalid input image";
        return res;
    }

    //图像宽高比校验
//    if (img.rows > img.cols) {
//        img = rotateImage(img);
//    }
    float origin_h = img.rows;
    float origin_w = img.cols;
    float h_ratio = origin_h / this->resize_h;
    float w_ratio = origin_w / this->resize_w;

    //图像预处理
    cv::Mat img_gray, bin_img, img_512;
    cv::resize(img, img_512, cv::Size(this->resize_w,this->resize_h));
    img_processing(img_512, img_gray, bin_img);
    res.img_drawed = img_512;

    try {
        //MLSD算法直线检测
        std::vector<std::vector<float>> pred_lines = line_tooler.detect(img_512);

        if (!(pred_lines.size() > 0)) {
            res.error_info = "待检测图像场景类型不合法,请重新核对图像内容!!!";
        }

        //获取板线
        get_border_line(img_gray, bin_img, pred_lines, res);
        if (!res.border_line_status) {
            //板线检测失败
            return res;
        }

        //获取墨迹线
        get_ink_line(bin_img, pred_lines, res);
        if (!res.ink_line_status) {
            return res;
        }

        //计算方式待进一步修正
        double dist = calculateDistanceBetweenParallelLines(res.border_res[4], res.border_res[1] * h_ratio, res.ink_res[1] * h_ratio);
        res.dist = dist*this->scale;
        return res;
    }
    catch (...) {
        res.error_info = "直线检测算法出现运行异常!";
        std::cout<<res.error_info<<std::endl;
        return res;
    }

}





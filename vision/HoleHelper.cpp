#include"HoleHelper.h"
#include<opencv2/opencv.hpp>
#include<string>


bool isContourCircular(const std::vector<cv::Point>& contour, double tolerance = 0.3) {
    // 计算轮廓的面积和周长
    double area = cv::contourArea(contour);
    double perimeter = cv::arcLength(contour, true);

    // 计算圆度
    double circularity = (4 * CV_PI * area) / (perimeter * perimeter);

    // 判断圆度是否在允许的容差范围内
    return (circularity > 1 - tolerance && circularity < 1 + tolerance);
}



void draw_circle_result(cv::Mat& img, cv::Point center, int radius, cv::Scalar color, bool is_circle=true){


    //绘制圆形
    if (is_circle){
        cv::circle(img, center, radius, color,2);
        //绘制垂直直径
        cv::Point s_upper = cv::Point(center.x, center.y-radius);
        cv::Point e_upper =  cv::Point(center.x, center.y+radius);
        cv::line(img, s_upper, e_upper, color, 2);

        //绘制水平直径
        cv::Point s_lower = cv::Point(center.x-radius, center.y);
        cv::Point e_lower =  cv::Point(center.x+radius, center.y);
        cv::line(img, s_lower, e_lower, color, 2);
    }
    else{ // 绘制矩形
        cv::rectangle(img, cv::Point(center.x-radius, center.y-radius), cv::Point(center.x+radius, center.y+radius), color, 2);
    }


    //绘制圆心
    cv::circle(img, center, 2, color, -1);




}


cv::Point calculate_circle_deviation(cv::Point p1, cv::Point p2){
    /**
     * @brief 计算两个圆心坐标之间的位置偏差
     *
     * 给定两个圆心坐标，计算它们之间的位置偏差。
     *
     * @param p1 外圆圆心坐标
     * @param p2 内圆圆心坐标
     *
     * @return 返回计算得到的位置偏差
     */

    // p1 指的是外圆圆心， p2 值得是内圆圆心

    // 圆心坐标逆时针旋转90度
    cv::Point  p1_rotate = cv::Point(-p1.y, p1.x);
    cv::Point  p2_rotate = cv::Point(-p2.y, p2.x);

    // 计算旋转后x,y坐标的位置偏差
    return cv::Point(p2_rotate.x-p1_rotate.x, p2_rotate.y-p1_rotate.y);

}




/*********************** 功能函数  *********************************************/
std::pair<double, double> getContourRadius(const std::vector<cv::Point>& contour) {
    /**
 * @brief 获取轮廓半径
 *
 * 计算给定轮廓的最大半径和平均半径。
 *
 * @param contour 轮廓点集合
 *
 * @return 返回一个包含最大半径和平均半径的pair对象
 */
    // 计算轮廓的矩
    cv::Moments m = cv::moments(contour);

    // 计算轮廓的中心
    cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);

    // 计算轮廓的半径
    double radius = 0;
    double maxRadius = 0;
    int point_num = 0;
    for (const auto& point : contour) {
        double distance = std::sqrt(std::pow(abs(point.x - center.x), 2) + std::pow(abs(point.y - center.y), 2));
        // 排除NaN值的计算参与
        if (distance== distance){
            if (distance > maxRadius) {
                maxRadius = distance;
            }
            radius += distance;
            point_num++;
        }
    }
    if (maxRadius>0){
        radius /= point_num; // 平均半径
    }
    return std::pair<double, double>(maxRadius, radius);
}

std::vector<std::vector<cv::Point>>  get_contours(cv::Mat img){
    /**
 * @brief 获取轮廓
 *
 * 从给定的图像中获取轮廓，并返回轮廓的向量。
 *
 * @param img 输入图像
 *
 * @return 轮廓的向量
 */
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat bin_img;
    cv::threshold(img, bin_img, 0, 255, cv::THRESH_OTSU);
    cv::findContours(bin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    return contours;
}

cv::Mat get_circle_region(cv::Mat img, double min_radius, double max_radius){
    /**
 * @brief 获取圆形区域
 *
 * 根据给定的图像、最小半径和最大半径，获取圆形区域的掩码图像。
 *
 * @param img 输入图像
 * @param min_radius 最小半径
 * @param max_radius 最大半径
 *
 * @return 圆形区域的掩码图像
 */
    std::vector<std::vector<cv::Point>> contours;
    contours = get_contours(img);
    cv::Mat circle_mask(img.size(), CV_8UC1, cv::Scalar(0));

    // 遍历轮廓
    for (size_t i = 0; i < contours.size(); ++i) {
        // 计算轮廓面积
        std::pair<double, double> radius = getContourRadius(contours[i]);
        double maxRadius = radius.first;
        double avgRadius = radius.second;   // 平均半径相对不准确

        if (maxRadius > min_radius && maxRadius < max_radius) {
            cv::drawContours(circle_mask, std::vector<std::vector<cv::Point>>{contours[i]}, -1, cv::Scalar(255),
                             cv::FILLED);
        }
    }
    return circle_mask;
}

cv::Mat kmeans_img(cv::Mat img) {
    /**
  * @brief 对图像进行 K-means 聚类处理并获取外圆区域掩膜
  *
  * 根据输入的图像，对其进行 K-means 聚类处理，获取外圆区域掩膜。
  *
  * @param img 输入的图像
  *
  * @return 外圆区域掩膜
  */
    int channels = img.channels();
    cv::Mat img_gray;
    if (channels == 3) {
        cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    }
    else {
        img_gray = img.clone();
    }

    cv::Mat img_copy = img_gray.clone();
    cv::GaussianBlur(img_gray, img_gray, cv::Size(5, 5), 0, 0);

    // 将图像转换为浮点型并展平
    cv::Mat imageReshaped;
    cv::Mat data;
    data = img_gray.reshape(1, img_gray.rows * img_gray.cols);
    data.convertTo(data, CV_32F);
    int kmeans_num = 3;
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.2);
    int attempts = 10;
    int flags = cv::KMEANS_PP_CENTERS; // 初始化方法

    // 执行K-means聚类
    cv::Mat labels, centers;
    cv::kmeans(data, kmeans_num, labels, criteria, attempts, flags, centers);

    //解析聚类结果,获取可能的外圆像素区域
    cv::Mat mask = cv::Mat::ones(img.rows, img.cols, CV_8U);
    labels = labels.reshape(1, { img.rows, img.cols });
    labels = labels + 1;
    double max_value;
    double min_value;
    cv::Point max_val_index;
    cv::Point  min_val_index;
    cv::minMaxLoc(centers, &min_value, &max_value, &min_val_index, &max_val_index);

    //默认情况下,内圆区域中心值是最大的, 外圆区域中心值要么是最小的,要么是中间值
    int max_index = max_val_index.y + 1;
    int min_index = min_val_index.y + 1;
    int mid_index = 6 - max_index - min_index;

    int outer_circle_index;
    cv::Mat outer_points;

    cv::Mat mid_res;
    cv::Mat min_res;
    cv::findNonZero(labels == min_index, min_res);
    cv::findNonZero(labels == mid_index, mid_res);

    if (mid_res.rows > min_res.rows) {
        outer_circle_index = min_index;
        outer_points = min_res;
    }
    else {
        outer_circle_index = mid_index;
        outer_points = mid_res;
    }

    for (int i = 0; i < outer_points.rows; ++i) {
        mask.at<uchar>(outer_points.at<cv::Point>(i).y, outer_points.at<cv::Point>(i).x) = 255;
    }


    return mask;
}

cv::Mat remove_noise_or_padding(cv::Mat mask, int ksize = 5, std::string mode = "padding") {
    /**
 * @brief 去除噪声或填充像素间隙
 *
 * 根据指定的模式，对给定的掩码进行噪声去除或填充操作，并返回处理后的结果。
 *
 * @param mask 输入的掩码
 * @param ksize 结构元素的大小，默认为5
 * @param mode 模式，可选值为"padding"（填充）或"remove"（去除），默认为"padding"
 *
 * @return 处理后的掩码结果
 */
    cv::Mat res;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ksize, ksize));
    if (mode == "padding") {

        cv::morphologyEx(mask, res, cv::MORPH_CLOSE, element);
    }
    if (mode == "remove") {
        cv::morphologyEx(mask, res, cv::MORPH_OPEN, element);
    }
    return res;
}

cv::Mat remove_noise_by_contour_areas(cv::Mat img, double area_thr = 100) {
    /**
 * @brief 通过轮廓面积去除噪声
 *
 * 从给定的图像中通过轮廓面积去除噪声，并返回处理后的图像。
 *
 * @param img 输入图像
 * @param area_thr 轮廓面积阈值，默认为100
 *
 * @return 处理后的图像
 */
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat bin_img;
    cv::threshold(img, bin_img, 0, 255, cv::THRESH_OTSU);

    cv::findContours(bin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//    cv::Mat mask_new(img.rows, img.cols, CV_8U, cv::Scalar(0));
    cv::Mat mask_new = bin_img.clone();

    // 遍历轮廓
    for (size_t i = 0; i < contours.size(); ++i) {
        // 计算轮廓面积
        double area = cv::contourArea(contours[i]);
        std::pair<double,double>  radius = getContourRadius(contours[i]);
        double maxRadius = radius.first;
        double avgRadius = radius.second;

        // 如果面积小于10，则将轮廓内部填充为0
        if (area < area_thr) {
            /*cv::fillPoly(mask_new,);*/
            cv::drawContours(mask_new, std::vector<std::vector<cv::Point>>{contours[i]}, -1, cv::Scalar(255), cv::FILLED);
            //std::vector<cv::Point> fillContour(contours[i].begin(), contours[i].end());
            /*std::vector<vector<cv::Point>> temp;
            temp.push_back(contours[i]);
            cv::fillPoly(mask_new, temp, cv::Scalar(255),8,0);*/
        }
    }
    return mask_new;
}

double compute_radius_dist(cv::Point p1, cv::Point p2) {
    /**
 * @brief 计算两点之间的距离
 *
 * 根据给定的两个点 p1 和 p2，计算它们之间的距离。
 *
 * @param p1 第一个点
 * @param p2 第二个点
 *
 * @return 两点之间的距离
 */
    double x_dist = p1.x - p2.x;
    double y_dist = p1.y - p2.y;
    return sqrt(pow(x_dist, 2) + pow(y_dist, 2));
}

std::vector<std::vector<float>> get_circle_by_fit(cv::Mat mask){
    /**
  * @brief 根据掩码获取拟合圆
  *
  * 根据给定的掩码图像，找到其中的轮廓，并计算每个轮廓的最小外接圆。
  * 返回拟合圆的参数列表。
  *
  * @param mask 掩码图像
  *
  * @return 拟合圆的参数列表
  */
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<float>> res;
    for (const auto& contour: contours) {
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);
        res.push_back({center.x, center.y, radius});
    }
    return res;


}

cv::Mat get_grad(cv::Mat img){
    /**
 * @brief 获取梯度图像
 *
 * 根据输入的图像，计算其梯度，并返回梯度图像。
 *
 * @param img 输入图像
 *
 * @return 梯度图像
 */
    // 创建存储梯度的Mat对象
    cv::Mat grad_x, grad_y;
    cv::Mat gradient_img;
    cv::Mat img_gray;
    cv::Mat mask(img.size(), CV_8UC1, cv::Scalar(0));
    if (img.channels() == 3) {
        cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    }else{
        img_gray = img;
    }
    // 模糊图像，去除噪声
    cv::GaussianBlur(img_gray, img_gray, cv::Size(5, 5), 0);

    cv::Sobel(img_gray, grad_x, CV_32F, 1, 0, 3);  // 计算x方向的梯度
    cv::Sobel(img_gray, grad_y, CV_32F, 0, 1, 3);  // 计算y方向的梯度

    cv::convertScaleAbs(grad_x, grad_x);
    cv::convertScaleAbs(grad_y, grad_y);

    cv::addWeighted(grad_x, 0.5, grad_y,0.5, 0, gradient_img);
//    cv::imwrite("E:\\znzz\\data\\circles\\results\\badcases\\grad.png", gradient_img);
    gradient_img.convertTo(gradient_img, CV_8UC1);

    for (int y = 0; y < gradient_img.rows; ++y) {
        for (int x = 0; x < gradient_img.cols; ++x) {
            if (gradient_img.at<uchar>(y, x) > 15) { //梯度根据经验进行设定
                mask.at<uchar>(y, x) = 255;
            }
        }
    }
    return mask;
}

#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>



/***********************圆孔检测类方法实现**************************************************************/

HoleHelper::HoleHelper() {
    resize_h = 578;
    resize_w = 320;   // Maintain the image aspect ratio at 1.8:1
    offset = 80;
    // 获取内圆相关预定参数
    min_inner_circle_radius = 35;
    max_inner_circle_radius = 50;
    min_inner_pixels_threshold = 190;

    // 获取外圆相关预定参数
    min_outer_circle_radius = 60;
    max_outer_circle_radius = 120;

}

std::vector<float> HoleHelper::get_inner_circle_v2(cv::Mat img){
    /**
   * @brief 获取内圆信息（v2版本）
   *
   * 根据给定的图像，通过一系列图像处理步骤获取内圆的信息，包括圆心坐标和半径。
   *
   * @param img 输入的图像
   *
   * @return 包含圆心横坐标、圆心纵坐标和半径的向量
   */

    int img_width = img.cols;
    int img_height = img.rows;

    int start_y = static_cast<int>( img_height *0.2);
    int end_y = static_cast<int>( img_height *0.7);

    int start_x = static_cast<int>( img_width *0.2);
    int end_x = static_cast<int>( img_width *0.7);


    cv::Rect cropRect(start_x, start_y, end_x-start_x, end_y- start_y);
    cv::Mat croppedImage = img(cropRect);
    cv::Mat mask = kmeans_img(croppedImage);
    mask = 255-mask;


    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<float> circle_info;
    // 遍历轮廓
    for (size_t i = 0; i < contours.size(); ++i) {
        // 计算轮廓面积
        double area = cv::contourArea(contours[i]);
        // 如果面积小于10，则将轮廓内部填充为0
        if (area > min_inner_circle_radius*min_inner_circle_radius*0.4 && area < max_inner_circle_radius*max_inner_circle_radius) {
            cv::RotatedRect fittedCircle = cv::fitEllipse(contours[i]);
            // 获取拟合圆的中心点和半径
            cv::Point2f center(fittedCircle.center);
            float radius = fittedCircle.size.width / 2;
            circle_info = { start_x+center.x, start_y+center.y,radius };
            break;
        }
    }
    return circle_info;
}


void HoleHelper::detect_inner_circle(cv::Mat img, HoleRes& res) {
    /**
 * @brief 检测内圆位置信息以及局部区域
 *
 * 根据给定的图像，检测内圆的位置信息以及局部区域，并返回一个包含位置和半径信息的pair。
 *
 * @param img 输入图像
 *
 * @return 包含位置和半径信息的pair
 */
    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(img_gray, img_gray, cv::Size(5, 5), 0, 0);
    cv::Mat bin_img, hist_img;
    cv::equalizeHist(img_gray, hist_img);
    cv::threshold(hist_img, bin_img, 0, 255,  cv::THRESH_BINARY+cv::THRESH_OTSU);

    //1.0对二进制图像进行预处理  先移除噪声,在膨胀
    bin_img = remove_noise_or_padding(bin_img, 5, "remove");
    bin_img = remove_noise_or_padding(bin_img, 5, "padding");

    //2.0轮廓检测
    std::vector<std::vector<cv::Point>> contours = get_contours(bin_img);
    std::vector<float> circle_info;
    //3.0筛选合适的轮廓
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        cv::Rect boundingRect = cv::boundingRect(contours[i]);
        int x1, y1, w, h;

        //获取矩形框
        x1 = boundingRect.x;
        y1 = boundingRect.y;
        w = boundingRect.width;
        h = boundingRect.height;
        if (abs(w - h) > 10) {
            continue;
        }
//        if(!isContourCircular(contours[i])){break;}

        // 面积大小约束,像素值约束
        if (area > min_inner_circle_radius*min_inner_circle_radius*0.4 && area < max_inner_circle_radius*max_inner_circle_radius) {
            cv::Rect cropRect(x1, y1, w, h);
            cv::Mat crop_img = bin_img(cropRect);
            cv::Scalar mean_value;
            mean_value = cv::mean(crop_img);
            if (mean_value[0] > min_inner_pixels_threshold) {
                cv::RotatedRect fittedCircle = cv::fitEllipse(contours[i]);
                // 获取拟合圆的中心点和半径
                cv::Point2f center(fittedCircle.center);
                float radius = fittedCircle.size.width / 2;
                circle_info = { center.x, center.y,radius };
                break;
            }
        }
    }

    // 执行候补方案(通过K-means获取内圆坐标)
    if (circle_info.size() == 0) {
         circle_info = get_inner_circle_v2(img);
    }


    //4.0结果校验处理
    if (circle_info.size() > 0) {
        draw_circle_result(img, cv::Point2f(circle_info[0], circle_info[1]), static_cast<int>(circle_info[2]), cv::Scalar(0, 255, 0),false);
        res.msg = "Inner Circle Detection Successed";
        res.inner_status = true;
        res.outer_status = false;
        res.status = false;
        res.img_drawed = img;
        res.inner_circle_res = circle_info;
    }
    else {
        res.msg = "Inner Circle Detection Failed";
        res.img_drawed = img;
        res.inner_status = false;
        res.outer_status = false;
        res.status = false;
        res.img_drawed = img;
    }
}

std::vector<int> HoleHelper::kmeans_post_processing(cv::Mat mask) {
    /**
   * @brief 对掩膜进行K-means后处理
   *
   * 使用K-means算法对给定的掩膜进行后处理，并返回检测到的外圆的参数列表。
   *
   * @param mask 掩膜图像
   *
   * @return 检测到的外圆的参数列表
   */
    std::vector<int> res;
    cv::Mat bin_img;
    cv::threshold(mask, bin_img, 0, 255, cv::THRESH_OTSU);

    // 利用面积去除存在粘连的情况
    cv::Mat mask_new = get_circle_region(bin_img, 30, 120);


    // 获取轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_new, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 遍历轮廓
    for (size_t i = 0; i < contours.size(); i++) {
        cv::RotatedRect fittedCircle = cv::fitEllipse(contours[i]);
        // 获取拟合圆的中心点和半径
        cv::Point2f center(fittedCircle.center);
        float radius = fittedCircle.size.width / 2;
        double area = 3.14*pow(radius, 2);
        if (area > min_outer_circle_radius*min_outer_circle_radius && area < max_outer_circle_radius * max_outer_circle_radius) {
            res.push_back(center.x);
            res.push_back(center.y);
            res.push_back(static_cast<int>(radius));
            return res;
        }

    }
    return res;
}

std::vector<int> HoleHelper::detect_outer_circle_by_grad(cv::Mat img) {
/**
 * @brief 检测外圆
 *
 * 通过梯度检测图像中的外圆，并返回检测到的外圆的参数。
 *
 * @param img 输入图像
 *
 * @return 检测到的外圆的参数列表
 */
    std::vector<int> res;
    cv::Mat  grad_mask = get_grad(img);

    grad_mask = remove_noise_or_padding(grad_mask, 3,"padding");  // 边缘断联区域填充
    grad_mask  = get_circle_region(grad_mask, 50, 120);
//    cv::imshow("grad_mask", grad_mask);

    //进一步收缩
    cv::Mat eroded_image, dliate_image;
    // 进行腐蚀操作
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(3, 3)); // 3x3的矩形核
    cv::erode(grad_mask, eroded_image, element, cv::Point(-1, -1), 1);
    eroded_image = remove_noise_or_padding(eroded_image, 3,"remove");
    cv::dilate(eroded_image, dliate_image, element, cv::Point(-1, -1), 1);


    //外圆拟合
    std::vector<std::vector<float>>  circles = get_circle_by_fit(grad_mask);
    //遍历轮廓
    for (size_t i = 0; i < circles.size(); i++) {
        float radius = circles[i][2];
        double area = 3.14 * pow(radius, 2);
        if (area > min_outer_circle_radius*min_outer_circle_radius && area < max_outer_circle_radius * max_outer_circle_radius) {
            res.push_back(circles[i][0]);
            res.push_back(circles[i][1]);
            res.push_back(static_cast<int>(radius));
            return res;
        }
    }
    return res;
}

void HoleHelper::detect_outer_circle(cv::Mat img, HoleRes& res) {
    /**
    * @brief 检测外圆
    *
    * 主要使用kmeans算法对圆环进行检测，并计算外圆的参数。
    *
    * @param img 输入的图像
    * @param res 可视化结果
    */

    // 基于kmeans算法对圆环进行检测
    cv::Mat kemans_mask = kmeans_img(img);
    std::vector<int> outer_circle = kmeans_post_processing(kemans_mask);

    std::vector<int> roi_pos = res.rect_roi;
    if (outer_circle.size()<0) {
        int x = outer_circle[0] + roi_pos[0];
        int y = outer_circle[1] + roi_pos[1];
        draw_circle_result(res.img_drawed, cv::Point2f(x, y), outer_circle[2], cv::Scalar(0, 0, 255));

        // 计算圆心之间的坐标偏差
        cv::Point offset = calculate_circle_deviation(cv::Point(x, y),cv::Point(res.inner_circle_res[0],res.inner_circle_res[1]));
        res.offset_x = (offset.x)*this->scale;
        res.offset_y = (offset.y)*this->scale;
        cv::flip(res.img_drawed, res.img_drawed, -1); // 图像逆时针翻转
        res.msg = "Outer Circle Detection Successed";
        res.status = true;
        res.outer_status = true;

    }else{
        //  兜底策略, 当Kmeans检测没有检测结果时, 直接使用梯度检测
        std::vector<int> outer_circle = detect_outer_circle_by_grad(img);
        if (outer_circle.size() > 0) {
            int x = outer_circle[0] + roi_pos[0];
            int y = outer_circle[1] + roi_pos[1];
            draw_circle_result(res.img_drawed, cv::Point2f(x, y), outer_circle[2], cv::Scalar(0, 0, 255));

            //计算距离
            cv::Point offset = calculate_circle_deviation(cv::Point(x, y),cv::Point(res.inner_circle_res[0],res.inner_circle_res[1]));
            res.offset_x = (offset.x);
            res.offset_y = (offset.y);
            res.msg = "Outer Circle Detection Successed";
            res.status = true;
            res.outer_status = true;
        }
        else{
            res.msg = "Outer Circle Detection Failed";
            res.status = false;
            res.outer_status = false;
        }
    }
}

HoleRes HoleHelper::get_holes_distance(cv::Mat img) {
    /**
  * @brief 获取孔洞的距离
  *
  * 根据给定的图像，计算孔洞的距离，并返回结果。
  *
  * @param img 输入的图像
  *
  * @return 包含孔洞距离信息的 VisRes 对象
  */
    HoleRes res;
    if (img.empty()) {
        res.msg = "Invalid input image";
        return res;
    }

    // image processing
    cv::Mat img_578;
    cv::resize(img, img_578, cv::Size(this->resize_w, this->resize_h));

    int origin_w = img.cols;
    int origin_h = img.rows;
    float w_ratio = static_cast<float>(origin_w) / img_578.cols;
    float h_ratio = static_cast<float>(origin_h) / img_578.rows;
    res.img_drawed = img_578;

    try{
        // obtain the position of the inner circle
        detect_inner_circle(img_578, res);

        if (res.inner_status){ //
            std::vector<float> circle_info = res.inner_circle_res;
            int center_x = static_cast<int>(circle_info[0] );
            int center_y = static_cast<int>(circle_info[1]);

            // 裁剪可能的圆环区域
            int x11 = center_x - this->offset > 0 ? center_x -  this->offset : 0;
            int y11 = center_y - this->offset > 0 ? center_y - this->offset : 0;
            int x13 = center_x + this->offset < img_578.cols ? center_x + this->offset : img_578.cols - 1;
            int y13 = center_y + this->offset < img_578.rows ? center_y + this->offset : img_578.rows - 1;
            cv::Rect cropRect(x11, y11, x13 - x11, y13 - y11);
            cv::Mat img_roi = img_578(cropRect);
            res.rect_roi = {x11,y11,x13,y13};
            // obtain the position of the outer circle
            detect_outer_circle(img_roi, res);

            if (res.outer_status){
                res.offset_x = res.offset_x*w_ratio*this->scale;
                res.offset_y = res.offset_y*h_ratio*this->scale;
            }
        }
        return res;
    }
    catch(...){
        res.msg = "runtime error";
        return res;

    }
}




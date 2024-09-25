//
// Created by csh_i on 2024/6/12.
//

#include "LineHelper.h"
#define PI acos(-1)


//double calculateDistanceBetweenParallelLines(double slope, double intercept1, double intercept2) {
//    double denominator = std::sqrt(1 + slope * slope);
//    return std::abs(intercept2 - intercept1) / denominator;
//}

LineHelper::LineHelper() {
    this->resize_h = 512;
    this->resize_w = 512;
    // 有效线段分布区间
    this->min_pos_valid = 30;
    this->max_pos_valid = 512 - 50;
    // 两个线段之间的距离范围
    this->min_dist_lines = 50;
    this->max_dist_lines = 250;
    // 角度范围
    this->max_angle_degrees = 7;
    this->min_len_seg = static_cast<unsigned>(512 * 0.1);
}

// 计算两条线段是否重合或接近重合
bool isOverlapping(const std::vector<float>& a, const std::vector<float>& b, double tolerance = 3) {
    // 这里使用简单的逻辑来判断线段是否重合
    // 你可以根据需要添加更复杂的逻辑，比如检查线段是否足够接近
    return (std::abs(a[0] - b[0]) < tolerance && std::abs(a[1] - b[1]) < tolerance &&
            std::abs(a[2] - b[2]) < tolerance && std::abs(a[3] - b[3]) < tolerance);
}

// 过滤掉重合的线段
std::vector<std::vector<float>> filterOverlappingSegments(const std::vector<std::vector<float>>& segments) {
    std::vector<std::vector<float>> filteredSegments;
    for (const auto& segment : segments) {
        bool is_overlapping = false;
        for (const auto& existingSegment : filteredSegments) {
            if (isOverlapping(segment, existingSegment)) {
                is_overlapping = true;
                break;
            }
        }
        if (!is_overlapping) {
            filteredSegments.push_back(segment);
        }
    }
    return filteredSegments;
}


void LineHelper::img_processing(cv::Mat img, cv::Mat &img_gray, cv::Mat &bin_img) {
    return ;
}

LineRes LineHelper::compute_lines_distance(cv::Mat img, bool is_upper) {

    //数据合法性校验
    LineRes res;
    memset(&res, 0, sizeof(LineRes));
    if (img.empty()) {
        res.error_info = "Input data can not be null";
        return res;
    }
    // 图像旋转, 主要针对9号和10号相机
    if (img.rows > img.cols) {
        img = rotateImage(img);
    }

    float origin_h = img.rows;
    float origin_w = img.cols;
    float h_ratio = origin_h / this->resize_h;
    float w_ratio = origin_w / this->resize_w;

    //图像预处理
    cv::Mat img_gray, bin_img, img_512;
    cv::resize(img, img_512, cv::Size(this->resize_w,this->resize_h));
    cv::cvtColor(img_512, img_gray, cv::COLOR_BGR2GRAY);
    cv::threshold(img_gray, bin_img, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
    res.img_drawed = img_512;

    // mlsd  算法检测所有可能的线段
    std::vector<std::vector<float>>  all_lines = get_all_possible_lines(img_512);

    // 对检测到的直线进行排序 ;
    if(is_upper){
        sort_lines(all_lines, true) ;   // 升序排序
    }else{
        sort_lines(all_lines, false) ;  // 降序排序
    }

    if(all_lines.empty()){
        res.error_info = "Invalid input image";
        return res;
    }

    // 获取板线段
    std::vector<float> line_border_res = this->get_border_line(img_512,all_lines);

    if(line_border_res.empty()){
        res.error_info = "Board line detection failed";
        return res;
    }else{
        double x1 = line_border_res[0]*w_ratio,  y1 =line_border_res[1]*h_ratio, x2 = line_border_res[2]*w_ratio ,y2 = line_border_res[3]*h_ratio;
        float scope = (y1-y2)/(x1-x2);
        res.border_res = {line_border_res[0]*w_ratio,line_border_res[1]*h_ratio,line_border_res[2]*w_ratio,line_border_res[3]*h_ratio, scope};
        res.border_line_status = true;
        cv::line(img_512, cv::Point2f(line_border_res[0], line_border_res[1]), cv::Point2f(line_border_res[2], line_border_res[3]), cv::Scalar(255,0,255),2);
        res.img_drawed = img_512;
    }

    // 获取墨迹线段
    std::vector<float> line_ink_res = this->get_ink_line(bin_img, line_border_res,all_lines);
    if(line_ink_res.empty()){
        res.error_info = "Ink line detection failed";
        return res;
    }else{
        res.ink_line_status = true;
        double x1 = line_ink_res[0]*w_ratio,  y1 =line_ink_res[1]*h_ratio, x2 = line_ink_res[2]*w_ratio ,y2 = line_ink_res[3]*h_ratio;
        float scope1 = (y1-y2)/(x1-x2);
        res.ink_res = {line_ink_res[0]*w_ratio,line_ink_res[1]*h_ratio,line_ink_res[2]*w_ratio,line_ink_res[3]*h_ratio, scope1};
        cv::line(img_512, cv::Point2f(line_ink_res[0], line_ink_res[1]), cv::Point2f(line_ink_res[2], line_ink_res[3]), cv::Scalar(0,255,255),2);
        res.img_drawed = img_512;
    }

    if(line_ink_res.size()>0 && line_border_res.size()>0){
        // 计算墨迹线与板线的距离
        double distance = calculateDistanceBetweenParallelLines(line_border_res[4], line_ink_res[1]*h_ratio, line_border_res[1]*h_ratio);
        double distance_1 = computePointToLineDistance(res.border_res, res.ink_res);
        res.dist = distance;
        res.status = true;
    }
    return res;
}




// 函数用于计算两条平行直线之间的距离
std::vector<std::vector<float>> LineHelper::get_all_possible_lines(cv::Mat img) {

    // MLSD算法检测所有可能的线段
    std::vector<std::vector<float>> line_res;
    std::vector<std::vector<float>>  lines_mlsd = this->line_detector.detect(img);
    cv::Mat line_mask = cv::Mat::zeros(img.size(), CV_8UC1);
    for(auto item: lines_mlsd){
        cv::line(line_mask, cv::Point(item[0], item[1]), cv::Point(item[2], item[3]), cv::Scalar(255), 1);
    }
    std::vector<std::vector<float>> lines = get_line_by_lsd(line_mask);
    sort_lines(lines);
    std::vector<std::vector<float>> lines_new = linesAggregation(lines);

    //对检测出的直线进一步过滤
    for (auto &line : lines_new) {
        // 1.0 边线分布位置特点过滤
        if (line[1] < this->min_pos_valid || line[3] > max_pos_valid) { continue;}
        // 2.0 过滤掉竖直为竖直方向的直线
        if(line[1]<0 | line[3]<0|line[1]>img.cols|line[3]>img.cols){
            continue;
        }
        line_res.push_back(std::vector<float>{line[0],line[1],line[2],line[3],line[4]});
    }
    return line_res;
}

std::vector<float> LineHelper::get_border_line(cv::Mat img, std::vector<std::vector<float>> lines) {

    // 率先通过mlsd 算法检测出板线
    std::vector<float> border_line_res;
    cv::Mat bin_img;
    if(img.channels() == 3){
        cv::Mat img_gray;
        cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
        cv::threshold(img_gray, bin_img, 0, 255, cv::THRESH_OTSU);
    }else{
        bin_img = img;
    }


    std::vector<float> border_line_mlsd;
    for(auto &line_item : lines){
        cv::Vec4f line = { round(line_item[0]),
                           round(line_item[1]),
                           round(line_item[2]),
                           round(line_item[3])
        };
        if(check_is_border_line(bin_img, line, 0.65)){
            border_line_mlsd = line_item;
            std::cout<<"border scope:"<<line_item[4]<<std::endl;
            break;
        }
    }

    //  传统算法进一步线段矫正
    std::vector<float> border_line_lsd;
//    border_line_lsd = get_border_line_by_lsd(bin_img);    //LSD算法检测线段较为耗时, 默认状态下没有开启
    if(border_line_mlsd.size()>0){
        if (border_line_lsd.size()>0){
            // 计算两个线段的距离
            float dist = calculateDistanceBetweenParallelLines(border_line_mlsd[4], border_line_mlsd[1], border_line_lsd[1]);
            if(dist<5){
                border_line_res = border_line_lsd;
                std::cout << "mlsd 算法检测的板线与 lsd 算法检测的板线距离为："<< dist << std::endl;
            }
            else {
                border_line_res = border_line_mlsd;
            }
        }else{
            border_line_res = border_line_mlsd;
        }
    }else{
        if(border_line_lsd.size()>0){
            border_line_res = border_line_lsd;
        }
    }
    return border_line_res;
}

void LineHelper::imageLocalBinarization(cv::Mat img, const std::vector<std::vector<float>> &lines, cv::Mat &bin_img) {
    /**
 * @brief 对图像进行局部二值化处理
 *
 * 根据给定的线条信息，对图像进行局部二值化处理。首先将图像转换为灰度图像，
 * 然后使用 Otsu 阈值法进行全局二值化。接着，遍历线条信息，对每条线条的中心区域
 * 进行局部二值化处理，并将结果应用到输出图像中。
 *
 * @param img 输入图像
 * @param lines 线条信息，以二维浮点向量形式表示
 * @param bin_img 输出二值化图像
 */

    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    cv::threshold(img_gray, bin_img, 0, 255, cv::THRESH_OTSU);

    int offset = 50;
    for(const auto &line : lines){
        cv::Vec4f line_item = {round(line[0]), round(line[1]), round(line[2]), round(line[3])};
        cv::Point2f center = {
                (line_item[0] + line_item[2]) / 2,
                (line_item[1] + line_item[3]) / 2
        };
        cv::Point2f left_point = {
                center.x - offset/2,
                center.y - offset/2
        };

        cv::Point2f right_point = {
                center.x + offset/2,
                center.y + offset/2
        };
        if(left_point.x>0 && left_point.y>0
           &&right_point.x<img.cols
           && right_point.y<img.rows){
            cv::rectangle(img, left_point, right_point, cv::Scalar(0,255,0), 1);
            cv::Rect roi(center.x, center.y, offset, offset);
            cv::Mat roi_img = img_gray(roi);
            cv::Mat roi_bin;
            cv::threshold(roi_img, roi_bin, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
            bin_img(roi) = roi_bin;

        }
    }
}

std::vector<float> LineHelper::get_border_line_by_lsd(cv::Mat bin_img) {

    std::vector<float> border_line;
    //基于LSD算法获取线检测点
    cv::Ptr<cv::LineSegmentDetector> detector = createLineSegmentDetector(cv::LSD_REFINE_STD);
    std::vector<cv::Vec4f> lines;
    detector->detect(bin_img, lines);
    // 筛选板线点
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
        if ((distance > this->resize_w*0.2) && check_is_boundary(bin_img, line)) { //满足直线的长度要求，且直线上下区域像素相似度低
            points.push_back(cv::Point2f(line[0], line[1]));
            points.push_back(cv::Point2f(line[2], line[3]));
            break;
        }
    }
    if(points.size()>0){
        border_line = fiting_line(points);
    }
    return border_line;
}

std::vector<float> LineHelper::get_ink_line(cv::Mat bin_img,  std::vector<float> &border_line,   std::vector<std::vector<float>>& pred_lines) {

    std::vector<float> border_line_info =border_line;
    float slope = border_line_info[4]; //板线斜率
    std::vector<float> ink_line;

    for (const auto& row : pred_lines) {

        float slope_ink = row[4];
        float line_dist = abs(border_line_info[1] - row[1]);

        // 计算两条之间角度偏差
        float slope_offset = slope - slope_ink;
        float angle = atan(slope_offset);
        float angle_degrees = abs(angle*(180.0 / PI));

        // 线间距、角度偏差判断
        if (line_dist>this->min_dist_lines &&line_dist<this->max_dist_lines&&angle_degrees < this->max_angle_degrees) {
            cv::Vec4f line = { round(row[0]),
                               round(row[1]),
                               round(row[2]),
                               round(row[3])
            };
            // 判断是否是墨迹线
            if(!check_is_border_line(bin_img, line,0.7)){
                ink_line = row;
                break;
            }
        }
    }
    return ink_line;
}
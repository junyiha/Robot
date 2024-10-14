//
// Created by csh_i on 2024/7/5.
//

#include "LineDetector.h"


LineResult LineDetector::getLineDistance(cv::Mat img) {

    LineResult res;
    memset(&res, 0, sizeof(LineResult));
    if (img.empty()) {
        res.errorInfo = "Input data can not be null";
        return res;
    }

    //1. �Ҷ�ͼ��תRGBͼ��
    if(img.channels() == 1){
        cv::cvtColor(img, img, cv::COLOR_GRAY2RGB);
    }

    float origin_h = img.rows;
    float origin_w = img.cols;
    float h_ratio = origin_h / this->resize_h;
    float w_ratio = origin_w / this->resize_w;

    //1.0 ͼ��Ԥ����
    cv::Mat img_gray, bin_img, img_512, img_256;
    cv::Rect binImgRoi(0,256, 512, 256) ;
    cv::resize(img, img_512, cv::Size(this->resize_w,this->resize_h));
    cv::cvtColor(img_512, img_gray, cv::COLOR_RGB2GRAY);
    img_256 = img_gray(binImgRoi);
    cv::threshold(img_256, bin_img, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
    res.imgDrawed = img_512;

//    cv::imshow("bin_img", bin_img);
//    cv::waitKey();

    // mlsd  �㷨������п��ܵ��߶�
    std::vector<MLine> all_lines =  getAllPossibleLines(img_512);

//    for(auto &line:all_lines){
//        cv::line(img_512, cv::Point2f(line.x1,line.y1), cv::Point2f(line.x2,line.y2), cv::Scalar(255,0,255),2 );
//    }
//    cv::imshow("all line", img_512);
//    cv::waitKey();


    // 2.0 ��ȡ�ο���
    MLine referenceLine = getReferenceLineByHKCU60(bin_img, all_lines);
//    MLine referenceLine = getReferenceLineByContours(bin_img);

    if(referenceLine.x1!=referenceLine.x2){
        res.refResult = referenceLine;
        res.referLineStatus = true;
        cv::line(res.imgDrawed, cv::Point2f(referenceLine.x1, referenceLine.y1),
                 cv::Point2f(referenceLine.x2, referenceLine.y2), cv::Scalar(0, 0, 255), 1);
    }else{
        res.errorInfo = "Can not get reference line";
        return res;
    }

//    cv::imshow("ref", res.imgDrawed);
//    cv::waitKey();

    // 3.0 ��ȡīˮ��
    MLine inkLine = getInkLine(img_gray, all_lines, referenceLine);
    if(inkLine.x1!=inkLine.x2){
        res.inkResult = inkLine;
        res.inkLineStatus = true;
        cv::line(res.imgDrawed, cv::Point2f(inkLine.x1, inkLine.y1),
                 cv::Point2f(inkLine.x2, inkLine.y2), cv::Scalar(0, 255, 0), 1);
        res.status = true;
        double distance = calculateDistanceBetweenParallelLines(referenceLine.slope, inkLine.y1*h_ratio, referenceLine.y1*h_ratio);
        res.lineDist = distance;
    }else{
        res.errorInfo = "Get line distance failed";
    }
    return res;
}

MLine LineDetector::getInkLine(cv::Mat img, std::vector<MLine> lines,  MLine refLine) {

    // ��ȡ�������Ͽ��ܵ�ī����������
    double min_Y =  std::min(refLine.y1, refLine.y2);
    cv::Rect inkLineRoi(0,0,512, min_Y);
    cv::Mat inkLineImg = img(inkLineRoi);
    cv::Mat inkBinImg;
    cv::threshold(inkLineImg, inkBinImg, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);

    MLine lineRes;
    memset(&lineRes, 0, sizeof(MLine));
    std::vector<MLine> possInkLines;
    for(const auto &line:lines) {
         cv::Vec4f line_ = {line.x1, line.y1, line.x2, line.y2};
         if(line.y1>min_Y){ continue;}  // 过滤参考板以下的直线

         // ī����ͨ���ڰ���֮��
         if (!check_is_border_line(inkBinImg, line_)) {
             std::vector<cv::Point2f> points;
             points.push_back(cv::Point2f(line.x1, line.y1));
             points.push_back(cv::Point2f(line.x2, line.y2));
             std::vector<float> line_fited = fiting_line(points);
             double angle = atan(line_fited[4])*180/CV_PI;
             if(fabs(angle)>45){  // б�ʴ���30��,��Ϊ����īˮ��
                 continue;
             }
             MLine line_;
             line_.x1 = line_fited[0];
             line_.y1 = line_fited[1];
             line_.x2 = line_fited[2];
             line_.y2 = line_fited[3];
             line_.slope = line_fited[4];
             possInkLines.push_back(line_);
         }
     }

    // 筛选距离参考板最近的墨迹线
    double minDist = 1000;
    double minIndex = -1;
    if(possInkLines.size()>0){
        for(int i=0; i<possInkLines.size();i++){
            double minY_P = std::min(possInkLines[i].y1, possInkLines[i].y2);
            double minLen = min_Y - minY_P;
            if(minLen<minDist && minLen>5){
                minDist = minLen;
                minIndex = i;
            }
        }
    }

    if(minDist<1000){
        lineRes = possInkLines[minIndex];
    }
    return lineRes;
}

MLine LineDetector::getReferenceLineLSD(cv::Mat img, cv::Mat bin_img){

    MLine lineRes;
    memset(&lineRes, 0, sizeof(MLine));

    //����LSD�㷨��ȡ�߼���
    cv::Ptr<cv::LineSegmentDetector> detector = createLineSegmentDetector(cv::LSD_REFINE_STD);
    std::vector<cv::Vec4f> lines;
    detector->detect(bin_img, lines);

    for(int i=0; i<lines.size();i++) {
        cv::Vec4f line = lines[i];
        cv::line(img, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow("test", img);
    cv::waitKey();

    // ���ͼ����б�Ե����


    // ɸѡ���ߵ�
    std::vector<cv::Point2f> points;
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4f line = lines[i];

        float distance = calculatePointsDistance(cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]));
        bool isBoundary = check_is_border_line(bin_img, line);
        if ((distance > this->resize_w*0.1) && isBoundary) { //����ֱ�ߵĳ���Ҫ����ֱ�����������������ƶȵ�
            points.push_back(cv::Point2f(line[0], line[1]));
            points.push_back(cv::Point2f(line[2], line[3]));
            break;
        }
    }
    if(points.size()>0){
        std::vector<float>  border_line = fiting_line(points);
        lineRes.x1 = border_line[0];
        lineRes.y1 = border_line[1];
        lineRes.x2 = border_line[2];
        lineRes.y2 = border_line[3];
        lineRes.slope = border_line[4];
    }
    return lineRes;
}

MLine LineDetector::getReferenceLine(cv::Mat img, std::vector<MLine> lines) {

    // �ָ��߶�
    MLine refLine_lsd, refLine_model, refLine;
    memset(&refLine_lsd, 0, sizeof(MLine));
    memset(&refLine_model, 0, sizeof(MLine));
    memset(&refLine, 0, sizeof(MLine));

    cv::Mat img_gray, bin_img;
    if(img.channels() == 3){
        cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    }else{
        img_gray = img;
    }
    cv::threshold(img_gray, bin_img, 0, 255, cv::THRESH_OTSU);
//    cv::imshow("bin_img",bin_img);
//    cv::waitKey();

    // ��ͳͼ�����㷨��ȡ���
    refLine_lsd = getReferenceLineLSD(img, bin_img);

    // ģ���㷨��ȡ���
    for (auto &line : lines) {
        cv::Vec4f line_ = { line.x1,line.y1, line.x2,line.y2 };
        if(abs(line.x1 - line.x2)<50){
            continue;
        }  // �ο���̫�̣����������
        if(check_is_border_line(bin_img, line_)){
            std::vector<cv::Point2f> points;
            points.push_back( cv::Point2f(line.x1, line.y1));
            points.push_back( cv::Point2f(line.x2, line.y2));
            std::vector<float> line_fited = fiting_line(points);
            if(line_fited[1]>512 || line_fited[3]>512){ continue;}  // ��Ϻ��ֱ�߶˵�����Խ��
            refLine_model = {line_fited[0], line_fited[1], line_fited[2], line_fited[3], line_fited[4]};
            break;
        }
    }
    if(refLine_lsd.x1 == refLine_lsd.x2){
        refLine_lsd = refLine_model;
    }

//    cv::line(img, cv::Point2f(refLine_lsd.x1, refLine_lsd.y1),cv::Point2f(refLine_lsd.x2, refLine_lsd.y2), cv::Scalar(255,0,0),2);
//    cv::imshow("test",img);
//    cv::waitKey();

    // Ѱ�Ҿ�������������
//    double minDist = 1000;
//    int minDistIndex = 0;
//    double minYLsd = std::min(refLine_lsd.y1, refLine_lsd.y2);
//    for(int i =0; i<lines.size(); i++){
//        if(abs(lines[i].x1 - lines[i].x2)<50){  // ���˽϶̵�ֱ��
//            continue;
//        }
//        double minY = std::min(lines[i].y1, lines[i].y2);
//        if(minY>minYLsd){
//            continue;
//        }
//        double lineDist = minYLsd-minY;
//        if(lineDist<minDist && lineDist>20 ){
//            minDist = (minYLsd-minY);
//            minDistIndex = i;
//        }
//    }
//    if(minDist<200){ // ������
//        refLine = lines[minDistIndex];
//    }

//    cv::line(img, cv::Point2f(refLine.x1, refLine.y1),cv::Point2f(refLine.x2, refLine.y2), cv::Scalar(255,0,0),2);
//    cv::imshow("test2",img);
//    cv::waitKey();
    return refLine_lsd;

}

std::vector<MLine> LineDetector::getAllPossibleLines(cv::Mat img) {

    // MLSD�㷨������п��ܵ��߶�
    std::vector<MLine> line_res;
    std::vector<std::vector<float>>  lines_mlsd = this->mlsd.detect(img);
    cv::Mat line_mask = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Mat line_mask_ = cv::Mat::zeros(img.size(), CV_8UC1);
    for(auto item: lines_mlsd){
        cv::line(line_mask, cv::Point(item[0], item[1]), cv::Point(item[2], item[3]), cv::Scalar(255), 1);
    }

    imageOpenedOrClosed(line_mask);
//    cv::imshow("line_mask", line_mask);
//    cv::waitKey();


    std::vector<std::vector<float>> lines = get_line_by_lsd(line_mask);
    sort_lines(lines);
//    std::vector<std::vector<float>> lines_new = linesAggregation(lines);

//    for(auto item: lines){
//        cv::line(line_mask_, cv::Point(item[0], item[1]), cv::Point(item[2], item[3]), cv::Scalar(255),
//                1);
//    }
//    cv::imshow("line_mask", line_mask_);
//    cv::waitKey();



    //�Լ�����ֱ�߽�һ������
    for (auto &line : lines) {
        // 1.0 ���߷ֲ�λ���ص����
        if (line[1] < this->min_pos_valid || line[3] > max_pos_valid) { continue;}
        // 2.0 ���˵�Խ���ֱ��
        if(line[1]<0 | line[3]<0|line[1]>img.cols|line[3]>img.cols){
            continue;
        }
        // 3.0 ���˵���ֱ�����ֱ��
        double dx = line[2] - line[0];
        double dy = line[3] - line[1];
        double angle = atan2(dy, dx) * 180 / CV_PI;
        if(fabs(angle)>60){
            continue;
        }
        MLine line_true = {line[0],line[1],line[2],line[3],0};
        line_res.push_back(line_true);
    }
    return line_res;
}

LineDetector::LineDetector() {
    this->resize_h = 512;
    this->resize_w = 512;
    // ��Ч�߶ηֲ�����
    this->min_pos_valid = 10;
    this->max_pos_valid = 512 - 10;
    // �����߶�֮��ľ��뷶Χ
    this->min_dist_lines = 50;
    this->max_dist_lines = 200;
    // �Ƕȷ�Χ
    this->max_angle_degrees = 10;
    this->min_len_seg = static_cast<unsigned>(512 * 0.15);
}

LineDetector::~LineDetector() {

}

MLine LineDetector::getReferenceLineByContours(cv::Mat bin_img) {
    MLine lineRes;
    memset(&lineRes,0, sizeof(lineRes));
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bin_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // �ҵ��������������Ϊ�ı���
    double maxArea = 0;
    std::vector<cv::Point> maxContour, approxContour;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            maxContour = contours[i];
        }
    }

    double epsilon = 0.02 * cv::arcLength(maxContour, true);
    cv::approxPolyDP(maxContour, approxContour, epsilon, true);

    // ��鲢����ı��ζ���
    if (approxContour.size() == 4) {
        for (int i = 0; i < 4; i++){
            bool swapped = false;
            for(int j = 0; j<3-i;j++){
                if(approxContour[j].y > approxContour[j+1].y){
                    cv::Point temp = approxContour[j];
                    approxContour[j] = approxContour[j+1];
                    approxContour[j+1] = temp;
                    swapped = true;
                }
            }
            if(!swapped){
                break;
            }
        }
        std::vector<cv::Point2f> points;
        points.emplace_back(approxContour[0]);
        points.emplace_back(approxContour[1]);
        std::vector<float> line_fited = fiting_line(points);
        if(line_fited[0]<0 | line_fited[1]<0){
            return lineRes;
        }
        lineRes = {line_fited[0], line_fited[1]+256, line_fited[2], line_fited[3]+256, line_fited[4]};
    }
    return lineRes;
}

MLine LineDetector::getReferenceLineByHKCU60(cv::Mat bin_img, std::vector<MLine> lines) {
    MLine lineRes;
    memset(&lineRes, 0, sizeof(lineRes));
    // 先获取粗糙板线
    MLine lineResRough = getReferenceLineByContours(bin_img);
    if(lineResRough.x1 == lineResRough.x2){ // 粗糙板线检测失败
        return lineRes;
    }


    double minDist = 1000;
    int minDistIndex = 0;
    double minYLsd = std::min(lineResRough.y1, lineResRough.y2);
    for(int i =0; i<lines.size(); i++){
//        if(abs(lines[i].x1 - lines[i].x2)<50){  // ���˽϶̵�ֱ��
//            continue;
//        }
        double minY = std::min(lines[i].y1, lines[i].y2);
        if(minY<256){ // 过滤分布在上半部分的参考
            continue;
        }

        if(minY>minYLsd){
            continue;
        }
        double lineDist = minYLsd-minY;
        if(lineDist<minDist && lineDist>20 ){
            minDist = lineDist;
            minDistIndex = i;
        }
    }


    if(minDist<200){ // ������
        std::vector<cv::Point2f> possLine = {
                cv::Point2f(lines[minDistIndex].x1, lines[minDistIndex].y1),
                cv::Point2f(lines[minDistIndex].x2, lines[minDistIndex].y2)
        };
        std::vector<float> referLine =  fiting_line(possLine);
        if(referLine[0]>=0 && referLine[1]>0 && referLine[2]>=0 && referLine[3]>0){
            lineRes.x1 = referLine[0];
            lineRes.y1 = referLine[1];
            lineRes.x2 = referLine[2];
            lineRes.y2 = referLine[3];
            lineRes.slope = referLine[4];
        }
    }
    return lineRes;
}







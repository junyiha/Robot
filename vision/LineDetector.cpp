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
    cv::Mat img_gray, bin_img, img_512;
    cv::resize(img, img_512, cv::Size(this->resize_w,this->resize_h));
    cv::cvtColor(img_512, img_gray, cv::COLOR_RGB2GRAY);
    cv::threshold(img_gray, bin_img, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
    res.imgDrawed = img_512;

    // mlsd  �㷨������п��ܵ��߶�
    auto start_poss = std::chrono::high_resolution_clock::now();
    std::vector<MLine> all_lines =  getAllPossibleLines(img_512);
    auto end_poss = std::chrono::high_resolution_clock::now();
    auto end_poss_total =  std::chrono::duration_cast<std::chrono::milliseconds>(end_poss - start_poss);
    std::cout<<"get possible times:"<<end_poss_total.count()<<std::endl;

    // 2.0 ��ȡ�ο���
    auto start_refer = std::chrono::high_resolution_clock::now();
    MLine referenceLine = getReferenceLine(img_512, all_lines);
    auto end_refer = std::chrono::high_resolution_clock::now();
    auto end_refer_total =  std::chrono::duration_cast<std::chrono::milliseconds>(end_refer - start_refer);
    std::cout<<"getReferenceLine:"<<end_refer_total.count()<<std::endl;

    if(referenceLine.x1!=referenceLine.x2){
        res.refResult = referenceLine;
        res.referLineStatus = true;
        cv::line(res.imgDrawed, cv::Point2f(referenceLine.x1, referenceLine.y1),
                 cv::Point2f(referenceLine.x2, referenceLine.y2), cv::Scalar(0, 0, 255), 1);
    }else{
        res.errorInfo = "Can not get reference line";
        return res;
    }

    // 3.0 ��ȡīˮ��
    auto start_ink = std::chrono::high_resolution_clock::now();
    MLine inkLine = getInkLine(bin_img, all_lines, referenceLine);
    auto end_ink = std::chrono::high_resolution_clock::now();
    auto end_ink_total =  std::chrono::duration_cast<std::chrono::milliseconds>(end_ink - start_ink);
    std::cout<<"getInkLine:"<<end_ink_total.count()<<std::endl;



    if(inkLine.x1!=inkLine.x2){
        res.inkResult = inkLine;
        res.inkLineStatus = true;
        cv::line(res.imgDrawed, cv::Point2f(inkLine.x1, inkLine.y1),
                 cv::Point2f(inkLine.x2, inkLine.y2), cv::Scalar(0, 255, 0), 1);
        res.status = true;
        double distance = calculateDistanceBetweenParallelLines(referenceLine.slope, inkLine.y1*h_ratio, referenceLine.y1*h_ratio);
        res.lineDist = distance;
    }
    return res;
}

MLine LineDetector::getInkLine(cv::Mat img, std::vector<MLine> lines,  MLine refLine) {

    MLine lineRes;
    memset(&lineRes, 0, sizeof(MLine));
     for(const auto &line:lines) {
         cv::Vec4f line_ = {line.x1, line.y1, line.x2, line.y2};
         if(line.y1>refLine.y1){ continue;}  // ī����ͨ���ڰ���֮��
         if (!check_is_border_line(img, line_)) {
             std::vector<cv::Point2f> points;
             points.push_back(cv::Point2f(line.x1, line.y1));
             points.push_back(cv::Point2f(line.x2, line.y2));
             std::vector<float> line_fited = fiting_line(points);

             if(abs(line_fited[4])>45){  // б�ʴ���30��,��Ϊ����īˮ��
                 continue;
             }
             if(abs(line_fited[4]-refLine.slope)>30){ // б����ο���б��������30��,��Ϊ����īˮ��
                 continue;
             }

             lineRes.x1 = line_fited[0];
             lineRes.y1 = line_fited[1];
             lineRes.x2 = line_fited[2];
             lineRes.y2 = line_fited[3];
             lineRes.slope = line_fited[4];
             break;
         }
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

    // ɸѡ���ߵ�
    std::vector<cv::Point2f> points;
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4f line = lines[i];

        float distance = calculatePointsDistance(cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]));
        // ����б���������ڰ��ߵķ��룿
        // ���ȴ�С���������λ���ص�

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

//    cv::imshow("bin_image", bin_img);
//    cv::waitKey();

    // ��ͳͼ�����㷨��ȡ���
    refLine_lsd = getReferenceLineLSD(img, bin_img);

    // ģ���㷨��ȡ���
    for (auto &line : lines) {
        cv::Vec4f line_ = { line.x1,line.y1, line.x2,line.y2 };
        if(abs(line.x1 - line.x2)<50){ continue;}  // �ο���̫�̣����������
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

    // ��ͳ�㷨��ģ���㷨�໥����
    if(refLine_model.x1!=refLine_model.x2){ // ģ��Ԥ������Ч
        if(refLine_lsd.x1!=refLine_lsd.x2){ // ��ͳ�㷨Ԥ������Ч
            if(abs(refLine_model.x1- refLine_lsd.x1)<50){ // �ж��������Ƿ��������
                refLine = refLine_lsd;
            }else{
                refLine = refLine_model;
            }
        }else{ //��ͳ�㷨Ԥ������Ч
            refLine = refLine_model;
        }
    }else{
        refLine = refLine_lsd;
    }
    return refLine;
}

std::vector<MLine> LineDetector::getAllPossibleLines(cv::Mat img) {

    // MLSD�㷨������п��ܵ��߶�
    std::vector<MLine> line_res;
    std::vector<std::vector<float>>  lines_mlsd = this->mlsd.detect(img);
    cv::Mat line_mask = cv::Mat::zeros(img.size(), CV_8UC1);
    for(auto item: lines_mlsd){
        cv::line(line_mask, cv::Point(item[0], item[1]), cv::Point(item[2], item[3]), cv::Scalar(255), 1);
    }
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<float>> lines = get_line_by_lsd(line_mask);
    auto end = std::chrono::high_resolution_clock::now();

    // ���㲢�������ʱ��
    auto  elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "get_line_by_lsd: " << elapsed_seconds.count() << " ��" << std::endl;


    sort_lines(lines);
    std::vector<std::vector<float>> lines_new = linesAggregation(lines);

    //�Լ�����ֱ�߽�һ������
    for (auto &line : lines_new) {
        // 1.0 ���߷ֲ�λ���ص����
        if (line[1] < this->min_pos_valid || line[3] > max_pos_valid) { continue;}
        // 2.0 ���˵���ֱΪ��ֱ�����ֱ��
        if(line[1]<0 | line[3]<0|line[1]>img.cols|line[3]>img.cols){
            continue;
        }
        MLine line_true = {line[0],line[1],line[2],line[3],line[4]};
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

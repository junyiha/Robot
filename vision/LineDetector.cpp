//
// Created by csh_i on 2024/10/21.
//

#include "LineDetector.h"

LineDetector::LineDetector() {

}

LineDetector::~LineDetector() {

}

std::vector<cv::Mat> LineDetector::getMasks(cv::Mat maskIndex) {

    std::vector<cv::Mat> masks;
    cv::Mat inkMask = cv::Mat::zeros(maskIndex.size(), CV_8UC1);
    cv::Mat referMask = cv::Mat::zeros(maskIndex.size(), CV_8UC1);

    for (int y = 0; y < maskIndex.rows; ++y) {
        for (int x = 0; x < maskIndex.cols; ++x) {
            if (maskIndex.at<uchar>(y, x) == 1) {
                inkMask.at<uchar>(y, x) = 255;
            } else if (maskIndex.at<uchar>(y, x) == 2) {
                referMask.at<uchar>(y, x) = 255;
            }
        }
    }
    masks.push_back(inkMask);
    masks.push_back(referMask);
    return masks;
}

std::vector<float> LineDetector::getReferenceLineLSD(cv::Mat img, cv::Mat bin_img){

//    cv::imshow("img", bin_img);
//    cv::waitKey();
    std::vector<float> lineRes;
    cv::Mat bin_img_blur;
    cv::GaussianBlur(bin_img, bin_img_blur, cv::Size(5,5), 0 );
    cv::Ptr<cv::LineSegmentDetector> detector = createLineSegmentDetector(cv::LSD_REFINE_STD);
    std::vector<cv::Vec4f> lines;
    detector->detect(bin_img_blur, lines);
//    cv::Mat temp = cv::Mat::zeros(img.size(),CV_8UC3);
//    for(int i=0;i<lines.size();i++){
//
//        cv::line(temp,cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,255,0), 2);
//
//    }
//    cv::imshow("temp",temp);
//    cv::waitKey(0);


    std::vector<cv::Point2f> points;
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4f line = lines[i];

        // 1.位置过滤
        double max_y =  line[1]>line[3]?line[1]:line[3];
        if(max_y<img.rows/2 || max_y>img.rows*0.8){
            continue;
        }
        // 2.斜率过滤
        double slope = (line[1] - line[3]) / (line[0] - line[2]);
        double dgree = atan(slope)*180/CV_PI;
        if(abs(dgree)>20){
            continue;
        }

        float distance = calculatePointsDistance(cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]));
        bool isBoundary = check_is_border_line(bin_img, line);
        if ((distance > this->imgInputW*0.1) && isBoundary) {
            points.push_back(cv::Point2f(line[0], line[1]));
            points.push_back(cv::Point2f(line[2], line[3]));
            break;
        }
    }
    if(points.size()>0){
        std::vector<float>  border_line = fiting_line(points);
        lineRes = border_line;
    }
    return lineRes;
}

std::vector<float> LineDetector::getRerferenceLine(cv::Mat referMask, cv::Mat imgInput) {

    std::vector<float> referenceLine;
    std::vector<std::vector<cv::Point2f>> possibleLines;

    // 1. get possible lines from mask
    getPossibleLinesFromMask(referMask, possibleLines);

    if(possibleLines.size() >0){
        // 进一步筛选
        double max_dist = 0;
        int max_index = -1;
        for (size_t i = 0; i < possibleLines.size(); ++i) {
            double dist_i = calculatePointsDistance(possibleLines[i][0], possibleLines[i][1]);
            // 1.0 长度过滤
            if(dist_i<this->imgInputW*0.1){ // 1. too smal line, ignore it
                continue;
            }
            // 2.0 分布位置过滤
            if(possibleLines[i][0].y < imgInputH/2){ // 2. reference line must in upper half of image
                continue;
            }
            // ToDo: 角度过滤

            // 3.0 过滤绝缘最外层边线
//            cv::Point2f p1;
//            p1.x = (possibleLines[i][0].x+possibleLines[i][1].x)/2;
//            p1.y = (possibleLines[i][0].y+possibleLines[i][1].y)/2;
//            cv::Rect rect(p1.x, p1.y, 30, 30);
//            cv::Mat roi = imgInput(rect);
//            double mean_color = cv::mean(roi)[0];
//            if(mean_color>180){
//                continue;
//            }

            if(dist_i > max_dist) {
                max_dist = dist_i;
                max_index = i;
            }
        }
        if(max_index!= -1){
            std::vector<float> lineRes = fiting_line(possibleLines[max_index]);
            if(lineRes[0]>=0 && lineRes[1]>=0 && lineRes[2]>=0 && lineRes[3]>=0){
                referenceLine = lineRes;
            }else{
                std::cout << "get Line error" << std::endl;
            }
        }
    }
    if(referenceLine.empty()){
        // LSD算法兜底
        cv::Mat bin_img; 
        cv::threshold(imgInput, bin_img, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU); 
        std::vector<float> refLineLSD =  getReferenceLineLSD(imgInput, bin_img); 
        referenceLine = refLineLSD;

    }


    return referenceLine;
}

void LineDetector::getPossibleLinesFromMask(cv::Mat mask, vector<std::vector<cv::Point2f>> &possibleLines) {

    // 1. blur mask
//    cv::Mat referMaskBlur;
//    cv::GaussianBlur(mask, referMaskBlur, cv::Size(5, 5), 0);
    cv::Mat binImg;
    cv::threshold(mask, binImg, 0, 255, cv::THRESH_OTSU);

    //2. compute contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if(contours.size()==0){
       return ;
    }

    // 3. filter contours
    for (size_t i = 0; i < contours.size(); ++i) {

        // compute area of contour
        double area = contourArea(contours[i]);
        if (area < 768*0.2) { // 1. too smal line, ignore it
            continue;
        }

        // compute minAreaRect
        cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
        cv::Mat points;
        std::vector<cv::Point2f> rectPoints;
        cv::boxPoints(minRect, points);

        for (int i = 0; i < points.rows; ++i) {
            rectPoints.push_back(points.at<cv::Point2f>(i, 0));
        }

        // sort points by x-axis
        for (int j = 0; j < rectPoints.size() - 1; ++j) {
            for (int k = 0; k < rectPoints.size() - j - 1; ++k) {
                if (rectPoints[k].x > rectPoints[k + 1].x) {
                    cv::Point temp = rectPoints[k];
                    rectPoints[k] = rectPoints[k + 1];
                    rectPoints[k + 1] = temp;
                }
            }
        }

        // 过滤掉竖直型轮廓
        if(abs(rectPoints[0].y-rectPoints[1].y)>imgInputH*0.1){
            continue;
        }

        // get left and right point
        cv::Point2f point_left;
        cv::Point2f point_right;
        if(abs(rectPoints[0].y-rectPoints[1].y)>imgInputH*0.05){ // 处理板线水平与垂直粘连的情况
            point_left = rectPoints[0];
            point_right = rectPoints[2].y<rectPoints[3].y?rectPoints[2]:rectPoints[3];
        }else{
            point_left = cv::Point((rectPoints[0].x+rectPoints[1].x)/2, (rectPoints[0].y+rectPoints[1].y)/2);
            point_right = cv::Point((rectPoints[2].x+rectPoints[3].x)/2, (rectPoints[2].y+rectPoints[3].y)/2);
        }
        possibleLines.push_back({point_left, point_right});
    }
}

std::vector<float> LineDetector::getInkLine(cv::Mat inkMask, std::vector<float> referenceLine){

    std::vector<float> inkLine;
    std::vector<std::vector<cv::Point2f>> possibleLines;
    getPossibleLinesFromMask(inkMask, possibleLines);

    double dgree_ref = atan((referenceLine[3]-referenceLine[1])/(referenceLine[2]-referenceLine[0]))*180/CV_PI;

    if(possibleLines.size() >0){
        // 进一步筛选
        double max_dist = 0;
        int max_index = -1;
        for (size_t i = 0; i < possibleLines.size(); ++i) {
            double dist_i = calculatePointsDistance(possibleLines[i][0], possibleLines[i][1]);
            // 1. 长度大小过滤
            if(dist_i<768*0.15){
                continue;
            }
            //2. 分布位置过滤
            double min_ref_y =  referenceLine[1]<referenceLine[3]?referenceLine[1]:referenceLine[3];
            double max_ink_y =  possibleLines[i][0].y<possibleLines[i][1].y?possibleLines[i][1].y:possibleLines[i][0].y;
            if(max_ink_y>min_ref_y){
                continue;
            }
            // 3.角度偏差过滤
            double slope_i = (possibleLines[i][1].y - possibleLines[i][0].y) / (possibleLines[i][1].x - possibleLines[i][0].x);
            double dgree_i = atan(slope_i)*180/CV_PI;
            if(abs(dgree_i-dgree_ref)>10){
                continue;
            }

            if(dist_i > max_dist) {
                max_dist = dist_i;
                max_index = i;
            }
        }
        if(max_index!= -1){
            std::vector<float> lineRes = fiting_line(possibleLines[max_index]);
            if(lineRes[0]>=0 && lineRes[1]>=0 && lineRes[2]>=0 && lineRes[3]>=0){
                inkLine = lineRes;
            }else{
                std::cout << "getRerferenceLine error" << std::endl;
            }
        }
    }
    return inkLine;
}

LineSpaceResult LineDetector::getLinesDistance(cv::Mat img) {

    LineSpaceResult lineResult;
    lineResult.status = false;
    lineResult.border_line_status = false;
    lineResult.ink_line_status = false;
//    memset(&lineResult, 0, sizeof(LineSpaceResult));

    if(img.empty() || img.isContinuous() == false){
        lineResult.error_info = "img is empty or not continuous";
        return lineResult;
    }

    cv::Mat img_resize;
    cv::resize(img, img_resize, cv::Size(imgInputH, imgInputW));
    int originH = img.rows;
    int originW = img.cols;
    float scaleH = (float)originH / imgInputH;
    float scaleW = (float)originW / imgInputW;

    lineResult.img_drawed = img_resize.clone();
    cv::Mat img_gray;
    if(img.channels() == 3){
        cv::cvtColor(img_resize, img_gray, cv::COLOR_BGR2GRAY);
    }else{
        img_gray = img;
    }

    // 1. image enhancement
    cv::Mat imgEqHist;
    cv::equalizeHist(img_gray, imgEqHist);

    // 2. get mask index
    cv::Mat maskIndex = lineSegModel.predict(imgEqHist);
    double maskSum = cv::sum(maskIndex)[0];
//    cv::imshow("maskIndex", maskIndex*255);
//    cv::waitKey(0);
    if(maskSum<100){
        lineResult.error_info = "Not detected any matching straight line";
        return lineResult;
    }

    // 3. get reference line
    std::vector<cv::Mat> masks;
    masks = getMasks(maskIndex);
//    std::vector<float> referLine = getRerferenceLine(masks[1],img_gray);
    std::vector<float> referLine = getReferenceLineFromMask(masks[1],img_gray);
    if(referLine.size()>0){
        lineResult.border_res = referLine;
        lineResult.border_line_status = true;
        cv::line(lineResult.img_drawed, cv::Point(referLine[0], referLine[1]), cv::Point(referLine[2], referLine[3]), cv::Scalar(0, 0, 255), 2);
    }else{
        lineResult.error_info = "Detection board line failed!";
        return lineResult;
    }

    // 4. get ink line
    std::vector<float> inkLine = getInkLine(masks[0], referLine);
    if(inkLine.size()>0){
        lineResult.ink_res = inkLine;
        lineResult.ink_line_status = true;
        cv::line(lineResult.img_drawed, cv::Point(inkLine[0], inkLine[1]), cv::Point(inkLine[2], inkLine[3]), cv::Scalar(0, 255, 0), 2);
    }else{
        lineResult.error_info = "Detection ink line failed!";
        return lineResult;
    }

    // ToDo: 排除板线，仅仅获取内部线
    // ToDo: 角度进行校验
    referLine = {referLine[0]*scaleW, referLine[1]*scaleH, referLine[2]*scaleW, referLine[3]*scaleH,referLine[4]};
    inkLine = {inkLine[0]*scaleW, inkLine[1]*scaleH, inkLine[2]*scaleW, inkLine[3]*scaleH, referLine[4]};
    double dist  = computePointToLineDistance(referLine, inkLine);
    lineResult.dist = dist;
    lineResult.status = true;
    return lineResult;
}

std::vector<float> LineDetector::getReferenceLineFromMask(cv::Mat mask, cv::Mat img_gray) {
    std::vector<float> referLine;
    cv::Ptr<cv::LineSegmentDetector> detector = createLineSegmentDetector(cv::LSD_REFINE_STD);
    std::vector<cv::Vec4f> lines;
    detector->detect(mask, lines);

    std::vector<std::vector<float>> possiableLines;
    for(int i=0;i<lines.size();i++){
        cv::Vec4f line = lines[i];

        double lineDistX = abs(line[0]-line[2]);
        double scope = (line[1]-line[3])/(line[0]-line[2]);
        double angle = atan(scope)*180/CV_PI;
        double max_y =  line[1]>line[3]?line[1]:line[3];
        // 位置过滤
        if(max_y<mask.rows/2 || max_y>mask.rows*0.8){
            continue;
        }
        // 斜率过滤
        if(abs(angle)>30){
            continue;
        }
        // 长度过滤
        if(lineDistX<mask.cols*0.1){
            continue;
        }
        possiableLines.push_back({line[0], line[1], line[2], line[3] , static_cast<float>(scope)});
    }

    // 寻找长度最长的候选直线
    if(possiableLines.size()>0){
        double index = -1;
        double max_len = -1;
        for(int i=0;i<possiableLines.size();i++){
            double lineLen = abs(possiableLines[i][0]-possiableLines[i][2]);
            if(lineLen>max_len){
                max_len = lineLen;
                index = i;
            }
        }
        if(max_len!=-1){
            std::vector<cv::Point2f> lineFin;
            lineFin.push_back(cv::Point2f(possiableLines[index][0],possiableLines[index][1]));
            lineFin.push_back(cv::Point2f(possiableLines[index][2],possiableLines[index][3]));
            std::vector<float> lineRes = fiting_line(lineFin);
            if (lineRes.size() > 0)
            {
                if(lineRes[0]>=0 && lineRes[1]>=0 && lineRes[2]>=0 && lineRes[3]>=0){
                    // 板线位置限制 
                    double max_y_ref = lineRes[1] > lineRes[3] ? lineRes[1] : lineRes[3];
                    if (max_y_ref>this->imgInputH*0.5 && max_y_ref < this->imgInputH * 0.85) {
                        referLine = lineRes;
                    }
                
                }else{
                    std::cout << "get Line error" << std::endl;
                }
    //            referLine = possiableLines[index];

            }
        }
    }else{ // 传统算法进行兜底
        cv::Mat bin_img;
        cv::threshold(img_gray, bin_img, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
        std::vector<float> refLineLSD =  getReferenceLineLSD(img_gray, bin_img);
        if (refLineLSD.size() > 0)
        {
            // 板线位置限制
            double max_y_ref_lsd = refLineLSD[1] > refLineLSD[3] ? refLineLSD[1] : refLineLSD[3];
            if (max_y_ref_lsd > this->imgInputH * 0.5 && max_y_ref_lsd<this->imgInputH * 0.85) {
                referLine = refLineLSD;
            }
        }
    }
    return referLine;



}



//
// Created by csh_i on 2024/10/21.
//

#include "LineDetector.h"

LineDetector::LineDetector()
{

}

LineDetector::~LineDetector()
{

}

std::vector<cv::Mat> LineDetector::getMasks(cv::Mat maskIndex)
{

    std::vector<cv::Mat> masks;
    cv::Mat inkMask = cv::Mat::zeros(maskIndex.size(), CV_8UC1);
    cv::Mat referMask = cv::Mat::zeros(maskIndex.size(), CV_8UC1);

    for (int y = 0; y < maskIndex.rows; ++y)
    {
        for (int x = 0; x < maskIndex.cols; ++x)
        {
            if (maskIndex.at<uchar>(y, x) == 1)
            {
                inkMask.at<uchar>(y, x) = 255;
            }
            else if (maskIndex.at<uchar>(y, x) == 2)
            {
                referMask.at<uchar>(y, x) = 255;
            }
        }
    }
    masks.push_back(inkMask);
    masks.push_back(referMask);
    return masks;
}

std::vector<float> LineDetector::getRerferenceLine(cv::Mat referMask, cv::Mat imgInput)
{

    std::vector<float> referenceLine;
    std::vector<std::vector<cv::Point2f>> possibleLines;

    // 1. get possible lines from mask
    getPossibleLinesFromMask(referMask, possibleLines);

    if (possibleLines.size() > 0)
    {
        // 进一步筛选
        double max_dist = 0;
        int max_index = -1;
        for (size_t i = 0; i < possibleLines.size(); ++i)
        {
            // double dist_i = calculatePointsDistance(possibleLines[i][0], possibleLines[i][1]);
            double dist_i =  possibleLines[i][0].y>possibleLines[i][1].y?possibleLines[i][0].y:possibleLines[i][1].y;
            
            calculatePointsDistance(possibleLines[i][0], possibleLines[i][1]);
            // 1.0 长度过滤
            if (dist_i < 768 * 0.15)
            { // 1. too smal line, ignore it
                continue;
            }
            // 2.0 分布位置过滤
            if (possibleLines[i][0].y < imgInputH / 2)
            { // 2. reference line must in upper half of image
                continue;
            }
            // ToDo: 角度过滤

            // 3.0 过滤绝缘最外层边线
            cv::Point2f p1;
            p1.x = (possibleLines[i][0].x + possibleLines[i][1].x) / 2;
            p1.y = (possibleLines[i][0].y + possibleLines[i][1].y) / 2;
            if (p1.x + 30 > imgInput.cols || p1.y + 30 > imgInput.rows)
            {
                continue;
            }
            cv::Rect rect(p1.x, p1.y, 30, 30);
            cv::Mat roi = imgInput(rect);
            double mean_color = cv::mean(roi)[0];
            if (mean_color > 180)
            {
                continue;
            }

            if (dist_i > max_dist)
            {
                max_dist = dist_i;
                max_index = i;
            }
        }
        if (max_index != -1)
        {
            std::vector<float> lineRes = fiting_line(possibleLines[max_index]);
            if (lineRes[0] >= 0 && lineRes[1] >= 0 && lineRes[2] >= 0 && lineRes[3] >= 0)
            {
                referenceLine = lineRes;
            }
            else
            {
                std::cout << "get Line error" << std::endl;
            }
        }
    }
    return referenceLine;
}

void LineDetector::getPossibleLinesFromMask(cv::Mat mask, vector<std::vector<cv::Point2f>>& possibleLines)
{

    // 1. blur mask
    cv::Mat referMaskBlur;
    cv::GaussianBlur(mask, referMaskBlur, cv::Size(5, 5), 0);
    cv::Mat binImg;
    cv::threshold(referMaskBlur, binImg, 0, 255, cv::THRESH_OTSU);

    //2. compute contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size() == 0)
    {
        return;
    }

    // 3. filter contours
    for (size_t i = 0; i < contours.size(); ++i)
    {

        // compute area of contour
        double area = contourArea(contours[i]);
        if (area < 768 * 0.2)
        { // 1. too smal line, ignore it
            continue;
        }

        // compute minAreaRect
        cv::RotatedRect minRect = cv::minAreaRect(contours[i]);
        cv::Mat points;
        std::vector<cv::Point2f> rectPoints;
        cv::boxPoints(minRect, points);

        for (int i = 0; i < points.rows; ++i)
        {
            rectPoints.push_back(points.at<cv::Point2f>(i, 0));
        }

        // sort points by x-axis
        for (int j = 0; j < rectPoints.size() - 1; ++j)
        {
            for (int k = 0; k < rectPoints.size() - j - 1; ++k)
            {
                if (rectPoints[k].x > rectPoints[k + 1].x)
                {
                    cv::Point temp = rectPoints[k];
                    rectPoints[k] = rectPoints[k + 1];
                    rectPoints[k + 1] = temp;
                }
            }
        }

        // get left and right point
        cv::Point2f point_left = cv::Point((rectPoints[0].x + rectPoints[1].x) / 2, (rectPoints[0].y + rectPoints[1].y) / 2);
        cv::Point2f point_right = cv::Point((rectPoints[2].x + rectPoints[3].x) / 2, (rectPoints[2].y + rectPoints[3].y) / 2);
        possibleLines.push_back({ point_left, point_right });
    }
}

std::vector<float> LineDetector::getInkLine(cv::Mat inkMask, std::vector<float> referenceLine)
{

    std::vector<float> inkLine;
    std::vector<std::vector<cv::Point2f>> possibleLines;
    getPossibleLinesFromMask(inkMask, possibleLines);

    double dgree_ref = atan((referenceLine[3] - referenceLine[1]) / (referenceLine[2] - referenceLine[0])) * 180 / CV_PI;

    if (possibleLines.size() > 0)
    {
        // 进一步筛选
        double max_dist = 0;
        int max_index = -1;
        double min_dist = 10000;
        int min_index = -1; 

        for (size_t i = 0; i < possibleLines.size(); ++i)
        {
            double dist_i = calculatePointsDistance(possibleLines[i][0], possibleLines[i][1]);
            // 1. 长度大小过滤
            if (dist_i < 768 * 0.15)
            {
                continue;
            }
            //2. 分布位置过滤
            double max_ref_y = referenceLine[1] > referenceLine[3] ? referenceLine[1] : referenceLine[3];
            double max_ink_y = possibleLines[i][0].y < possibleLines[i][1].y ? possibleLines[i][1].y : possibleLines[i][0].y;
            if (max_ink_y > max_ref_y)
            {
                continue;
            }
            // 3.角度偏差过滤
            double slope_i = (possibleLines[i][1].y - possibleLines[i][0].y) / (possibleLines[i][1].x - possibleLines[i][0].x);
            double dgree_i = atan(slope_i) * 180 / CV_PI;
            if (abs(dgree_i - dgree_ref) > 10)
            {
                continue;
            }

            if (dist_i > max_dist)
            {
                max_dist = dist_i;
                max_index = i;
            }
            
            // 寻找距离板线最近的墨迹线
            double ref_ink_dist_y = abs(max_ink_y - max_ref_y);
            if(min_dist>ref_ink_dist_y){
                min_dist = ref_ink_dist_y; 
                min_index = i;
            } 
        }
        if (min_index != -1)
        {
            std::vector<float> lineRes = fiting_line(possibleLines[min_index]);
            if (lineRes[0] >= 0 && lineRes[1] >= 0 && lineRes[2] >= 0 && lineRes[3] >= 0)
            {
                inkLine = lineRes;
            }
            else
            {
                std::cout << "getRerferenceLine error" << std::endl;
            }
        }
    }
    return inkLine;
}

LineSpaceResult LineDetector::getLinesDistance(cv::Mat img)
{

    LineSpaceResult lineResult;
    lineResult.status = false;
    lineResult.border_line_status = false;
    lineResult.ink_line_status = false;
    //    memset(&lineResult, 0, sizeof(LineSpaceResult));

    if (img.empty() || img.isContinuous() == false)
    {
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
    if (img.channels() == 3)
    {
        cv::cvtColor(img_resize, img_gray, cv::COLOR_BGR2GRAY);
    }

    // 1. image enhancement
//    cv::Mat imgEqHist;
//    cv::equalizeHist(img_gray, imgEqHist);

    // 2. get mask index
    cv::Mat maskIndex = lineSegModel.predict(img_gray);
    double maskSum = cv::sum(maskIndex)[0];

    if (maskSum < 100)
    {
        lineResult.error_info = "Not detected any matching straight line";
        return lineResult;
    }

    // 3. get reference line
    std::vector<cv::Mat> masks;
    masks = getMasks(maskIndex);
    std::vector<float> referLine = getRerferenceLine(masks[1], img_gray);
    if (referLine.size() > 0)
    {
        lineResult.border_res = referLine;
        lineResult.border_line_status = true;
        cv::line(lineResult.img_drawed, cv::Point(referLine[0], referLine[1]), cv::Point(referLine[2], referLine[3]), cv::Scalar(0, 0, 255), 2);
    }
    else
    {
        lineResult.error_info = "Detection board line failed!";
        return lineResult;
    }

    // 4. get ink line
    std::vector<float> inkLine = getInkLine(masks[0], referLine);
    if (inkLine.size() > 0)
    {
        lineResult.ink_res = inkLine;
        lineResult.ink_line_status = true;
        cv::line(lineResult.img_drawed, cv::Point(inkLine[0], inkLine[1]), cv::Point(inkLine[2], inkLine[3]), cv::Scalar(0, 255, 0), 2);
    }
    else
    {
        lineResult.error_info = "Detection ink line failed!";
        return lineResult;
    }

    // ToDo: 排除板线，仅仅获取内部线
    // ToDo: 角度进行校验
    referLine = { referLine[0] * scaleW, referLine[1] * scaleH, referLine[2] * scaleW, referLine[3] * scaleH,referLine[4] };
    inkLine = { inkLine[0] * scaleW, inkLine[1] * scaleH, inkLine[2] * scaleW, inkLine[3] * scaleH, referLine[4] };
    double dist = computePointToLineDistance(referLine, inkLine);
    lineResult.dist = dist;
    lineResult.status = true;
    return lineResult;
}



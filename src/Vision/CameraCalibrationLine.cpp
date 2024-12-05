//
// Created by csh_i on 2024/10/31.
//

#include "CameraCalibrationLine.h"

CameraCalibrationLine::CameraCalibrationLine()
{
}

CameraCalibrationLine::~CameraCalibrationLine()
{
}

bool CameraCalibrationLine::checkDataIsValid(cv::Mat image)
{

    if (image.empty())
    {
        return false;
    }
    if (image.type() != CV_8UC1)
    {
        return false;
    }
    double meanValue = cv::mean(image)[0];
    if (meanValue < 100)
    {
        return false;
    }
    else
    {
        return true;
    }
}

CalibrationLine CameraCalibrationLine::findCalibrationLine(cv::Mat image)
{

    CalibrationLine calibrationLine;
    calibrationLine.status = false;
    calibrationLine.imageDrawd = image.clone();

    // processing image
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    cv::Mat equImg;
    cv::equalizeHist(grayImage, equImg);
    cv::Mat binImg;
    cv::threshold(equImg, binImg, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // find rectangle
    if (checkDataIsValid(binImg))
    { // ToDo: 待修改
        cv::Mat binImgCopy = binImg.clone();
        binImgCopy = 255 - binImgCopy;
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binImgCopy, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::Point> approxContour;

        for (int i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);

            //            if(area>10*10){
            //                continue;
            //            }

            double epsilon = 0.02 * cv::arcLength(contours[i], true);

            cv::approxPolyDP(contours[i], approxContour, epsilon, true);

            if (approxContour.size() == 4)
            {
                // sort points
                for (int i = 0; i < 4; i++)
                {
                    bool swapped = false;
                    for (int j = 0; j < 3 - i; j++)
                    {
                        if (approxContour[j].y > approxContour[j + 1].y)
                        {
                            cv::Point temp = approxContour[j];
                            approxContour[j] = approxContour[j + 1];
                            approxContour[j + 1] = temp;
                            swapped = true;
                        }
                    }
                    if (!swapped)
                    {
                        break;
                    }
                }
                break;
            }
        }

        if (approxContour.size() > 0)
        {
            calibrationLine.status = true;
            cv::rectangle(calibrationLine.imageDrawd, approxContour[0], approxContour[3], cv::Scalar(255), 2);
            double dist = fabs(approxContour[0].x - approxContour[1].x);
            calibrationLine.scale = this->RectangleWidth / dist;
        }
    }
    return calibrationLine;
}

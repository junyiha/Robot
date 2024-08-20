//
// Created by csh_i on 2024/6/25.
//

#ifndef HK_DEMO2_PARAMETERS_H
#define HK_DEMO2_PARAMETERS_H
#include"MvCamera.h"
#include <iostream>

// ��������豸���ӽṹ��
struct HKCameraDeviceInfo{
    const char* camName ;                                         //  �������
    const char* camIp;                                            //  ���IP��ַ
    const char*  ethIp ;                                             //  ��̫��IP��ַ
    MV_CC_DEVICE_INFO pstDeviceInfo ;         //  ����豸��Ϣ���
    MV_CC_DEVICE_INFO stDevInfo = {0};
    MV_GIGE_DEVICE_INFO stGigEDev = {0};
};


enum DetectType{
    LINE, HOLE, LIDAR
};


// LineDetectorRunner ����, ֱ�߼�����ṹ��
struct LineDetectRes{
    bool status;
    float dist;
    cv::Mat img_drawed;
    std::string cam_name;
};


// LineDetector��, ֱ�߼�ֱ�߼�����Ľṹ�嶨��
struct MLine{
    float x1;
    float y1;
    float x2;
    float y2;
    float slope;
};


struct LineResult {
    float lineDist;
    bool status;
    bool inkLineStatus;
    bool referLineStatus;

    std::string errorInfo; //������Ϣ
    MLine inkResult;  // ī�߽��
    MLine refResult;  // �ο��߽��
    cv::Mat imgDrawed;
};











#endif //HK_DEMO2_PARAMETERS_H

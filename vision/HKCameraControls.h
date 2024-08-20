//
// Created by csh_i on 2024/6/7.
//

#ifndef HK_DEMO2_HKCAMERACONTROLS_H
#define HK_DEMO2_HKCAMERACONTROLS_H
#include<QThread>
#include"MvCamera.h"
#include<opencv2/opencv.hpp>
#include<QMutex>
#include<queue>
#include<windows.h>
#include "Parameters.h"
#include <spdlog/spdlog.h>


/************ Single HK Camera  ***********/

class HKCameraControls : public QThread{
      Q_OBJECT
public:
    std::queue<cv::Mat> camera_queue;
    QMutex data_mutex;
    bool open_status = false;
    bool thread_runing = true;
    bool isRunning = true;
    CMvCamera* pstMvCamera = nullptr;
    std::shared_ptr<spdlog::logger> logger;


    HKCameraControls(const char* camName, const char* camIp, const char* ethIp);
    HKCameraControls(MV_CC_DEVICE_INFO pstDeviceInfo, const char* camIp);


    ~HKCameraControls();
    void run() override;
    int openCamera();
    int closeCamera();
    void getCameraConfigInfo();
    void get_frame( );
    cv::Mat getFrame();
    // start image acquisition
    void start_image_acquisition(std::string mode);

    // initialize camera config
    void initializationCameraConfig();

    cv::Mat getImageFromeQueue();

    void closeThread();





    cv::Mat getOneImage();

    static void __stdcall ReconnectDevice(unsigned int nMsgType, void* pUser);

    int SetExposureTime();
    int SetGain();
    int SetFrameRate();
    int SetImageResolution();
    cv::Size GetImageResolution();
    int setGevSCPD();
    int setHeartBeatTimeOut();
    int GetSDKVersion();




private:

    MV_CC_DEVICE_INFO pstDeviceInfo ;  //  相机设备信息相关
    MV_CC_DEVICE_INFO stDevInfo = {0};
    MV_GIGE_DEVICE_INFO stGigEDev = {0};
    HKCameraDeviceInfo stHkDev ;


    /*****图像预处理超参数配置相关 ********/
    double  m_dFrameRateEdit = 5.0;     // 帧率
    int imgWidth = 768;                       //  图像宽度
    int imgHeight = 512;                      //  图像高度
    int stIntValue_SCPD = 800;           //  SCPD  数据包间隔
    int stIntValue_HBTO = 1000;            //  心跳包超时时间
    double m_dGainEdit = 1000.0 ;         // 相机增益
    double  m_dExposureEdit = 2000.0;       //  曝光时间

       //  相机句柄
    unsigned short  max_queue_size = 5;    //  队列最大容量


    // pdata convert to cv::Mat
    cv::Mat Convert2Mat(MV_FRAME_OUT_INFO_EX* pstFrameInfo, unsigned char * pData);
    // RGB2BGR
    int RGB2BGR( unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight );

    int getGevSCPD();

    int getGevHeartbeatTimeout();

    float getFrameRate();

    float GetGain();

    float getExposureTime();
};


#endif //HK_DEMO2_HKCAMERACONTROLS_H

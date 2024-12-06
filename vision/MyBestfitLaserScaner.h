//
// Created by csh_i on 2024/9/9.
//

#ifndef BESTFITLASERDEMO_MYBESTFITLASERSCANER_H
#define BESTFITLASERDEMO_MYBESTFITLASERSCANER_H
#include<QThread>
#include<windows.h>
#include<BestfitLaserScanerSDKDefine.h>
#include<BestfitLaserScanerSDK.h>
//#include "pevents.h"
#include<QPixmap>
#include<QDateTime>
#include<QMessageBox>
#include<QDebug>
#include<QFile>
#include<iostream>
#include<opencv2/opencv.hpp>
#include<spdlog/spdlog.h>
//#include "cimage.h"
#include <QTimer>
#include<QMutex>

#define BUFFERLENGTH        1000
#define EXTRAINFOLENGTH     32
#define IMAGEWIDTHMAX       1280
#define IMAGEHEIGHTMAX      1024

#define NORUNINGTASK        0x00
#define IMAGEPREVIEWTASK    0x01
#define GRABCLOUDTASK       0x02

struct LaserScanerDeviceInfo {
    std::string Laser_name;
    std::string Ip;
    std::string Timeout;
    std::string SN;
    std::string Mask;
    std::string Gateway;
};





class MyBestfitLaserScaner: public QThread {
    Q_OBJECT
public:
    std::string      laser_name;

    // Scanner connect and status
    void            *m_hScanner= nullptr; //句柄
    bool            m_bScannerConnectStatus{false};//是否连接
    bool            m_isRunning{true};
    bool            m_threadQuit{false};

    // 触感器属性相关变量
    std::string     m_strSupplier;
    std::string     m_strProducer;
    std::string     m_strDeviceID;
    std::string     m_strFWVersion;
    std::string     m_strIPAddress;
    std::string     m_strGetManufacturer;
    std::string     m_strZstart;
    std::string     m_strZrange;
    std::string     m_strXRangeAtStart;
    std::string     m_strXRangeAtEnd;
    double          m_dZstart;
    double          m_dZrange;
    double          m_dZbest;
    double          m_dXRangeAtStart;
    double          m_dXRangeAtEnd;
    double          m_dXRangeBest;

    int             m_num = 0; 


    // Scanner output members (传感器获取点云数据缓存相关变量）
    char            *m_cImgData;
    char            m_cScannerInfo[ETHERNETSCANNER_GETINFOSIZEMAX];
    double          m_dScannerBufferX[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];
    double          m_dScannerBufferZ[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX]; // 2048*4
    int             m_iScannerBufferI[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];
    int             m_iScannerBufferPeakWidth[ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX];
    unsigned int    m_uScannerEncoder;
    unsigned char   m_ucScannerDigitalInputs;

    QMutex          m_mutex;
    std::mutex      m_picCntMutex;

    //Scanner members
    int    m_usPicCnt;
    unsigned long   m_dwScanner_Frequency;
    int             m_iLengthReceivedData;
    int    m_iPicCntErr;
    int             m_iNumberOfSavedProfiles;
    clock_t         m_timerCnt;
    int             m_iTaskType;

    // 点云输出结果相关变量
    int m_bGrabCloud_Once;  // 触发模式
    std::vector<LaserScanerDeviceInfo> m_vecScannerDeviceInfo;
    std::vector<cv::Point2f> m_vecPointClouds;
    std::vector<cv::Point2f> m_vecPointClouds_;
    QMutex pointCloudsMutex;
    cv::Mat mask;
    bool dataValid;
    void setDataValid(bool valid);
    bool isDataValid();

    // 点云数据是否有效
    cv::Mat resultMask;
    QMutex maskMutex;
    QMutex dataValidMutex;

    std::shared_ptr<spdlog::logger> logger;


    // 装板相关参数
    bool isleft;
    bool revAngle;
    int  minX;
    int  minY;
    void setResultMask(cv::Mat resultMask);
    cv::Mat getResultMask();


    MyBestfitLaserScaner(const std::string scannerIP,  std::string laserName);
    ~MyBestfitLaserScaner();
    void run() override;
    void EnumScannerDevice();

    void startTimeCnt() { m_timerCnt = clock(); }
    int endTimeCnt() { return clock() - m_timerCnt; }

    // 连接点激光
    void scannerConnect();
    void scanerDisConnect();
    bool getConnectState();

    // 开启图像预览
    void startImagePreview();
    void stopImagePreview();

    // 开启数据抓取
    void startAcquisition();
    void laserOn(); // 打开激光
    void laserOff(); // 关闭激光
    void startLaserScanTask();

private:
    std::string  m_strScannerIP;
    std::string  m_strScannerTimeOut;
    void getConnectDeviceInfo();
    cv::Mat pointCloud2Image(std::vector<cv::Point2f> pointClouds);
};


#endif //BESTFITLASERDEMO_MYBESTFITLASERSCANER_H

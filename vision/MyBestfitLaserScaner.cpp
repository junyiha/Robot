//
// Created by csh_i on 2024/9/9.
//

#include "MyBestfitLaserScaner.h"

MyBestfitLaserScaner::MyBestfitLaserScaner( std::string scannerIP,  std::string laserName) {
    this->m_strScannerIP = scannerIP;
    this->m_strScannerTimeOut = "3000";
    m_dwScanner_Frequency = 100;
    this->laser_name = laserName;

}

MyBestfitLaserScaner::~MyBestfitLaserScaner() {
    // 关闭激光雷达传感器
    scanerDisConnect();
}

void MyBestfitLaserScaner::run() {

    if(!m_hScanner)
    {
        printf(" m_hScanner is Null\n");
        return ;
    }

    while(m_isRunning){

        if(m_threadQuit){
            break;
        }

        int iPicCnt = 0;
        int dataLength = EthernetScanner_GetXZIExtended(m_hScanner,
                                                        m_dScannerBufferX,
                                                        m_dScannerBufferZ,
                                                        m_iScannerBufferI, // 实际没有使用
                                                        m_iScannerBufferPeakWidth, // 实际没有使用
                                                        ETHERNETSCANNER_SCANXMAX * ETHERNETSCANNER_PEAKSPERCMOSSCANLINEMAX, // 缓存区的大小
                                                        &m_uScannerEncoder,
                                                        &m_ucScannerDigitalInputs,
                                                        1000, //等待新测量轮廓的阻塞时间值， 阻塞式
                                                        nullptr,
                                                        0,
                                                        &iPicCnt); //返回当前测量轮廓的图片计数器，该值用于控制接收到的轮廓的顺序。
        if (m_bGrabCloud_Once)
        {
            m_bGrabCloud_Once = false;
            m_iTaskType = NORUNINGTASK;
        }

        // no data
        if(dataLength <0)
        {
            printf("No Data\n");
            setDataValid(false);
            continue;
        }

        m_iLengthReceivedData = dataLength;
        if (iPicCnt - m_usPicCnt != 1) // 接收图片轮廓数据帧不连续
        {
            if( (m_usPicCnt != 0) && (m_usPicCnt < 0xEFFFFFFF) )
            {
                m_iPicCntErr++;
            }
        }

        m_usPicCnt = iPicCnt;
        if(dataLength>0){
            // 清空原始原始数据
            if(this->m_vecPointClouds.size() > 0)
            {
                this->m_vecPointClouds.clear();
            }

            for(int i = 0; i < dataLength; i++ )
            {
                m_vecPointClouds.push_back(cv::Point2f(m_dScannerBufferX[i], m_dScannerBufferZ[i]));
            }
        }

        // data size=0
        if(dataLength == 0)
        {
            qDebug() << "No Image Data, Cloud Data is NULL, Pic Counter:" << iPicCnt;
            setDataValid(false);
            continue;
        }

        if(this->m_vecPointClouds.size()>0){
            pointCloudsMutex.lock();
            this->m_vecPointClouds_ = this->m_vecPointClouds;
            pointCloudsMutex.unlock();
            setDataValid(true);
        }

        // 清空缓存区数据
        EthernetScanner_WriteData(m_hScanner,(char*)"SetClearCloudFifo\r", sizeof("SetClearCloudFifo\r"));
        Sleep(200);
    }

}
cv::Mat MyBestfitLaserScaner::pointCloud2Image(std::vector<cv::Point2f> pointClouds) {

    if(pointClouds.size()<=0){
        return cv::Mat();
    }

    float minY=1000000;
    float maxY=0;
    float minX=1000000;
    float maxX=0;

    for(int i=0;i<pointClouds.size();i++)
    {
        minY=std::min(pointClouds[i].y,minY); //这里的y就是z
        maxY=std::max(pointClouds[i].y,maxY);
        minX=std::min(pointClouds[i].x,minX);
        maxX=std::max(pointClouds[i].x,maxX);
    }
    this->minX= minX;
    this->minY= minY;

//    std::cout<<"minY:"<<minY<<"maxY"<<maxY<<"minX"<<minX<<"maxX"<<maxX<<std::endl;
    //规定row是z，col是x，且计算时要先减去最小值再乘10（放大）+偏置（为了看出整体形状）
    int rows=(maxY-minY)*10+100;
    int cols=(maxX-minX)*10+100;
    cv::Mat m(rows,cols,CV_8UC1,cv::Scalar(0));
    int bias = 50;
    for(int i=0;i<pointClouds.size();i++){
        m.at<char>((pointClouds[i].y-minY)*10+bias,(pointClouds[i].x-minX)*10+bias)=255; //at <类型> (行,列) [通道(如果有通道的话)]
    }
    return m;
}
void MyBestfitLaserScaner::scannerConnect() {

    if(m_hScanner == nullptr){
        m_hScanner = EthernetScanner_Connect((char*)m_strScannerIP.c_str(), (char*) "5566", atoi(m_strScannerTimeOut.c_str()));
        if(m_hScanner == nullptr){
            printf("The scanner %s connect fail\n",this->laser_name.c_str());
            return ;
        }else{
            printf("The scanner %s connect success\n",this->laser_name.c_str());
        }

        // 获取设备连接状态
        DWORD time = GetTickCount();
        int ConnectionStatus = 0;
        while(ConnectionStatus != ETHERNETSCANNER_TCPSCANNERCONNECTED)
        {
            //current state of the connection
            EthernetScanner_GetConnectStatus(m_hScanner, &ConnectionStatus);
            if(ConnectionStatus == ETHERNETSCANNER_TCPSCANNERCONNECTED)
            {
                m_bScannerConnectStatus = true;
            }
            //Detect the timeout
            if( (GetTickCount()-time) > 5000 )
            {
                m_hScanner = EthernetScanner_Disconnect(m_hScanner);
                printf("Error: EthernetScanner_Connect: Error in connection");
                m_hScanner = nullptr;
                return;
            }
            //获取设备连接信息
            getConnectDeviceInfo();
        }
    }
    else{
        printf("scanner has connected\n");
    }
}
void MyBestfitLaserScaner::getConnectDeviceInfo() {

    char retBuf[100] = {0,};
//    EthernetScanner_ReadData(m_hScanner, (char*)"GetSupplier", retBuf, 100, -1);
//    m_strSupplier = std::string((char*)retBuf);

//    EthernetScanner_ReadData(m_hScanner, (char*)"GetDeviceID", retBuf, 100, -1);
//    m_strDeviceID = std::string((char*)retBuf);

//    EthernetScanner_ReadData(m_hScanner, (char*)"GetFirmWareVersion", retBuf, 100, -1);
//    m_strFWVersion = std::string((char*)retBuf);

//    EthernetScanner_ReadData(m_hScanner, (char*)"GetIPAddress", retBuf, 100, -1);
//    m_strIPAddress = std::string((char*)retBuf);

//    EthernetScanner_ReadData(m_hScanner, (char*)"GetManufacturer", retBuf, 100, -1);
//    m_strGetManufacturer = std::string((char*)retBuf);

    // 获取Z轴测量的起始值,单位毫米
    EthernetScanner_ReadData(m_hScanner, (char*)"GetZStart", retBuf, 100, -1);
    m_strZstart = std::string((char*)retBuf);
    m_dZstart = atoi(m_strZstart.data());
    printf("The ZStart is %s\n", m_strZstart.data());

    // 获取Z轴测量的范围值,单位毫米
    EthernetScanner_ReadData(m_hScanner, (char*)"GetZRange", retBuf, 100, -1);
    m_strZrange = std::string((char*)retBuf);
    m_dZrange = atoi(m_strZrange.data());
    printf("The ZRange is %s\n", m_strZrange.data());

    // 获取Z轴测量的最佳值,单位毫米
    EthernetScanner_ReadData(m_hScanner, (char*)"GetZBest", retBuf, 100, -1);
    m_dZbest = atoi(std::string((char*)retBuf).data());
    printf("The ZBest is %s\n", std::string((char*)retBuf).data());

    // 获取X轴测量的起始值,单位毫米
    EthernetScanner_ReadData(m_hScanner, (char*)"GetXRangeAtStart", retBuf, 100, -1);
    m_strXRangeAtStart = std::string((char*)retBuf);
    m_dXRangeAtStart = atoi(m_strXRangeAtStart.data());
    printf("The XRangeAtStart is %s\n", m_strXRangeAtStart.data());

    // 获取X轴测量的结束值,单位毫米
    EthernetScanner_ReadData(m_hScanner, (char*)"GetXRangeAtEnd", retBuf, 100, -1);
    m_strXRangeAtEnd = std::string((char*)retBuf);
    m_dXRangeAtEnd = atoi(m_strXRangeAtEnd.data());
    printf("The XRangeAtEnd is %s\n", m_strXRangeAtEnd.data());

    // 获取X轴测量的最佳值,单位毫米
    EthernetScanner_ReadData(m_hScanner, (char*)"GetXRangeBest", retBuf, 100, -1);
    m_dXRangeBest = atoi(std::string((char*)retBuf).data());
    printf("The XRangeBest is %s\n", std::string((char*)retBuf).data());

    // 获取传感器当前设置的采集激光线点云个数,单位个
    EthernetScanner_ReadData(m_hScanner,
                             (char*)"GetGrabNum",
                             retBuf,
                             100,
                             -1);
    std::string grabNum = std::string((char*)retBuf);
    printf("The grabNum is %s\n", grabNum.c_str());

    // 获取激光器的亮度值
    EthernetScanner_ReadData(m_hScanner,
                             (char*)"GetLaserBrightness",
                             retBuf,
                             100,
                             -1);
    std::string laserBrightness= std::string((char*)retBuf);
    printf("The laserBrightness is %s\n", laserBrightness.c_str());


    // 获取当前图像获取的触发源
    EthernetScanner_ReadData(m_hScanner,
                             (char*)"GetTriggerSource",
                             retBuf,
                             100,
                             -1);
    std::string triggerSource= std::string((char*)retBuf);
    printf("The triggerSource is %s\n", triggerSource.c_str());

    // 获取当前图像的曝光时间
    EthernetScanner_ReadData(m_hScanner,
                             (char*)"GetExposureTime",
                             retBuf,
                             100,
                             -1);
    std::string exposureTime= std::string((char*)retBuf);
    printf("The exposureTime is %s\n", exposureTime.c_str());

    // 获取图像的帧速率
    EthernetScanner_ReadData(m_hScanner,
                             (char*)"GetFrameRate",
                             retBuf,
                             100,
                             -1);
    std::string frameRate= std::string((char*)retBuf);
    printf("The frameRate is %s\n", frameRate.c_str());

    // 获取设置的 ROI 左上角点在图像中相对于第一列的偏移
    EthernetScanner_ReadData(m_hScanner,
                             (char*)"GetROI1OffsetX",
                             retBuf,
                             100,
                             -1);
    std::string ROI1OffsetX= std::string((char*)retBuf);
    printf("The ROI1OffsetX is %s\n", ROI1OffsetX.c_str());


    EthernetScanner_ReadData(m_hScanner,
                             (char*)"GetROI1OffsetZ",
                             retBuf,
                             100,
                             -1);
    std::string ROI1OffsetZ = std::string((char*)retBuf);
    printf("The ROI1OffsetZ is %s\n", ROI1OffsetZ.c_str());


    EthernetScanner_ReadData(m_hScanner,
                             (char*)"GetROI1WidthX",
                             retBuf,
                             100,
                             -1);
    std::string ROI1WidthX= std::string((char*)retBuf);
    printf("The ROI1WidthX is %s\n", ROI1WidthX.c_str());


    EthernetScanner_ReadData(m_hScanner,
                             (char*)"GetROI1HeightZ",
                             retBuf,
                             100,
                             -1);
    std::string ROI1HeightZ= std::string((char*)retBuf);
    printf("The ROI1HeightZ is %s\n", ROI1HeightZ.c_str());
}
void MyBestfitLaserScaner::EnumScannerDevice() {

    char info[1024] = {0};
    startTimeCnt();
    if (EthernetScanner_EnumDevices(info, 1024) == ETHERNETSCANNER_OK)
    {
        QString infos(info);
        QStringList deviceList = infos.split(";");
        foreach (QString device, deviceList)
        {
            LaserScanerDeviceInfo l_info;
            l_info.Laser_name = laser_name;
            QStringList dInfo = device.split(",");
            if (dInfo.size() >= 4)
            {
                l_info.Ip = dInfo.at(1).toStdString();
                l_info.Mask = dInfo.at(2).toStdString();
                l_info.Gateway = dInfo.at(3).toStdString();
                l_info.SN = dInfo.at(0).toStdString();
            }
            m_vecScannerDeviceInfo.push_back(l_info);
        }
    }
    else
    {
        printf("Error: EthernetScanner_EnumDevices Failed!\n");
    }

}
void MyBestfitLaserScaner::scanerDisConnect() {

    if (m_hScanner)
    {
        startTimeCnt();
        EthernetScanner_Disconnect(m_hScanner);
        m_hScanner = nullptr;
    }
    m_bScannerConnectStatus = false;
    m_iTaskType = NORUNINGTASK;
    m_threadQuit = true;
}
void MyBestfitLaserScaner::startImagePreview() {
    if(m_hScanner == nullptr || !m_bScannerConnectStatus)
    {
        printf("error Sensor should be connected");
        return;
    }

    if (m_iTaskType == GRABCLOUDTASK)
    {
        printf("Error Sensor should be stop acquisition");
        return;
    }

    //set IMAGEPREVIEWTASK
    m_iTaskType = IMAGEPREVIEWTASK;

    //set event
//    neosmart::SetEvent(m_Event);

    //send start
    startTimeCnt();
    EthernetScanner_WriteData(m_hScanner, (char*) "SetIamgePreviewStart\r", sizeof("SetIamgePreviewStart\r"));
    printf("SetIamgePreviewStart");
}
void MyBestfitLaserScaner::stopImagePreview() {
    if(m_hScanner == nullptr || !m_bScannerConnectStatus)
    {
        printf("Error, Sensor should be connected!");
        return;
    }

    if (m_iTaskType == GRABCLOUDTASK)
    {
        printf("Error, Sensor should be stop acquisition!");
        return;
    }

    //reset event
//    neosmart::ResetEvent(m_Event);

    //send stop
    startTimeCnt();
    EthernetScanner_WriteData(m_hScanner, (char*) "SetIamgePreviewStop\r", sizeof("SetIamgePreviewStop\r"));
}
void MyBestfitLaserScaner::startAcquisition() {

    if(m_hScanner == nullptr || !m_bScannerConnectStatus)
    {
        printf("Error, Sensor should be connected");
        return;
    }

    if (m_iTaskType == IMAGEPREVIEWTASK)
    {
        printf("Error,Sensor should be stop image preview");
        return;
    }

    m_usPicCnt = 0;

    //set GRABCLOUDTASK
    m_iTaskType = GRABCLOUDTASK;  // 获取点云数据


    int mode = 2;  // 1: 一次获取点云数据, 2: 连续获取点云数据 3:不记数连续采集
    if(mode==0){
        m_bGrabCloud_Once = true;
        EthernetScanner_WriteData(m_hScanner, (char*) "SetAcquisitionStart=1\r", sizeof("SetAcquisitionStart=1\r"));
    }else{
        m_bGrabCloud_Once = false;
        EthernetScanner_WriteData(m_hScanner, (char*) "SetAcquisitionStart\r", sizeof("SetAcquisitionStart\r"));
    }



}
void MyBestfitLaserScaner::laserOn() {
    if(m_hScanner == nullptr || !m_bScannerConnectStatus)
    {
        printf("Error, Sensor should be connected");
        return;
    }
    EthernetScanner_WriteData(m_hScanner, (char*) "SetLaserOn\r", sizeof("SetLaserOn\r"));
}
void MyBestfitLaserScaner::laserOff() {

    if(m_hScanner == nullptr || !m_bScannerConnectStatus)
    {
        printf("Error, Sensor should be connected");
        return;
    }

    startTimeCnt();
    EthernetScanner_WriteData(m_hScanner, (char*) "SetLaserOff\r", sizeof("SetLaserOff\r"));
}
 cv::Mat MyBestfitLaserScaner::getResultMask()  {
    if(isDataValid()){
        std::vector<cv::Point_<float>> pointClouds;
        pointCloudsMutex.lock();
        pointClouds = this->m_vecPointClouds_;
        pointCloudsMutex.unlock();
        if(pointClouds.size()>0){
            cv::Mat image = pointCloud2Image(pointClouds);
            return image;
        }else{
            return cv::Mat();
        }
    }else{
        return cv::Mat();
    }
}
void MyBestfitLaserScaner::setResultMask(cv::Mat resultMask) {
    maskMutex.lock();
    MyBestfitLaserScaner::resultMask = resultMask;
    maskMutex.unlock();
}
bool MyBestfitLaserScaner::isDataValid()  {
    bool dataValid_;
    dataValidMutex.lock();
    dataValid_ = dataValid;
    dataValidMutex.unlock();
    return dataValid_;
}
void MyBestfitLaserScaner::setDataValid(bool dataValid) {
    dataValidMutex.lock();
    MyBestfitLaserScaner::dataValid = dataValid;
    dataValidMutex.unlock();
}

void MyBestfitLaserScaner::startLaserScanTask() {
    this->scannerConnect();
    if(this->m_bScannerConnectStatus){
        this->laserOn();
        this->startAcquisition();
        std::cout<<"layser:"<<this->laser_name<< "start successed!"<<std::endl;
    }else{
        std::cout<<"layser:"<<this->laser_name<< "start failed!"<<std::endl;
    }
}

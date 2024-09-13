//
// Created by csh_i on 2024/6/7.
//

#include "HKCameraControls.h"
#include <qDebug>
static  unsigned int __stdcall UpcomingThread(void* pUser)
{
    Sleep(3000);//为了等MV_CC_GetImageBuffer调用后再发送软触发命令
    printf("Trigger Software Once for MV_GrabStrategy_UpcomingImage\n");
    MV_CC_SetCommandValue(pUser, "TriggerSoftware");
    return 0;
}

void __stdcall HKCameraControls::ReconnectDevice(unsigned int nMsgType, void* pUser)
{
    std::cout << "nMsgType:" <<nMsgType<< std::endl;
    std::cout<<"MV_EXCEPTION_DEV_DISCONNECT:"<<MV_EXCEPTION_DEV_DISCONNECT<<std::endl;
    if(nMsgType == MV_EXCEPTION_DEV_DISCONNECT)
    {
        HKCameraControls* pThis = (HKCameraControls*)pUser;

        if (!pThis->open_status)
        {
            while (1)
            {
                int nRet = pThis->pstMvCamera->Open(&pThis->stDevInfo);
                if (MV_OK == nRet)
                {
                    pThis->pstMvCamera->RegisterExceptionCallBack(ReconnectDevice, pUser);
                    pThis->open_status = true;
                    break;
                }
                else
                {
                    Sleep(100);
                }
            }
        }
    }
}

int HKCameraControls::RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight )
{
    if ( NULL == pRgbData )
    {
        return MV_E_PARAMETER;
    }
    for (unsigned int j = 0; j < nHeight; j++)
    {
        for (unsigned int i = 0; i < nWidth; i++)
        {
            unsigned char red = pRgbData[j * (nWidth * 3) + i * 3];
            pRgbData[j * (nWidth * 3) + i * 3]     = pRgbData[j * (nWidth * 3) + i * 3 + 2];
            pRgbData[j * (nWidth * 3) + i * 3 + 2] = red;
        }
    }
    return MV_OK;
}

cv::Mat HKCameraControls::Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char * pData)
{
    cv::Mat srcImage;
    if ( pstImageInfo->enPixelType == PixelType_Gvsp_Mono8 )
    {
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, pData);
    }
    else if ( pstImageInfo->enPixelType == PixelType_Gvsp_RGB8_Packed )
    {
//        RGB2BGR(pData, pstImageInfo->nWidth, pstImageInfo->nHeight);
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3, pData);
    }
    else
    {
        printf("unsupported pixel format\n");
    }
    return srcImage;
}

HKCameraControls::HKCameraControls(const char* camName, const char* camIp, const char* ethIp) {

    this->pstMvCamera = new CMvCamera(); // 海康相机SDK类
    this->stHkDev.camIp = camIp;
    this->stHkDev.ethIp = ethIp;
    this->stHkDev.camName = camName;


    // 解析设备连接相关信息
    unsigned int nIp1, nIp2, nIp3, nIp4, nIp;
    sscanf_s(camIp, "%d.%d.%d.%d", &nIp1, &nIp2, &nIp3, &nIp4);
    nIp = (nIp1 << 24) | (nIp2 << 16) | (nIp3 << 8) | nIp4;
    this->stHkDev.stGigEDev.nCurrentIp = nIp;

    sscanf_s(ethIp, "%d.%d.%d.%d", &nIp1, &nIp2, &nIp3, &nIp4);
    nIp = (nIp1 << 24) | (nIp2 << 16) | (nIp3 << 8) | nIp4;
    this->stHkDev.stGigEDev.nNetExport = nIp;

    this->stHkDev.stDevInfo.nTLayerType = MV_GIGE_DEVICE;// ch:仅支持GigE相机 | en:Only support GigE camera
    this->stHkDev.stDevInfo.SpecialInfo.stGigEInfo = this->stHkDev.stGigEDev;
    // 初始化日志
    this->logger = spdlog::get("logger");

}

HKCameraControls::HKCameraControls(MV_CC_DEVICE_INFO pstDeviceInfo, const char *camName) {
    this->pstMvCamera = new CMvCamera(); // 海康相机SDK类
    this->stHkDev.camName = camName;
//    this->stHkDev.stDevInfo = pstDeviceInfo;
    memcpy(&stHkDev.stDevInfo, &pstDeviceInfo, sizeof(MV_CC_DEVICE_INFO));
    this->logger = spdlog::get("logger");
}



// 析构函数
HKCameraControls::~HKCameraControls(){

    if(NULL != this->pstMvCamera){
        this->pstMvCamera->StopGrabbing();
        delete this->pstMvCamera;
    }
    this->thread_runing = false;
    this->open_status = false;
    QThread::exit();
}

int HKCameraControls::openCamera() {

    int nRet=-1;
    if(NULL == this->pstMvCamera)  {
        this->logger->info("HKCameraControls is NULL");
        return -1;
    }

    bool res =  this->pstMvCamera->IsDeviceAccessible(&this->stHkDev.stDevInfo, 3);
    if(!res){
        this->logger->info(" Camera {} IsDeviceAccessible failed", this->stHkDev.camName);
        return -1;
    }

    nRet = this->pstMvCamera->Open(&this->stHkDev.stDevInfo);
    // 打开相机设置相关相关参数
    if(MV_OK == nRet){
        this->initializationCameraConfig();
        this->open_status = true;
        float fps = this->getFrameRate();
        this->logger->info(" CamName: {}, fps:{}", this->stHkDev.camName, fps);

    }else{
        this->logger->info("open camera {} failed", this->stHkDev.camName);
    }
    return nRet;
}

int HKCameraControls::closeCamera() {

    this->open_status = false;
    this->thread_runing = false;

    if(this->pstMvCamera!=NULL){
        this->pstMvCamera->StopGrabbing();
        this->pstMvCamera->Close();
        return 1;
    }
    return 0;
}

void HKCameraControls::run(){


    //  开启数据流抓取
    if(!this->open_status){
        this->logger->info("camera is not open");
        return ;
    }

    MV_FRAME_OUT stImageInfo = {0};
    unsigned char * pData = NULL;
    int grab_ret = this->pstMvCamera->StartGrabbing();
    if (MV_OK != grab_ret)
    {
        this->logger->info("Start grabbing fail. [0x%x]");
    }

    while(this->isRunning){

        int lossCount = 0;

        if(this->thread_runing){

            // 开启数据流抓取
            // 获取图像数据
            int  nRet =-1;
             //GetImageBuffer() 其取流缓存的分配是由sdk内部自动分配的,用户不用担心,   MV_CC_GetOneFrameTimeout() 接口是需要客户自行分配
            nRet = this->pstMvCamera->GetImageBuffer(&stImageInfo, 1000);   // 从缓存中获取图像数据,效率更高一些
            if (MV_OK == nRet)
            {
//                // 分配内存
                if(pData == nullptr){
                    pData = new unsigned char[stImageInfo.stFrameInfo.nFrameLen];
                }
                memcpy (pData, stImageInfo.pBufAddr, stImageInfo.stFrameInfo.nFrameLen);
                if (pData == NULL)
                {
                    this->logger->info("Allocate memory failed.");
                    continue;
                }
                cv:: Mat srcImage = Convert2Mat(&stImageInfo.stFrameInfo, pData);  // 将数据转换为Mat格式
                if(srcImage.channels() == 1){
                    cv::cvtColor(srcImage, srcImage, cv::COLOR_GRAY2RGB);
                }
//                std::string img_path = "E:/temp/";
//                img_path += this->stHkDev.camName;
                // 图像入对列
                if(!srcImage.empty()||srcImage.channels() == 3){
//                    cv::imwrite(img_path+".jpg", srcImage);
                    this->data_mutex.lock();
                    if(this->camera_queue.size() < 2){
                        this->camera_queue.push(srcImage);
                    }else{
                        this->camera_queue.pop();
                        this->camera_queue.push(srcImage);
                    }
                    this->data_mutex.unlock();
                }else{
                    this->logger->info(" HKCameraControls::  camName: {}, get image failed", this->stHkDev.camName);
                }
                this->pstMvCamera->FreeImageBuffer(&stImageInfo);  // 释放图像缓存
            }else{
                // 待处理   软触发模式处理策略
                this->logger->info("GetImageBuffer fail {}", this->stHkDev.camName);
                QThread::msleep(100);
                lossCount++;
                if(lossCount>50){
                    this->openCamera();
                    this->logger->info("{} is reconnected!", this->stHkDev.camName);
                    lossCount = 0;
                }
                continue;
            }
        }
        QThread::msleep(200);
    }

    delete[] pData;
    this->pstMvCamera->StopGrabbing();
    this->pstMvCamera->Close();
}

cv::Mat HKCameraControls::getFrame() {
    cv::Mat srcImage;
    this->data_mutex.lock();
    if(this->camera_queue.size()>0){
        srcImage = this->camera_queue.back();
    }else{
        this->logger->info(" HKCameraControls::getFrame()  camName: {}, get frame failed", this->stHkDev.camName);
    }
    this->data_mutex.unlock();
    return srcImage;
}

void HKCameraControls::getCameraConfigInfo() {

    // 获取触发模式
    int nRet = -1;
    MVCC_ENUMVALUE stEnumValue = {0};
    nRet = this->pstMvCamera->GetEnumValue("TriggerMode", &stEnumValue);
    if (MV_OK == nRet){
        printf("TriggerMode: %d\n", stEnumValue.nCurValue);
    }

    // 获取信息增益参数
    GetGain();
    // 获取帧速率
    getFrameRate();
    // 获取心跳时间
    getGevHeartbeatTimeout();
    // 获取曝光时间
    getExposureTime();
    // 获取触发源
    MVCC_ENUMVALUE stEnumValue_TriggerSource = {0};
    nRet = this->pstMvCamera->GetEnumValue("TriggerSource", &stEnumValue_TriggerSource);
    if (MV_OK == nRet){
        printf("TriggerSource: %d\n", stEnumValue_TriggerSource.nCurValue);
    }

    // 获取GevSCPD参数
    getGevSCPD();

    // 获取图像分辨率
    cv::Size imgSize;
    imgSize = this->GetImageResolution();
    if(imgSize.width > 0 && imgSize.height > 0){
        printf("ImageResolution: %d x %d\n", imgSize.width, imgSize.height);
    }else{
        printf("Get ImageResolution fail! nRet [0x%x]\n", nRet);
    }

    // 获取SDK版本号
    nRet = this->GetSDKVersion();
    printf("SDK Version: [x%x]\n", nRet);

}

float HKCameraControls::getExposureTime() {
    int nRet= -1;
    float expposureTime = 0;
    MVCC_FLOATVALUE stFloatValueExposureTime = {0};
    nRet = pstMvCamera->GetFloatValue("ExposureTime", &stFloatValueExposureTime);
    if(MV_OK == nRet){
        printf("ExposureTime: %f\n", stFloatValueExposureTime.fCurValue);
        expposureTime = stFloatValueExposureTime.fCurValue;
    }
    return expposureTime;
}

float HKCameraControls::GetGain() {
    int nRet = -1;
    float  gain = 0;
    MVCC_FLOATVALUE stFloatValue = {0};
    nRet = pstMvCamera->GetFloatValue("Gain", &stFloatValue);
    if (MV_OK == nRet){
        printf("Gain: %f\n", stFloatValue.fCurValue);
        gain = stFloatValue.fCurValue;
    }
    return gain;
}

float HKCameraControls::getFrameRate() {
    // 获取帧速率
    int nRet = -1;
    float  frameRate = 0;
    MVCC_FLOATVALUE stFloatValue_FrameRate = {0};
    nRet = pstMvCamera->GetFloatValue("ResultingFrameRate", &stFloatValue_FrameRate);
    if (MV_OK == nRet){
        printf("ResultingFrameRate: %f\n", stFloatValue_FrameRate.fCurValue);
        frameRate = stFloatValue_FrameRate.fCurValue;
    }
    return frameRate;
}

int HKCameraControls::getGevHeartbeatTimeout() {// 获取心跳超时时间
    int nRet = -1;
    int nTimeout = 0;
    _MVCC_INTVALUE_EX_T stIntValue_Timeout= {0};
    nRet = pstMvCamera->GetIntValue("GevHeartbeatTimeout", &stIntValue_Timeout);
    if (MV_OK == nRet){
        printf("GevHeartbeatTimeout: %f\n", stIntValue_Timeout.nCurValue);
        nTimeout = stIntValue_Timeout.nCurValue;
    }
    return nTimeout;
}

int HKCameraControls::getGevSCPD() {
    int nRet = -1;
    _MVCC_INTVALUE_EX_T stIntValueSCPD = {0};
    nRet = pstMvCamera->GetIntValue("GevSCPD", &stIntValueSCPD);
    if(MV_OK == nRet){
        printf("GevSCPD: %d\n", stIntValueSCPD.nCurValue);
    }else{
        printf("Get GevSCPD fail! nRet [0x%x]\n", nRet);
    }
    return stIntValueSCPD.nCurValue;
}

void HKCameraControls::start_image_acquisition(std::string mode) {

    int nRet = -1;
    if(mode== "Trigger"){
        nRet = this->pstMvCamera->SetEnumValueByString("TriggerMode", "On" );
        // 触发模式
    }else{ // 连续模式
        nRet = this->pstMvCamera->SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF );

    }
    if (MV_OK != nRet)
    {
        printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
    }
}

cv::Mat HKCameraControls::getOneImage() {

    // 通过触发模式获取图像
    cv::Mat resImage;
    int nRet = -1;
    nRet = this->pstMvCamera->SetEnumValueByString( "TriggerSource", "Software" );
    if (MV_OK != nRet)
    {
        printf("Set Trigger Source fail! nRet [0x%x]\n", nRet);
    }


    // 设置触发策略
    unsigned int nGrabStrategy = 0;
    nGrabStrategy = MV_GrabStrategy_UpcomingImage;     // 共计4种触发策略
    std::cout << "nGrabStrategy: "<< nGrabStrategy << std::endl;
    nRet =this->pstMvCamera->SetGrabStrategy(nGrabStrategy);
    if (MV_OK != nRet){
        printf("Set Grab Strategy fail! nRet [0x%x]\n", nRet);
    }

//    unsigned int nImageNodeNum = 5;
//    nRet = this->pstMvCamera->SetImageNodeNum(nImageNodeNum);
//    if (MV_OK != nRet) {
//        printf("Set number of image node fail! nRet [0x%x]\n", nRet);
//    }


    unsigned int nThreadID = 0;
    void* hThreadHandle = (void*) _beginthreadex( NULL , 0 , UpcomingThread , this->pstMvCamera->m_hDevHandle, 0 , &nThreadID );

    // 启动采集
    nRet = this->pstMvCamera->StartGrabbing();
    // 发送触发命令
    nRet =  this->pstMvCamera->SetCommandValue("TriggerSoftware");
//    for (unsigned int i = 0;i < nImageNodeNum;i++)
//    {
//        nRet = this->pstMvCamera->SetCommandValue("TriggerSoftware");
//        if (MV_OK != nRet)
//        {
//            printf("Send Trigger Software command fail! nRet [0x%x]\n", nRet);
//            break;
//        }
//        Sleep(500);//如果帧率过小或TriggerDelay很大，可能会出现软触发命令没有全部起效而导致取不到数据的情况
//    }


    if (MV_OK != nRet){
        printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
    }

    MV_FRAME_OUT stOutFrame = {0};
    nRet = this->pstMvCamera->GetImageBuffer(&stOutFrame, 5000); //需要比较大的超时时间来获取即将到达的这帧数据
    if(nRet == MV_OK){
        printf("Get One Frame: Width[%d], Height[%d], FrameNum[%d]\n",
               stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum);
        unsigned char * pData = NULL;
        if(pData == nullptr){
            pData = new unsigned char[stOutFrame.stFrameInfo.nFrameLen];
        }
        memcpy  (pData, stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nFrameLen);
        if (pData == NULL)
        {
            printf("Allocate memory failed.\n");
        }
        resImage = Convert2Mat(&stOutFrame.stFrameInfo, pData);  // 将数据转换为Mat格式
        // 释放图像缓存
        nRet = this->pstMvCamera->FreeImageBuffer(&stOutFrame);
        if(nRet != MV_OK)
        {
            printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
        }
    }else{
        printf("Get One Frame fail! nRet [0x%x]\n", nRet);
    }
    this->pstMvCamera->StopGrabbing();

    return resImage;
}

int HKCameraControls::SetExposureTime() {

    // 曝光时间决定了传感器接收光线的时间长度。
    // 较长的曝光时间可以捕获更多的光线，适用于光线较暗的环境，但可能导致运动模糊。
    // 在运动场景中，应选择较短的曝光时间以避免拖影现象。


    // ch:调节这两个曝光模式，才能让曝光时间生效
    // en:Adjust these two exposure mode to allow exposure time effective
    int nRet;
    nRet = this->pstMvCamera->SetEnumValue("ExposureMode", MV_EXPOSURE_MODE_TIMED);
    if (MV_OK != nRet)
    {
        logger->info("Camera {} Set Exposure Mode fail! nRet!", this->stHkDev.camName);
        return nRet;
    }

    // MV_EXPOSURE_AUTO_MODE_OFF 关闭  ||  MV_EXPOSURE_AUTO_MODE_CONTINUOUS 连续 曝光  ||  MV_EXPOSURE_AUTO_MODE_ONCE 一次曝光
    nRet =  this->pstMvCamera->SetEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
    if (MV_OK != nRet){
        this->logger->info("Camrea {} Set Exposure Auto fail! nRet", this->stHkDev.camName);
    }

//         // 自定义参数曝光时间
//            nRet =  this->pstMvCamera->SetEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
//                if (MV_OK != nRet){
//                    logger->info("Set Exposure Auto fail! nRet [0x%x]\n", nRet);
//            }
//
//            double  m_dExposureEdit = 2000;
//            this->pstMvCamera->SetFloatValue("ExposureTime", (float)m_dExposureEdit);

    return nRet;



//        return this->pstMvCamera->SetFloatValue("ExposureTime", (float)m_dExposureEdit);

}

void HKCameraControls::initializationCameraConfig() {

    int nRet= -1;

    // 1.0  ch:探测网络最佳包大小(只对GigE相机有效)
    if (this->stHkDev.stDevInfo.nTLayerType == MV_GIGE_DEVICE)
    {
        unsigned int nPacketSize = 0;
        nRet = this->pstMvCamera->GetOptimalPacketSize(&nPacketSize);
        if (nRet == MV_OK)
        {
            nRet = this->pstMvCamera->SetIntValue("GevSCPSPacketSize",nPacketSize);
            if(nRet != MV_OK)
            {
                this->logger->info("Warning: Set Packet Size fail nRet {:02x}!", nRet);
            }
        }
        else
        {
            this->logger->info("Warning: Get Packet Size fail nRet {:02x}!", nPacketSize);
        }
    }

    // 2.0  设置心跳超时时间
    this->setHeartBeatTimeOut();

    // 3. 0设置像素格式
    nRet = this->pstMvCamera->SetEnumValue("PixelFormat", PixelType_Gvsp_RGB8_Packed);
    if (nRet != MV_OK)
    {
        this->logger->info("Camrea {} Warning: Set PixelFormat fail nRet !", this->stHkDev.camName);
    }

    // 4.0 设置曝光时间
//    this->SetExposureTime();

    // 5.0 设置增益
//    this->SetGain();

    // 6.0 设置帧率
    this->SetFrameRate();

    // 7.0 设置触发模式
//    nRet = this->pstMvCamera->SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF);
//    if (nRet != MV_OK){
//        this->logger->info("Camera {} Set Trigger Mode fail! nRet !", this->stHkDev.camName);
//    }

    //8.0 设置图像分辨率
//     this->SetImageResolution();

    //9.0 设置GevSCPD
    this->setGevSCPD();

}

int HKCameraControls::SetGain() {

    // 增益控制传感器对接收到的光线信号的放大程度。增益越高，图像越亮，但同时噪声也会增加。
    // 在光线不足的情况下可以适当提高增益，但要注意过高的增益可能会导致图像质量下降

    // ch:设置增益前先把自动增益关闭，失败无需返回
    //en:Set Gain after Auto Gain is turned off, this failure does not need to return
    int nRet;

    nRet = this->pstMvCamera->SetEnumValue("GainAuto", 2); // 0:Off, 1:On, 2:Auto  自动开启增益
    if (MV_OK != nRet)
    {
        logger->info("Set Gain Auto fail! nRet [0x%x]\n", nRet);
        return nRet;
    }
//    nRet = this->pstMvCamera->SetFloatValue("Gain", (float)m_dGainEdit);
//    if (MV_OK != nRet){
//        logger->info("Set Gain fail! nRet [0x%x]\n", nRet);
//    }
    return nRet;
}

int HKCameraControls::SetFrameRate() {

    float fFrameRate ;
    int nRet = -1;
    nRet = this->pstMvCamera->SetBoolValue("AcquisitionFrameRateEnable", true);
    if (MV_OK != nRet)
    {
        this->logger->info("Set FrameRate Enable fail! nRet");
        return nRet;
    }else{
        nRet = this->pstMvCamera->SetFloatValue("AcquisitionFrameRate", (float)m_dFrameRateEdit);
    }
////    fFrameRate = this->getFrameRate();
//    if(fFrameRate!=this->m_dFrameRateEdit){
//
//    }
    return nRet;
}

int HKCameraControls::SetImageResolution() {
    // 获取当前图像分辨率配置
    cv::Size imgSize;
    int nRet = -1;
    imgSize = this->GetImageResolution();
    if(imgSize.width!=this->imgWidth || imgSize.height!=this->imgHeight){
        nRet = this->pstMvCamera->SetIntValue("Width", imgWidth);
        if (MV_OK != nRet){
            this->logger->info("Set Width fail! nRet");
        }
        nRet = this->pstMvCamera->SetIntValue("Height", imgHeight);
        if(MV_OK != nRet){
            this->logger->info("Set Height fail! nRet");
        }
    }
    return nRet;
}

cv::Size HKCameraControls::GetImageResolution() {

    int  nRet;
    cv::Size imgSize(0,0);
    _MVCC_INTVALUE_EX_T stIntValue_Width= {0};
    nRet = this->pstMvCamera->GetIntValue("Width", &stIntValue_Width);
    if (MV_OK != nRet)
    {
        this->logger->info("Get Width fail! nRet [0x%x]\n", nRet);
    }
    imgSize.width = stIntValue_Width.nCurValue;
    _MVCC_INTVALUE_EX_T stIntValue_Height= {0};
    nRet = this->pstMvCamera->GetIntValue("Height", &stIntValue_Height);
    if(MV_OK != nRet){
        this->logger->info("Get Height fail! nRet [0x%x]\n", nRet);
    }
    imgSize.height = stIntValue_Height.nCurValue;
    return imgSize;
}

int HKCameraControls::setGevSCPD() {

    // 先获取
    int nRet =-1;
    int currentSCPD = 0;
    currentSCPD = this->getGevSCPD();
    if(currentSCPD!=this->stIntValue_SCPD){
        nRet = this->pstMvCamera->SetIntValue("GevSCPD", this->stIntValue_SCPD);
        if (MV_OK != nRet)
        {
            this->logger->info("Camrea {} Set Width GevSCPD!", this->stHkDev.camName);
        }
    }
    return nRet;
}

int HKCameraControls::setHeartBeatTimeOut() {
    int nRet = 0;
    nRet = this->pstMvCamera->SetIntValue("GevHeartbeatTimeout", stIntValue_HBTO);
    if (nRet != MV_OK)
    {
        this->logger->info("Camrea {} Warning: Set GevHeartbeatTimeout fail nRet!", this->stHkDev.camName);
    }
    return nRet;
}

int HKCameraControls::GetSDKVersion()
{
    return MV_CC_GetSDKVersion();
}

cv::Mat HKCameraControls::getImageFromeQueue() {

    cv::Mat image;
    this->data_mutex.lock();
    if(this->camera_queue.size() > 0){
        image = this->camera_queue.back();
    }
    this->data_mutex.unlock();
    return image;
}

void HKCameraControls::closeThread() {
    this->closeCamera();
    this->isRunning = false;
    QThread::wait();

}

bool HKCameraControls::cameraIsAccessible() {
    return  this->pstMvCamera->IsDeviceAccessible(&this->stHkDev.stDevInfo, 3);
}








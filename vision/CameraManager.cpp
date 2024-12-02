//
// Created by csh_i on 2024/6/25.
//

#include "CameraManager.h"

bool CameraManager::openCamera(std::string camera_name) {

    if(this->cameraList.find(camera_name) != this->cameraList.end()){
        this->cameraList[camera_name]->openCamera();
        if(this->cameraList[camera_name]->open_status){
            this->cameraList[camera_name]->start();
            this->logger->info("open camera {} success", camera_name);
            return true;
        }else{
            this->logger->error("open camera {} failed", camera_name);
            return false;
        }
    }else{
        this->logger->error("camera {} not exist", camera_name);
        return false;
    }
}


CameraManager::CameraManager() {


    // ��־�ļ�
    this->logger = spdlog::get("logger");
    //this->serial2names = {
    //        {"DA2720028", "cam_1"},//2
    //        {"DA2720133", "cam_2"},//1
    //        {"DA2720068", "cam_3"},//4
    //        {"DA2720132", "cam_4"},//3
    //        {"DA2720067", "cam_5"},//6
    //        {"DA2720078", "cam_6"},//5
    //};
    this->serial2names = {
            {"DA2720132", "cam_1"}, // 1 --> 4
            {"DA2720068", "cam_2"}, // 2 --> 3
            {"DA2720133", "cam_3"}, // 3 --> 2
            {"DA2720028", "cam_4"}, // 4 --> 1
            {"DA2720078", "cam_5"}, // 6 --> 5
            {"DA2720067", "cam_6"}, // 5 --> 6
    };
    // this->cameraInfoMap ��ʼ��
    MV_CC_DEVICE_INFO null_dev;
    memset(&null_dev, 0, sizeof(MV_CC_DEVICE_INFO));
    this->cameraInfoMap = {
            {"cam_1",  null_dev},
            {"cam_2",  null_dev},
            {"cam_3",  null_dev},
            {"cam_4",  null_dev},
            {"cam_5",  null_dev},
            {"cam_6",  null_dev}
    };

    this->camera_info = {
//            {"cam_1",  "192.168.1.118"},
//            {"cam_2",  "192.168.1.115"},
//            {"cam_2",  "192.168.1.116"},
//            {"cam_3",  "192.168.1.117"},
//            {"cam_4",  "192.168.1.112"},
//            {"cam_05",  "192.168.1.114"},
//            {"cam_06",  "192.168.1.115"}
    };

    //�Զ�ö�پ������ڵ��������cameraInfoMap���и�ֵ
    this->getDeviceList();

    if(this->m_stDevList.nDeviceNum==0){
        this->logger->info("no device found");
//        return ;
    }
    // ��ʼ���������
    for (auto &camera_info : this->cameraInfoMap) {
//        this->cameraList[camera_info.first] = new HKCameraControls(camera_info.first.data(), camera_info.second.data(), this->ethIp);
        this->cameraList[camera_info.first] = new HKCameraControls(camera_info.second, camera_info.first.data());
//        this->cameraList[camera_info.first]->start();
    }
    // �����߳�, ���������
    this->openCameraAll();

}

bool CameraManager::closeCamera(std::string camera_name) {
    if(this->cameraList.find(camera_name) != this->cameraList.end()){
        this->cameraList[camera_name]->closeCamera();
        this->logger->info("close camera {} success", camera_name);
        return true;
    }else{
        this->logger->error("camera {} not exist", camera_name);
        return false;
    }
}

cv::Mat CameraManager::getImage(std::string camera_name) {
    cv::Mat image;
    if(this->cameraList.find(camera_name) != this->cameraList.end()){
        image = this->cameraList[camera_name]->getFrame();
    }
    return image;
}

void CameraManager::openCameraAll() {
    this->start();
}

CameraManager::~CameraManager() {
     for (auto &camera : this->cameraList) {
         this->cameraList[camera.first]->closeCamera();
         delete this->cameraList[camera.first];
     }
}

void CameraManager::getImageAll() {
    // ��ȡ�������ͼ��
    this->dataMutex.lock();
    if(this->cameraImages.size()>0){
        this->cameraImages.clear();
    }

    for (auto &camera : this->cameraList) {
        cv::Mat image = this->cameraList[camera.first]->getFrame();
        if(!image.empty()){
            this->cameraImages[camera.first] = image;
        }else{
            //image = cv::imread("C:\\Users\\csh_i\\MVS\\Data\\Image_20240710150737081.bmp"); //��һ����
            //this->cameraImages[camera.first] = image;
            //this->logger->error("get image from camera {} failed", camera.first);
        }
    }
    this->dataMutex.unlock();
}

bool CameraManager::closeCameraAll() {
    for (auto &camera : this->cameraList) {
        this->cameraList[camera.first]->closeCamera();
        this->logger->info("close camera {} successed", camera.first);
    }
    return 0;
}

std::map<std::string, cv::Mat> CameraManager::getCameraImages() {
    std::map<std::string, cv::Mat> cameraImages;
    this->dataMutex.lock();
    if(this->cameraImages.size()>0){
        cameraImages = this->cameraImages;
    }
    this->dataMutex.unlock();
    return cameraImages;
}

void CameraManager::getDeviceList() {
    int nRet;
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_stDevList);
    if (nRet != MV_OK){
        this->logger->info("MV_CC_EnumDevices failed!");
        return ;
    }
    this->logger->info("find {} device", m_stDevList.nDeviceNum);
    // �Զ�������
    for (int i = 0; i < m_stDevList.nDeviceNum; i++) {
        MV_CC_DEVICE_INFO stDevInfo;
        memcpy(&stDevInfo, m_stDevList.pDeviceInfo[i], sizeof(MV_CC_DEVICE_INFO));
        std::string serial_number =  reinterpret_cast<const char*>(m_stDevList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chSerialNumber);
        this->cameraInfoMap[this->serial2names[serial_number]] = stDevInfo;
   }

}

void CameraManager::closeAllCameraThread() {

    if(this->cameraList.size()>0){
        for (auto &camera : this->cameraList) {
            this->cameraList[camera.first]->closeThread();
        }
    }
}

void CameraManager::run() {

    // �����߳�, ���������
    for (auto &camera : this->cameraList) {
        this->cameraList[camera.first]->openCamera();
        if(!this->cameraList[camera.first]->open_status){
            this->logger->info("open camera {} failed", camera.first);
            continue ;
        }else{
            this->cameraList[camera.first]->start();
            this->logger->info("open camera {} successed", camera.first);
        }

    }
}

std::vector<bool> CameraManager::checkCameraIsAccessible() {
    std::vector<bool> isAccessible;
    for(const auto &camera : this->cameraList){
        size_t index = camera.first.find("_");
        int number = camera.first[index+1]-'0';
        bool is_Acc = camera.second->cameraIsAccessible();
        isAccessible[number-1] = is_Acc;
    }
    return isAccessible;
}

std::vector<bool> CameraManager::getCameraOpenedInfo() {
    std::vector<bool> isOpened(this->cameraList.size());
    for(const auto &camera : this->cameraList){
        size_t index = camera.first.find("_");
        int number = camera.first[index+1]-'0';
        isOpened[number-1] = camera.second->open_status;
    }
    return isOpened;
}

bool CameraManager::camerasIsOpened() {
    std::vector<bool> res = this->getCameraOpenedInfo();
    for(int i=0; i<res.size(); i++){
        if(!res[i]){
            return false;
        }
    }
    return true;
}





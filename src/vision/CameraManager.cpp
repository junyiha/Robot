//
// Created by csh_i on 2024/6/25.
//

#include "CameraManager.h"

bool CameraManager::openCamera(std::string camera_name)
{

    if (this->cameraList.find(camera_name) != this->cameraList.end())
    {
        this->cameraList[camera_name]->openCamera();
        if (this->cameraList[camera_name]->open_status)
        {
            this->cameraList[camera_name]->start();
            this->logger->info("open camera {} success", camera_name);
            return true;
        }
        else
        {
            this->logger->error("open camera {} failed", camera_name);
            return false;
        }
    }
    else
    {
        this->logger->error("camera {} not exist", camera_name);
        return false;
    }
}


CameraManager::CameraManager()
{


    // ��־�ļ�
    this->logger = spdlog::get("logger");

    this->serial2names = {
            {"DA2603734", "LineCam_1"},  // ok
            {"DA2720095", "LineCam_2"},  // ok
            {"DA2720134", "LineCam_3"}, // ok
            {"DA2720128", "LineCam_4"},  // ok
            {"DA2461731", "LineCam_5"},  // ok
            {"DA2461742", "LineCam_6"},  // ok
            {"DA3218695", "HoleCam_7"},  // ok
            {"DA3218718", "HoleCam_8"},  // ok
            {"DA3218690", "HoleCam_9"},
            {"DA2723075", "HoleCam_10"}
    };

    // this->cameraInfoMap ��ʼ��
    MV_CC_DEVICE_INFO null_dev;
    memset(&null_dev, 0, sizeof(MV_CC_DEVICE_INFO));
    this->cameraInfoMap = {
            {"LineCam_1",  null_dev},
            {"LineCam_2",  null_dev},
            {"LineCam_3",  null_dev},
            {"LineCam_4",  null_dev},
            {"LineCam_5",  null_dev},
            {"LineCam_6",  null_dev},
            {"HoleCam_7",  null_dev},
            {"HoleCam_8",  null_dev},
            {"HoleCam_9",  null_dev},
            {"HoleCam_10",  null_dev},
    };

    this->imageCorrectMap = {
            {"LineCam_1",  0},
            {"LineCam_2",  2},
            {"LineCam_3",  0},
            {"LineCam_4",  2},
            {"LineCam_5",  3},
            {"LineCam_6",  1},
            {"HoleCam_7",  0},
            {"HoleCam_8",  0},
            {"HoleCam_9",  0},
            {"HoleCam_10",  0},
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

    if (this->m_stDevList.nDeviceNum == 0)
    {
        this->logger->info("no device found");
        //        return ;
    }
    // ��ʼ���������
    for (auto& camera_info : this->cameraInfoMap)
    {
        //        this->cameraList[camera_info.first] = new HKCameraControls(camera_info.first.data(), camera_info.second.data(), this->ethIp);
        this->cameraList[camera_info.first] = new HKCameraControls(camera_info.second, camera_info.first.data());
        //        this->cameraList[camera_info.first]->start();
    }
    // �����߳�, ���������
    this->openCameraAll();

}

bool CameraManager::closeCamera(std::string camera_name)
{
    if (this->cameraList.find(camera_name) != this->cameraList.end())
    {
        this->cameraList[camera_name]->closeCamera();
        this->logger->info("close camera {} success", camera_name);
        return true;
    }
    else
    {
        this->logger->error("camera {} not exist", camera_name);
        return false;
    }
}

cv::Mat CameraManager::getImage(std::string camera_name)
{
    cv::Mat image;
    if (this->cameraList.find(camera_name) != this->cameraList.end())
    {
        image = this->cameraList[camera_name]->getFrame();
    }
    return image;
}


void CameraManager::getImageAll(std::string camType)
{
    // ��ȡ�������ͼ��
    this->dataMutex.lock();
    if (this->cameraImages.size() > 0)
    {
        this->cameraImages.clear();
    }

    for (auto& camera : this->cameraList)
    {
        cv::Mat image = this->cameraList[camera.first]->getFrame();

        if (camType == "Line")
        { // �˳���λ���
            if (camera.first.find("HoleCam") != std::string::npos)
            {
                continue;
            }
        }

        if (camType == "Hole")
        {// �˳���λ���
            if (camera.first.find("LineCam") != std::string::npos)
            {
                continue;
            }
        }

        if (!image.empty())
        {
            this->cameraImages[camera.first] = image;
        }
        else
        {
            image = cv::Mat();
            //            this->cameraImages[camera.first] = image;
                        // this->logger->error("********************get image from camera {} failed**************************", camera.first);
        }
    }
    this->dataMutex.unlock();
}


void CameraManager::openCameraAll()
{
    this->start();
}

CameraManager::~CameraManager()
{
    for (auto& camera : this->cameraList)
    {
        this->cameraList[camera.first]->closeCamera();
        delete this->cameraList[camera.first];
    }
}

void CameraManager::getImageAllNonResize(std::string camType)
{
    // ��ȡ�������ͼ��
    this->dataMutex.lock();
    if (this->cameraImages.size() > 0)
    {
        this->cameraImages.clear();
    }

    for (auto& camera : this->cameraList)
    {
        cv::Mat image = this->cameraList[camera.first]->getFrame();

        if (camType == "Line")
        { // �˳���λ���
            if (camera.first.find("HoleCam") != std::string::npos)
            {
                continue;
            }
        }

        if (camType == "Hole")
        {// �˳���λ���
            if (camera.first.find("LineCam") != std::string::npos)
            {
                continue;
            }
        }

        if (!image.empty())
        {
            this->cameraImages[camera.first] = image;
        }
        else
        {
            image = cv::Mat();
            //            this->cameraImages[camera.first] = image;
        }
    }
    this->dataMutex.unlock();
}

bool CameraManager::closeCameraAll()
{
    for (auto& camera : this->cameraList)
    {
        this->cameraList[camera.first]->closeCamera();
        this->logger->info("close camera {} successed", camera.first);
    }
    return 0;
}

std::map<std::string, cv::Mat> CameraManager::getCameraImages(std::string camType)
{
    std::map<std::string, cv::Mat> cameraImages;
    this->dataMutex.lock();
    if (this->cameraImages.size() > 0)
    {
        for (auto& item : this->cameraImages)
        {
            if (camType == "Line")
            { // �˳���λ���
                if (item.first.find("HoleCam") != std::string::npos)
                {
                    continue;
                }
            }

            if (camType == "Hole")
            {// �˳���λ���
                if (item.first.find("LineCam") != std::string::npos)
                {
                    continue;
                }
            }
            cameraImages[item.first] = item.second;
        }
    }
    this->dataMutex.unlock();
    return cameraImages;
}

std::map<std::string, cv::Mat> CameraManager::getCameraImagesResized(std::string camType)
{
    std::map<std::string, cv::Mat> cameraImages;
    this->dataMutex.lock();
    if (this->cameraImagesResize.size() > 0)
    {
        for (auto& item : this->cameraImagesResize)
        {
            if (camType == "Line")
            { // �˳���λ���
                if (item.first.find("HoleCam") != std::string::npos)
                {
                    continue;
                }
            }

            if (camType == "Hole")
            {// �˳���λ���
                if (item.first.find("LineCam") != std::string::npos)
                {
                    continue;
                }
            }
            cameraImages[item.first] = item.second;
        }
    }
    this->dataMutex.unlock();
    return cameraImages;
}

void CameraManager::getDeviceList()
{
    int nRet;
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_stDevList);
    if (nRet != MV_OK)
    {
        this->logger->info("MV_CC_EnumDevices failed!");
        return;
    }
    this->logger->info("find {} device", m_stDevList.nDeviceNum);
    // �Զ�������
    for (int i = 0; i < m_stDevList.nDeviceNum; i++)
    {
        MV_CC_DEVICE_INFO stDevInfo;
        memcpy(&stDevInfo, m_stDevList.pDeviceInfo[i], sizeof(MV_CC_DEVICE_INFO));
        std::string serial_number = reinterpret_cast<const char*>(m_stDevList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chSerialNumber);
        if (this->serial2names.find(serial_number) == this->serial2names.end())
        {
            this->logger->error("camera {} not in serial2names", serial_number);
            continue;

        }
        this->cameraInfoMap[this->serial2names[serial_number]] = stDevInfo;
    }

}

void CameraManager::closeAllCameraThread()
{

    if (this->cameraList.size() > 0)
    {
        for (auto& camera : this->cameraList)
        {
            this->cameraList[camera.first]->closeThread();
        }
    }
}

void CameraManager::run()
{

    // �����߳�, ���������
    for (auto& camera : this->cameraList)
    {
        this->cameraList[camera.first]->openCamera();
        if (!this->cameraList[camera.first]->open_status)
        {
            this->logger->info("open camera {} failed", camera.first);
            continue;
        }
        else
        {
            this->cameraList[camera.first]->start();
            this->logger->info("open camera {} successed", camera.first);
        }
    }

    // 获取相机连接状态
    std::vector<bool> connectStatus;
    while (true)
    {
        connectStatus = this->getCameraOpenedInfo();
        conMutex.lock();
        if (this->cameraConnectStatus.size() > 0)
        {
            this->cameraConnectStatus.clear();
        }
        this->cameraConnectStatus = connectStatus;
        conMutex.unlock();
        QThread::msleep(300);
    }
}

std::vector<bool> CameraManager::checkCameraIsAccessible()
{
    std::vector<bool> isAccessible;
    for (const auto& camera : this->cameraList)
    {
        size_t index = camera.first.find("_");
        int number = camera.first[index + 1] - '0';
        bool is_Acc = camera.second->cameraIsAccessible();
        isAccessible[number - 1] = is_Acc;
    }
    return isAccessible;
}

std::vector<bool> CameraManager::getCameraOpenedInfo()
{
    std::vector<bool> isOpened(this->cameraList.size());
    for (const auto& camera : this->cameraList)
    {
        //        size_t index = camera.first.find("_");
        //        int number = camera.first[index+1]-'0';
        //        if(index!=prefix_.size()-1){
        //            number = (prefix_[index]-'0')*10+(prefix_[index+1]-'0');
        //        }

        std::string prefix_ = camera.first;
        size_t index = prefix_.find("_") + 1;
        int number = prefix_[index] - '0';
        if (index != prefix_.size() - 1)
        {
            number = (prefix_[index] - '0') * 10 + (prefix_[index + 1] - '0');
        }


        isOpened[number - 1] = camera.second->getDeviceConnectStatus();
    }
    return isOpened;
}

bool CameraManager::camerasIsOpened()
{
    std::vector<bool> res = this->getCameraOpenedInfo();
    for (int i = 0; i < res.size(); i++)
    {
        if (!res[i])
        {
            return false;
        }
    }
    return true;
}

void CameraManager::getImageAllByResizeAndCorrect(std::string camType)
{

    //
    this->dataMutex.lock();
    if (this->cameraImagesResize.size() > 0)
    {
        this->cameraImagesResize.clear();
    }

    for (auto& camera : this->cameraList)
    {
        cv::Mat image = this->cameraList[camera.first]->getFrame();

        if (camType == "Line")
        { //
            if (camera.first.find("HoleCam") != std::string::npos)
            {
                continue;
            }
        }

        if (camType == "Hole")
        {//
            if (camera.first.find("LineCam") != std::string::npos)
            {
                continue;
            }
        }

        if (!image.empty())
        {
            std::string prefix = camera.first;
            size_t index = prefix.find("_") + 1;
            int number = prefix[index] - '0';
            if (index != prefix.size() - 1)
            {
                number = (prefix[index] - '0') * 10 + (prefix[index + 1] - '0');
            }
            cv::Mat imgResize;
            cv::resize(image, imgResize, cv::Size(this->imgUiW, this->imgUiH));
            if (number == 5)
            {
                image_correction(imgResize, 3);
            }
            if (number == 6)
            {
                image_correction(imgResize, 1);
            }
            if (number == 2 || number == 4)
            {
                image_correction(imgResize, 2);
            }
            this->cameraImagesResize[camera.first] = imgResize;
        }
        else
        {
            image = cv::Mat();
            this->cameraImagesResize[camera.first] = image;
        }
    }
    this->dataMutex.unlock();
}

std::vector<bool> CameraManager::getCameraConnectStatus()
{
    std::vector<bool> isConnected;
    conMutex.lock();
    isConnected = this->cameraConnectStatus;
    conMutex.unlock();
    return isConnected;
}





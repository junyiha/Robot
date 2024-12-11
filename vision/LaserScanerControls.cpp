//
// Created by csh_i on 2024/4/28.
//



#include "LaserScanerControls.h"


LaserScanerControls::LaserScanerControls()
{

    //创建雷达对象
    this->ip_config = {
            {"lida_1" ,  "192.168.1.91"},
            {"lida_2" ,  "192.168.1.93"},
            {"lida_3" ,  "192.168.1.96"},
            {"lida_4" ,  "192.168.1.90"},
            //            {"lida_5" ,  "192.168.1.105"}
    };


    this->direct_config = {
            {"lida_1" ,  true},
            {"lida_2" ,  true},
            {"lida_3" ,  false},
            {"lida_4" ,  true},
            //            {"lida_5" ,  false}
    };

    this->logger = spdlog::get("logger");
    this->start(); // 启动线程，开启轮廓激光

}

LaserScanerControls::~LaserScanerControls()
{
    for (auto& item : this->scaners)
    {
        delete item.second;
    }
}
void LaserScanerControls::getDataAll()
{

    if (this->pointCloudMasks.size() > 0)
    {
        this->pointCloudMasks.clear();
    }
    //获取雷达数据Mat
    for (auto& item : this->scaners)
    {
        // 软触发方式
        item.second->getPointsMaskOnce();
        this->pointCloudMasks[item.first] = item.second->getResultMask();
    }
}

void LaserScanerControls::closeAllLaserScaner()
{

    for (auto& item : this->scaners)
    {
        item.second->scanerDisConnect();
    }
}

void LaserScanerControls::run()
{
    unsigned index = 0;
    for (auto& item : this->ip_config)
    {
        MyBestfitLaserScaner* layserScaner = new MyBestfitLaserScaner(item.second, item.first);
        layserScaner->isleft = CfgLidar[index][0];
        layserScaner->revAngle = CfgLidar[index][1];
        this->scaners[item.first] = layserScaner;
        scaners[item.first] = layserScaner;
        index += 1;
        layserScaner->startLaserScanTask();
    }
    std::vector<bool> states;
    // 实时获取轮廓激光设备连接状态
    while (true)
    {

        states = getLayserScannerConnectStates();
        conMutex.lock();
        if (con_states.size() > 0)
        {
            con_states.clear();
        }
        con_states = states;
        conMutex.unlock();
        QThread::msleep(200);
    }
}

std::vector<bool> LaserScanerControls::getLayserScannerConnectStates()
{

    std::vector<bool> con_states;
    for (auto& item : this->scaners)
    {
        con_states.push_back(item.second->getConnectState());
    }
    return con_states;
}

void LaserScanerControls::setLaserOn(std::vector<std::string> laser_name)
{
    for (auto& item : laser_name)
    {
        this->scaners[item]->laserOn();
    }
}

void LaserScanerControls::setLaserOff(std::vector<std::string> laser_name)
{
    for (auto& item : laser_name)
    {
        this->scaners[item]->laserOff();
    }
}

void LaserScanerControls::reconnectLaser(std::string laser_name)
{
    this->scaners[laser_name]->reconnectScanner();
}

std::vector<bool> LaserScanerControls::getLaserConnectStates()
{
    std::vector<bool> states;
    conMutex.lock();
    states = con_states;
    conMutex.unlock();
    return states;
}


//
// Created by csh_i on 2024/4/28.
//



#include "LaserScanerControls.h"


LaserScanerControls::LaserScanerControls(){

    //创建雷达对象
    this->ip_config = {
            {"lida_1" ,  "192.168.1.90"},
            {"lida_2" ,  "192.168.1.91"},
            {"lida_3" ,  "192.168.1.93"},
            {"lida_4" ,  "192.168.1.96"},
//            {"lida_5" ,  "192.168.1.105"}
    };


    this->direct_config = {
            {"lida_1" ,  true},
            {"lida_2" ,  true},
            {"lida_3" ,  false},
            {"lida_4" ,  true},
//            {"lida_5" ,  false}
    };


    this->start(); // 启动线程，开启轮廓激光

 }

LaserScanerControls::~LaserScanerControls(){
    for(auto &item : this->scaners){
        delete item.second;
    }
}
void LaserScanerControls::getDataAll(){

    if(this->pointCloudMasks.size()>0){
        this->pointCloudMasks.clear();
    }

    //获取雷达数据Mat
    for(auto &item : this->scaners){
        this->pointCloudMasks[item.first]=item.second->getResultMask();
    }
}

void LaserScanerControls::closeAllLaserScaner() {

    for(auto &item : this->scaners){
        item.second->scanerDisConnect();
    }
}

void LaserScanerControls::run() {

    bool CfgLidar[4][2] = {
            {false  ,   false},
            {true   ,   false},
            {true   ,   false},
            {false  ,   false},
//            {true   ,   false},
    };

    unsigned index = 0;
    for(auto &item : this->ip_config){
        MyBestfitLaserScaner* layserScaner = new MyBestfitLaserScaner(item.second,item.first);
        layserScaner->isleft = CfgLidar[index][0];
        layserScaner->revAngle = CfgLidar[index][1];
        this->scaners[item.first] = layserScaner;
        scaners[item.first] = layserScaner;
        index+=1;
        layserScaner->startLaserScanTask();
    }

}

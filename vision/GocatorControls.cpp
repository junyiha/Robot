//
// Created by csh_i on 2024/4/28.
//
#include "GocatorControls.h"


GocatorControls::GocatorControls(){

    //创建雷达对象
    this->ip_config = {
            {"lida_1" ,  "192.168.1.101"},
            {"lida_2" ,  "192.168.1.102"},
            {"lida_3" ,  "192.168.1.103"},
            {"lida_4" ,  "192.168.1.104"},
            {"lida_5" ,  "192.168.1.105"}
    };

    bool CfgLidar[5][2] = {
            {false  ,   false},
            {true   ,   false},
            {true   ,   false},
            {false  ,   false},
            {true   ,   false},
    };

    unsigned index = 0;
    for(auto &item : this->ip_config){
        Gocator* goc = new Gocator();
        goc->ip = item.second;
        goc->goc_name = item.first;
        goc->isleft = CfgLidar[index][0];
        goc->revAngle = CfgLidar[index][1];
        this->gocators[item.first] = goc;
        index+=1;
   }
 }

GocatorControls::~GocatorControls(){
    for(auto &item : this->gocators){
        delete item.second;
    }
}
void GocatorControls::get_data(){

    /**
 * @brief 获取数据
 *
 * 从 Gocator 设备中获取雷达数据，并将其存储在 lidars 成员变量中。
 * 如果 lidars 成员变量已经包含数据，则先清空其内容。
 */

    if(this->lidars.size()>0){
        this->lidars.clear();
    }

    //获取雷达数据Mat
    for(auto &item : this->gocators){
        item.second->getData();

        this->lidars[item.first]=item.second->lidar_data;
    }
}

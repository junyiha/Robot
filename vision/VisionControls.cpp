//
// Created by csh_i on 2024/4/28.
//
#include "VisionControls.h"




void VisionControls::parser_result(stMeasureData* stm){

    // 解析雷达设备数据
    if(this->detect_type ==LIDAR){
        if(this->lidar_res.size()>0){
           for(auto& lidar: this->lidar_res){
               std::string prefix =  lidar.first;
               size_t index = prefix.find("_");

               int number = prefix[index+1]-'0';
               if(index+1!=prefix.size()-1){
                   number = (prefix[index+1]-'0')*10+(prefix[index+2]-'0');
               }
               // 特殊处理，忽视雷达2结果数据，后续有待于进一步改进
               if(number==2){
                   lidar.second.gap = -1;
                   lidar.second.dist = -1;
                   lidar.second.flag= false;
               }
               stm->m_LaserGapDistance[number-1] = lidar.second.gap;
               stm->m_LaserGapHeight[number-1] = lidar.second.dist;
               stm->m_bLaserProfile[number-1] = lidar.second.flag;
           }
        }
    }


    //解析边缘相机结果
    if(this->detect_type ==LINE){
        if(this->line_res.size()>0){
           for(auto& line: this->line_res){
               std::string prefix =  line.first;
               size_t index = prefix.find("_");

               int number = prefix[index+1]-'0';
               if(index+1!=prefix.size()-1){
                   number = (prefix[index+1]-'0')*10+(prefix[index+2]-'0');
               }
               stm->m_LineDistance[number-1] = line.second.dist*this->scales_line[line.first];
               stm->m_bLineDistance[number-1] = line.second.status;
           }
        }
    }
     //解析圆孔相机结果
    if(this->detect_type ==HOLE){
        if(this->hole_res.size()>0){
           for(auto& hole: this->hole_res){
               std::string prefix =  hole.first;
               size_t index = prefix.find("_");

               int number = prefix[index+1]-'0';
               if(index+1!=prefix.size()-1){
                   number = (prefix[index+1]-'0')*10+(prefix[index+2]-'0');
               }
               stm->m_HoleDev[number-1] = Eigen::Vector2f(hole.second.offset_x*this->scales_hole[hole.first],hole.second.offset_y*this->scales_hole[hole.first]);
               stm->m_bHoleDev[number-1] = hole.second.status;
           }
        }
    }
}


VisionControls::VisionControls(){
    // 初始化硬件设备
    this->camera_controls_hole = new CameraControls("hole"); // 初始化圆孔摄像头
    this->camera_controls_edge = new CameraControls("edge"); //初始化边缘摄像头
    this->gocator_controls = new GocatorControls(); //初始化雷达设备

    //初始化比例系数
    this->scales_hole= {
        {"hole_1", 0.054},
        {"hole_2", 0.054},
        {"hole_3", 0.054},
        {"hole_4", 0.054},
    };

    this->scales_line= {
        {"edge_2", 0.058921},
        {"edge_4", 0.058921},
        {"edge_6", 0.058921},
        {"edge_7", 0.060000},
        {"edge_8", 0.058988},
        {"edge_9", 0.056059},
        {"edge_10",0.056457},
    };

    qDebug()<<"初始化硬件设备";

    // 生产者-消费者 共享互斥锁变量
    this->sharedDataHole = new SharedData();
    this->sharedDataLine = new SharedData();
    this->sharedDataGocator = new SharedData();
    QThread::msleep(3000);

    //初始化检测器
    qDebug()<<"初始化检测器";
    this->hole_helper = new HoleHelper();
    this->line_helper = new LineHelper();
    this->lidar_helper = new LidarHelper();

    //初始化处理线程
    qDebug()<<"初始化处理线程";
    this->hole_handler = new HoleHandler(this->hole_helper, this->camera_controls_hole,this->sharedDataHole);
    this->line_handler = new LineHandler(this->line_helper, this->camera_controls_edge, this->sharedDataLine);
    this->lidar_handler = new LidarHandler(this->lidar_helper, this->gocator_controls,this->sharedDataGocator);

   //空线程开启
    qDebug()<<"空线程开启";
//    this->hole_handler->start();
//    this->line_handler->start();
//   this->lidar_handler->start();
}

VisionControls::~VisionControls(){

    //指针变量销毁
     delete this->camera_controls_hole;
     delete this->camera_controls_edge;
     delete this->gocator_controls;

     delete this->hole_helper;
     delete this->line_helper;
     delete this->lidar_helper;

     delete this->hole_handler;
     delete this->line_handler;
     delete this->lidar_handler;

     delete this->sharedDataHole;
     delete this->sharedDataLine;
     delete this->sharedDataGocator;
}


void VisionControls::enableDetection(DetectType type){

    this->detect_type = type;
    if(type==LINE){
        this->line_handler->is_start_detect(true);
        this->hole_handler->is_start_detect(false);
        this->lidar_handler->is_start_detect(false);

    }

    if(type==HOLE){
       this->hole_handler->is_start_detect(true);
       this->line_handler->is_start_detect(false);
       this->lidar_handler->is_start_detect(false);
    }

    if(type==LIDAR){
       this->lidar_handler->is_start_detect(true);
       this->hole_handler->is_start_detect(false);
       this->line_handler->is_start_detect(false);
    }

}

void VisionControls::getDetectResult(DetectType type, stMeasureData* stm){
    //设置状态标识符
    this->enableDetection(type);
    // 获取边线检测数据
    if(this->detect_type==LINE){
        QMutexLocker locker_line(&this->sharedDataLine->mutex);
        this->line_handler->results.clear();
        this->sharedDataLine->spaceAvailable.wakeAll();
        if(!this->line_handler->results.size()){
            this->sharedDataLine->dataReady.wait(&this->sharedDataLine->mutex);
        }
        this->line_res = this->line_handler->results;


    }
    else if(this->detect_type==HOLE){ // 获取圆孔检测数据
        QMutexLocker locker(&this->sharedDataHole->mutex);
        this->hole_handler->results.clear();
        this->sharedDataHole->spaceAvailable.wakeOne();
        if(!this->hole_handler->results.size()){
            this->sharedDataHole->dataReady.wait(&this->sharedDataHole->mutex);
        }
        this->hole_res = this->hole_handler->results;

    }
    else if(this->detect_type==LIDAR){ // 获取雷达检测数据
        QMutexLocker locker(&this->sharedDataGocator->mutex);
        this->lidar_handler->results.clear();
        this->sharedDataGocator->spaceAvailable.wakeOne();
        if(!this->lidar_handler->results.size()){
            this->sharedDataGocator->dataReady.wait(&this->sharedDataGocator->mutex);
        }
        this->lidar_res = this->lidar_handler->results;

    }else{
        return ;
    }
    // Parser results
    this->parser_result(stm);
}








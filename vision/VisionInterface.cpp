//
// Created by chen on 2024/7/6.
//

#include "VisionInterface.h"

VisionInterface::VisionInterface() {


    this->camera_controls = new CameraManager();  // 相机管理类
    this->line_helper = new LineDetector();       // 边线检测功能类
    this->sharedDataLine = new SharedData();      // 共享数据类

    // 相机读取+边线检测 端到端处理类(线程)
    this->line_handler = new LineDetectorRunner(this->line_helper,this->camera_controls,this->sharedDataLine);

    // 启动直线检测线程
    this->line_handler->is_running = true;
    this->line_handler->start();

    this->scales_line= {
            {"cam_1", 0.153846153846153},
            {"cam_2", 0.153061224489795},
            {"cam_3", 0.153061224489795},
            {"cam_4", 0.153061224489795},
            {"cam_5", 0.153061224489795},
            {"cam_6", 0.153061224489795},
    };

    this->logger = spdlog::get("logger");
}

VisionInterface::~VisionInterface() {

    if(this->line_handler != nullptr)
        delete this->line_handler;
    if(this->camera_controls != nullptr)
        delete this->camera_controls;
    if(this->sharedDataLine != nullptr)
        delete this->sharedDataLine;
    this->isRunning = false;
    this->quit();
}

void VisionInterface::getDetectResult(DetectType type, stMeasureData *stm) {

    QMutexLocker locker_line(&this->sharedDataLine->mutex);
    if(this->line_handler->results.size()>0){
        this->line_handler->results.clear();
    }
    this->sharedDataLine->spaceAvailable.wakeOne();
    if(!this->line_handler->results.size()){
        this->sharedDataLine->dataReady.wait(&this->sharedDataLine->mutex);
    }

    this->line_res = this->line_handler->results;
    //解析相关数据
    this->parser_result(stm);
}

void VisionInterface::parser_result(stMeasureData *stm) {

    if(this->line_res.size()>0){
        for(auto& line: this->line_res){
            std::string prefix =  line.first;
            size_t index = prefix.find("_");
            int number = prefix[index+1]-'0';
            if(index+1!=prefix.size()-1){
                number = (prefix[index+1]-'0')*10+(prefix[index+2]-'0');
            }
            double dist = line.second.dist*this->scales_line[line.first];
            if(dist<1){
                stm->m_LineDistance[number-1] = 0;
                stm->m_bLineDistance[number-1] = false;
            }else{
                stm->m_LineDistance[number-1] = dist;
                stm->m_bLineDistance[number-1] = line.second.status;
            }
        }
    }

}

VisionResult VisionInterface::getVisResult(){
    VisionResult vis_result;
    memset(&vis_result,0,sizeof(vis_result));
    this->mutex.lock();
    vis_result = this->vis_result_;
    this->mutex.unlock();
    return vis_result;
}

void VisionInterface::run() {

    while(this->isRunning){
        //if(this->is_Detected){
        //    // 获取检测结果
        //    memset(&this->vis_result,0,sizeof(this->vis_result));
        //    getDetectResult(DetectType::LINE, &this->vis_result.stData);
        //    this->vis_result.status = true;
        //    // 结果拷贝
        //    this->mutex.lock();
        //    memset(&this->vis_result_,0,sizeof(this->vis_result_));
        //    this->vis_result_ = this->vis_result;
        //    this->mutex.unlock();
        //}
        QThread::msleep(200);
    }
}


std::map<std::string, LineDetectRes> VisionInterface::getLineRes() {
    std::map<std::string, LineDetectRes> line_res;
    this->mutex.lock();
    line_res = this->line_res;
    this->mutex.unlock();
    return line_res;
}


void VisionInterface::closeThread(){
    // 关闭相机线程
    this->camera_controls->closeAllCameraThread();

    // 关闭LineDetectorRunner线程
    this->line_handler->closedThread();

    // 关闭自身线程
    this->isRunning = false;
    QThread::quit();


}
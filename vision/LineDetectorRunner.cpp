//
// Created by chen on 2024/7/6.
//

#include "LineDetectorRunner.h"

LineDetectorRunner::LineDetectorRunner(LineDetector *line_helper, CameraManager *cam_controls, SharedData *shared) {

    this->logger = spdlog::get("logger");
    this->line_helper = line_helper;
    this->cam_controls = cam_controls;
    this->sharedData = shared;

}

LineDetectorRunner::~LineDetectorRunner() {
    this->is_running = false;
    QThread::wait() ; // 等待线程退出
}

void LineDetectorRunner::run() {

    while(this->is_running){
        if(this->detect_control_flag){
            // 先获取待检测图像
            std::map<std::string, LineDetectRes> temp_results;
            this->cam_controls->getImageAll();
            std::map<std::string, cv::Mat> cameraData = this->cam_controls->getCameraImages();
            if(cameraData.size() > 0){
                for(auto &camera_data : cameraData)
                {
                    LineDetectRes vision_res;
                    std::string camera_name = camera_data.first;
                    cv::Mat image = camera_data.second;
                    if(image.empty()){
                        logger->info("Gets the image of camera {} is empty !",camera_name);
                        continue;
                    }
                    // 检测图像
                    auto start_time = std::chrono::high_resolution_clock::now();
                    LineResult line_res = this->line_helper->getLineDistance(image);
                    auto end_time = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
//                    std::cout << "line detection total time:: " << duration.count() << " milliseconds." << std::endl;
                    if(line_res.status){ //检测成功
                        vision_res.status = true;
                        vision_res.dist = line_res.lineDist;
                        vision_res.img_drawed = line_res.imgDrawed;
//                        this->logger->info("Camera {} get line distance is {}",camera_name,line_res.lineDist);
                    }
                    else{
                        vision_res.status = false;
                        vision_res.dist = -1;
//                        this->logger->info(vision_res.)
                        vision_res.img_drawed =line_res.imgDrawed;
                    }
                    temp_results[camera_name] = vision_res;
                }
                QMutexLocker locker(&this->sharedData->mutex);
                this->results = temp_results;
                if(this->results.size()>0){ // 生产一个数据， 唤醒消费者进程，通知消费
                    this->sharedData->dataReady.wakeOne();
                    this->sharedData->spaceAvailable.wait(&this->sharedData->mutex);
                }
            }else{
                logger->info("Gets the number of image frames 0!");
            }
        }
        QThread::msleep(200);
    }
}

void LineDetectorRunner::closedThread() {
    this->detect_control_flag = false;
    this->is_running = false;
    QThread::wait();
}

void LineDetectorRunner::pauseDetect() {
    this->detect_control_flag = false;
}

void LineDetectorRunner::restartDetect() {
    this->detect_control_flag = true;
}








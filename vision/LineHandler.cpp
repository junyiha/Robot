#include"LineHandler.h"
#include<chrono>



LineHandler::LineHandler(LineHelper* hole,CameraControls* cam,SharedData* shared){
    this->cam_controls = cam;
    this->line_helper = hole;
    this->sharedData = shared;
}

LineHandler::~LineHandler(){
    this->detect_control_flag = false;
    this->thread_control_flag = false;
}

void LineHandler::run(){

    while(this->thread_control_flag){
        QThread::sleep(100);


        if(this->detect_control_flag){
            // 获取数据
            this->cam_controls->get_camera_data();

            std::map<std::string, CameraRes> cam_res =  this->cam_controls->cameras_data;
            std::map<std::string, LineVisionRes> temp_results;

            for(const auto& camera_res_item: cam_res){
               LineVisionRes vision_res;
               if(camera_res_item.second.status){
                   LineRes line_res = this->line_helper->compute_lines_distance(camera_res_item.second.img_data);
                   if(line_res.status){ //检测成功
                       vision_res.status = true;
                       vision_res.dist = line_res.dist;
                       vision_res.img_drawed = line_res.img_drawed;
                   }
                   else{
                       vision_res.status = false;
                       vision_res.dist = -1;
                       vision_res.img_drawed =line_res.img_drawed;
                   }

               }else{
                   vision_res.status = false;
                   vision_res.dist = -1;
               }
               temp_results[camera_res_item.first] = vision_res;
            }

            QMutexLocker locker(&this->sharedData->mutex);
            if(this->results.size()>0){
                this->sharedData->spaceAvailable.wait(&this->sharedData->mutex);
            }
            this->results = temp_results;
            this->sharedData->dataReady.wakeAll();
        }



    }

}


void LineHandler::is_start_detect(bool flag){
    this->detect_control_flag = flag;
}

void LineHandler::closed(){
    this->thread_control_flag = false;
}







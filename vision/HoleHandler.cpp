#include"HoleHandler.h"

HoleHandler::HoleHandler(HoleHelper* hole_helper,CameraControls* cam_controls, SharedData* sharedData){
    this->cam_controls = cam_controls;
    this->hole_helper = hole_helper;
    this->sharedData = sharedData;
}

HoleHandler::~HoleHandler(){
    this->detect_control_flag = false;
    this->thread_control_flag = false;
}


void HoleHandler::run(){

    while(this->thread_control_flag){
        QThread::sleep(100);

        if(this->detect_control_flag){
            // 获取数据

            this->cam_controls->get_camera_data();
            std::map<std::string, CameraRes> cam_res =  this->cam_controls->cameras_data;
            std::map<std::string, HoleVisionRes> temp_results;

            for(auto& camera_res_item : cam_res){
               HoleVisionRes vision_res;

               if(camera_res_item.second.status){

                   HoleRes hole_res = this->hole_helper->get_holes_distance(camera_res_item.second.img_data);

                   if(hole_res.status){ //检测成功
                       vision_res.status = true;
                       vision_res.offset_x = hole_res.offset_x;
                       vision_res.offset_y = hole_res.offset_y;
                       vision_res.img_drawed = hole_res.img_drawed;
                   }
                   else{
                       vision_res.status = false;
                       vision_res.offset_x = -1;
                       vision_res.offset_y = -1;
                       vision_res.img_drawed =hole_res.img_drawed;
                   }

               }else{
                   vision_res.status = false;
                   vision_res.offset_x = -1;
                   vision_res.offset_y = -1;
               }
               temp_results[camera_res_item.first] = vision_res;
            }

            // Producter
            QMutexLocker locker(&this->sharedData->mutex);
            if(this->results.size()>0){
                this->sharedData->spaceAvailable.wait(&this->sharedData->mutex);
            }
            this->results = temp_results;
            this->sharedData->dataReady.wakeOne();
        }

    }

}

void HoleHandler::is_start_detect(bool flag){
    this->detect_control_flag = flag;
}

void HoleHandler::closed(){
    this->thread_control_flag = false;
}





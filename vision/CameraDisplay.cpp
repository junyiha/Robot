#include "CameraDisplay.h"

CameraDisplay::CameraDisplay(VisionControls* visionInput,QLabel* labelInput[],size_t numLabels)
{
    m_vision = visionInput;
    labels = new QLabel*[numLabels];
    for(size_t i=0; i<numLabels ;++i){
        labels[i] = labelInput[i];
    }

    this->cfg_hole = {
        "hole_1","hole_2","hole_3","hole_4"
    };
}


void CameraDisplay::run()
{
    while(openThreadFlag == true){

        if(this->suspendFlag  == true){
            this->m_vision->camera_controls_hole->get_camera_data();
            auto cameras_data = m_vision->camera_controls_hole->cameras_data;

            for(int i=0;i<SUM_ShwHole;++i){
                if(cameras_data[cfg_hole[i]].status == true && cameras_data[cfg_hole[i]].img_data.type() == CV_8UC3){
                    cv::Mat img_dis;
                    cv::resize(cameras_data[cfg_hole[i]].img_data,img_dis,cv::Size(168,300));
                    QImage tmp_img(img_dis.data , img_dis.cols , img_dis.rows , QImage::Format_RGB888);
                    this->img[i] = tmp_img;
                    this->labels[i]->setPixmap(QPixmap::fromImage(img[i]));
                }else{
                    qDebug()<<"【ERROR】 void CameraDisplay::run()";
                }
            }
        }
//        QApplication::processEvents();
        QThread::msleep(200);
    }

}

void CameraDisplay::setSuspendFlag(bool flag)
{
    this->suspendFlag = flag;
}

void CameraDisplay::setOpenThreadFlag(bool flag)
{
    this->openThreadFlag = flag;
}

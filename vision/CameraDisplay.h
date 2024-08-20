#ifndef CAMERADISPLAY_H
#define CAMERADISPLAY_H

#include <opencv2/opencv.hpp>
#include <QLabel>
#include <QDebug>
#include <QImage>
#include <QObject>
#include "../vision/VisionControls.h"
#include <vector>
#include "../GVL.h"

class CameraDisplay :public QThread
{
    //Q_OBJECT
public:
    CameraDisplay(VisionControls* visionInput, QLabel* labelsInput[],size_t numLabels);
    ~CameraDisplay(){
        for(int i=0;i<4;++i){
            delete labels[i];
        }
        delete labels;
    }

    void run() override;
    void setSuspendFlag(bool flag);
    void setOpenThreadFlag(bool flag);
    QImage getImg(const int index);
private:
    VisionControls* m_vision = NULL;
    QImage  img[4];
    QLabel** labels;
    bool openThreadFlag = true;
    bool suspendFlag = true;
    std::vector<std::string> cfg_hole;

};

#endif // CAMERADISPLAY_H

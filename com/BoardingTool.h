//
// Created by fjbfei on 2024/9/20.
//
/*****************************************************************
* 类名称： 装板工装功能实现
* 功能描述： ·功能定义，将IO输出封装为功能接口
* 参数说明： 参数说明
* 返回值：   返回值说明
* 硬件功能--------------------------------------------------------
* 硬件清单：相机灯3组，推缸2，激光标线1组
*   长边相机灯    短边相机灯 孔位相机灯  电推杆前进 电推杆后退    十字激光
*    DO31        DO32       DO33     DO35     DO36        DO37

******************************************************************/

#ifndef CLINTEST_BOARDINGTOOL_H
#define CLINTEST_BOARDINGTOOL_H

#include "IOCom.h"
#include <QObject>
#include <Eigen/Dense>

class BoardingTool {
public:
    BoardingTool();
    ~BoardingTool();
    IOCom		m_cIOA;

//****************功能接口***************
    /**
    * @brief  灯光控制
    * @param  index 灯光编号0，全部 1.长边，2.短边，3.孔位
    * @param  On    开关量
    */
    void SetLight(quint8 index, bool On);

    /**
    * @brief  激光标线控制
    * @param  On    开关量
    */
    void SetLaserMarker(bool On);

    /**
    * @brief  推杆控制
    * @param  push  -1 推荐倒推，0 停止， 1推荐正推
    **/
    void SetCylinder(int push);

    //关闭通讯
    void close(){m_cIOA.close();}

protected:
    std::shared_ptr<spdlog::logger> log;


};


#endif //CLINTEST_BOARDINGTOOL_H

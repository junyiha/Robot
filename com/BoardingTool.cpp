//
// Created by fjbfe on 2024/9/20.
//

#include "BoardingTool.h"

BoardingTool::BoardingTool()
{
    log = spdlog::get("logger");
}


void BoardingTool::SetLight(quint8 index, bool On)
{
    byte portvalue = m_cIOA.getDOState()[1];
    if(On == true) {
        switch (index) {
            case 0:
                portvalue |= 0b0000111;
                break;
            case 1:
                portvalue |= 0b0000001;
                break;
            case 2:
                portvalue |= 0b0000010;
                break;
            case 3:
                portvalue |= 0b0000100;
                break;
            default:
                log->warn("Invalid light index: {}", index);
        }
    }
    else
    {
        switch (index)
            {
                case 0:
                    portvalue &= ~0b0000111;
                    break;
                case 1:
                    portvalue &= ~0b0000001;
                    break;
                case 2:
                    portvalue &= ~0b0000010;
                    break;
                case 3:
                    portvalue &= ~0b0000100;
                    break;
                default:
                    log->warn("Invalid light index: {}", index);
            }

    }

    m_cIOA.SetIO(1,portvalue);
}


void BoardingTool::SetLaserMarker(bool On)
{
    byte portvalue = m_cIOA.getDOState()[1];
    if(On == true)
    {
        portvalue |= 0b01000000;
    }
    else
    {
        portvalue &= ~0b01000000;
    }
    m_cIOA.SetIO(1,portvalue);
}

void BoardingTool::SetCylinder(int push)
{
    byte portvalue = m_cIOA.getDOState()[1];
    switch (push)
    {
        case -1:
            portvalue |=  0b00100000;
            portvalue &= ~0b00010000;
            break;
        case 0:
            portvalue &= ~0b00110000;
            break;
        case 1:
            portvalue |=  0b00010000;
            portvalue &= ~0b00100000;
            break;
        default:
            log->warn("Invalid cylinder action :push = {}", push);
            break;

    }
    m_cIOA.SetIO(1,portvalue);


}
QVector<double> BoardingTool::getLaserDistance()
{
    QVector<double> re(6);
    QVector<double> tmp(6);//转换一下

    tmp[0] = m_cIOA.getAIState()[3];
    tmp[1] = m_cIOA.getAIState()[5];
    tmp[2] = m_cIOA.getAIState()[6];
    tmp[3] = m_cIOA.getAIState()[7];

    log->debug("*******************LaserDistance*************: {} {} {} {} ",tmp[0],tmp[1],tmp[2],tmp[3]);

    //视觉标定板上表面为激光零点
    std::vector<Eigen::Vector2f> cfg_laser ={
            //      k       b
            // Eigen::Vector2f(1950,1545),//0mm对应的数值，95mm对应的数值
            // Eigen::Vector2f(1957,1548),//2
            // Eigen::Vector2f(1960,1545),//3
            // Eigen::Vector2f(1950,1540),//4

            Eigen::Vector2f( 1966,1545),//0mm对应的数值，80mm对应的数值
            Eigen::Vector2f(1965,1548),//2    // 15mm( 1965, 1968,)
            Eigen::Vector2f(1970,1545),//3
            Eigen::Vector2f(1965,1540),//4
    };
    for(int i=0;i<4;++i){
        //换算为真实距离，单位毫米
        tmp[i] = 80.0/(cfg_laser[i].y()-  cfg_laser[i].x()) * (tmp[i] -  cfg_laser[i].x());
    }

    re[0] = tmp[2];
    re[1] = tmp[3];
    re[2] = tmp[1];
    re[3] = tmp[0];

    //log->debug("测量距离：{} {} {} {} ",re[0],re[1],re[2],re[3]);
    return re;
}



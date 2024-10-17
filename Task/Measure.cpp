#include "Measure.h"
#include "qdebug.h"
#include "../robot/robot.h"

//2024年5月14日 17点11分
/* 碰钉偏差计算
 * 调平：TarPose0 使用四个点激光
 * 对齐：VirtualPose3 使用 6个边线数据+4个点激光数据
 */
QVector<Eigen::Matrix4d> CMeasure::calPoseDeviation(stMeasureData data, double tar_distance)
{
    std::vector<int> stepValid = {-2,-2,-2,-2,-2,-2};

    QVector<Eigen::Matrix4d> Result(7);
    std::vector<Eigen::Vector3d>  laserDistance(4);
    double Profilelaser = 1; //轮廓激光数据完备性
    std::vector<Eigen::Vector3d>    ProfilerlaserDistance(4); //轮廓激光数据;

    /* TarPose0 轮廓激光虚拟平面计算 ************************************************************/
    //输入：4个轮廓激光数据+测距激光(近似使用)，无轮廓激光时仅调平

    qDebug()<<"m_bLaserDistance:  "<<data.m_bLaserDistance[0]<<"   "<<data.m_bLaserDistance[1]<<"   "<<data.m_bLaserDistance[2]<<"   "<<data.m_bLaserDistance[3]<<"   "<<"\n\n";
    qDebug()<<"m_LaserDistance:  "<<data.m_LaserDistance[0]<<"   "<<data.m_LaserDistance[1]<<"   "<<data.m_LaserDistance[2]<<"   "<<data.m_LaserDistance[3]<<"   "<<"\n\n";

    if(data.m_bLaserDistance[0]&&data.m_bLaserDistance[1]&&data.m_bLaserDistance[2]&&data.m_bLaserDistance[3])//4个点激光均有效
    {
            laserDistance[0] << x_Laser,-y_Laser  , data.m_LaserDistance[0]-Lift_Distance_In_Parallel;
            laserDistance[1] << x_Laser,y_Laser   , data.m_LaserDistance[1]-Lift_Distance_In_Parallel;
            laserDistance[2] << -x_Laser,y_Laser  , data.m_LaserDistance[2]-Lift_Distance_In_Parallel;
            laserDistance[3] << -x_Laser,-y_Laser , data.m_LaserDistance[3]-Lift_Distance_In_Parallel;

            Result[0] = calPoseforSquare(laserDistance);
            stepValid[0] = true;
            Profilelaser = -1;
    }else{
        Result[0] << 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1;
        stepValid[0] = false;
    }

    /* VirtualPose1 对孔虚拟平面计算 ************************************************************/
    //输入：4个螺柱中心位置+激光检测距离(近似使用)
    std::vector<Eigen::Vector3d>    HolePosition(4); //轮廓激光数据;
    double holeInvid= 1;
    if(data.m_bHoleDev[0]&&data.m_bHoleDev[1]&&data.m_bHoleDev[2]&&data.m_bHoleDev[3] == false)
    {
        qDebug()<<"-----------hole error-----------";
        Result[1] = Eigen::Matrix4d::Identity();
        holeInvid = -1;
        stepValid[1] = false;
    }else
    {
        HolePosition[0]<< x_hole + data.m_HoleDev[0].x(),-y_hole + data.m_HoleDev[0].y(),data.m_LaserDistance[0]-DistanceB;
        HolePosition[1]<< x_hole + data.m_HoleDev[1].x(), y_hole + data.m_HoleDev[1].y(),data.m_LaserDistance[1]-DistanceB;
        HolePosition[2]<<-x_hole + data.m_HoleDev[2].x(), y_hole + data.m_HoleDev[2].y(),data.m_LaserDistance[2]-DistanceB;
        HolePosition[3]<<-x_hole + data.m_HoleDev[3].x(),-y_hole + data.m_HoleDev[3].y(),data.m_LaserDistance[3]-DistanceB;

        Result[1] = calPoseforSquare(HolePosition);
        stepValid[1] = true;
    }

    /* VirtualPose2 穿孔 ************************************************************************/

    if(data.m_bLaserDistance[0] &&
       data.m_bLaserDistance[1] &&
       data.m_bLaserDistance[2] &&
       data.m_bLaserDistance[3]){
        if(data.m_LaserDistance[0]>60 && data.m_LaserDistance[1]>60 && data.m_LaserDistance[2]>60 && data.m_LaserDistance[3]>60){
            double min = 10000;
            for(int i=0;i<4;++i){
                if(data.m_LaserDistance[i] <= min){
                    min = data.m_LaserDistance[i];
                }
            }

            Result[2] << 1, 0, 0, 0,
                         0, 1, 0, 0,
                         0, 0, 1, min-DistanceC,//// 错误 错误 错误 错误 错误 错误 错误 错误 错误
                         0, 0, 0, 1;
            stepValid[2] = true;
        }else{
            Result[2] << 1, 0, 0, 0,
                         0, 1, 0, 0,
                         0, 0, 1, 0,
                         0, 0, 0, 1;
            stepValid[2] = false;
        }
    }else{
        Result[2] << 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1;
        stepValid[2] = false;
    }

    /* VirtualPose3 对边线平面计算 ******************************************************************/
    //输入：6个边线距离+4个点激光数据(近似使用)
    std::vector<double> validDistance(6);
    const double staLineDist = 15;

    //data.m_LineDistance[3]  = data.m_LineDistance[3] - 1;//临时加偏置-----------------------------------
    //data.m_LineDistance[4]  = data.m_LineDistance[4] + 2;//临时加偏置-----------------------------------
    //data.m_bLineDistance[5] = false;//临时处理，后面视觉那边得补充数据有效性校验-----------------------------------

    for(int i=0;i<6;i++)
    {
        validDistance[i] = (data.m_LineDistance[i] - staLineDist)*(data.m_bLineDistance[i]?1:0);
    }
    std::vector<double> distance(4);


    Eigen::Vector4f linesDist[3] = {
        Eigen::Vector4f(validDistance[0],validDistance[1],data.m_bLineDistance[0],data.m_bLineDistance[1]),
        Eigen::Vector4f(validDistance[2],validDistance[3],data.m_bLineDistance[2],data.m_bLineDistance[3]),
        Eigen::Vector4f(validDistance[5],validDistance[4],data.m_bLineDistance[5],data.m_bLineDistance[4])
    };

    stepValid[3] = true;
    for(int i=0;i<3;++i)
    {
        if(linesDist[i][2] == false && linesDist[i][3] == false){
            stepValid[3] = false;
            break;
        }
        else
        {
            distance[i] = (linesDist[i][0] - linesDist[i][1])/(linesDist[i][2] + linesDist[i][3]);
        }
    }

    distance[2] = distance[2];
    double delatY  = 0;//(distance[0]-distance[1])/4; //短边旋转补偿

    std::vector<Eigen::Vector3d>  linePosition(4); //边线虚拟面;
    if(stepValid[3] == true){
        linePosition[0]<<  x_camera + distance[0],   delatY - y_camera + distance[2] , data.m_LaserDistance[0]-Distance_work;
        linePosition[1]<<  x_camera + distance[1],   delatY + y_camera + distance[2] , data.m_LaserDistance[1]-Distance_work;
        linePosition[2]<< -x_camera + distance[1],  -delatY + y_camera + distance[2] , data.m_LaserDistance[2]-Distance_work;
        linePosition[3]<< -x_camera + distance[0],  -delatY - y_camera + distance[2] , data.m_LaserDistance[3]-Distance_work;
        Result[3] = calPoseforSquare(linePosition);
        //Result[3](1,3) = Result[3](1,3)+27;//让工具向着y负方向偏离框中心24mm，让3号相机边线距39
        //Result[3](0,3) = Result[3](0,3) + 1;
    }else{
        Result[3] << 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1;
    }

    /* VirtualPose4 贴合壁面 ************************************************************/
    if(data.m_LaserDistance[0] > 25 && data.m_LaserDistance[1] > 25 && data.m_LaserDistance[2] > 25 && data.m_LaserDistance[3] > 25){
        double min = 10000;
        for(int i=0;i<4;++i){
            if(data.m_LaserDistance[i] <= min){
                min = data.m_LaserDistance[i];
            }
        }
        Result[4] << 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, min-DistanceD,
                     0, 0, 0, 1;
        stepValid[4] = true;
    }else{
        Result[4] << 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1;
        stepValid[4] = false;
    }

    /* VirtualPose5 对边线平面计算 ************************************************************/
    //输入：10个边线距离（平面调整）+轮廓激光（有板角点） +点激光（无板角点）

    std::vector<Eigen::Vector3d>  linePosition_4(4); //边线虚拟面;
    if(stepValid[3] == true){
        std::vector<double> targetdistance(4);
        for(int i=0;i<4;i++)
        {
            int index = 2 * i;   //0 2 4  0 1 2 3
            if(index <= 4 && data.m_bLaserProfile[index] == true)
            {
                targetdistance[i] = data.m_LaserGapDistance[index];
            }
            else
            {
                targetdistance[i] = data.m_LaserDistance[i];
            }
        }
        linePosition_4[0]<<  x_camera + distance[0], -y_camera + distance[2],targetdistance[0]- tar_distance;//方向问题
        linePosition_4[1]<<  x_camera + distance[1],  y_camera + distance[2],targetdistance[1]- tar_distance;
        linePosition_4[2]<< -x_camera + distance[1],  y_camera + distance[3],targetdistance[2]- tar_distance;
        linePosition_4[3]<< -x_camera + distance[0], -y_camera + distance[3],targetdistance[3]- tar_distance;
        Result[5] = calPoseforSquare(linePosition_4);
        stepValid[5] = true;
    }else{
        Result[5] << 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1;
        stepValid[5] = false;
    }

    /* VirtualPose6 数据有效性附录 ************************************************************/
    double minboard_distance = 1000;
    double maxboard_distance = 0;
    for(int i = 0;i<4;i++)
    {
        minboard_distance =  minboard_distance < data.m_LaserDistance[i] ? minboard_distance : data.m_LaserDistance[i];
        maxboard_distance =  maxboard_distance > data.m_LaserDistance[i] ? maxboard_distance : data.m_LaserDistance[i];
    }

    Result[6] <<minboard_distance,             Profilelaser,  maxboard_distance-minboard_distance,holeInvid,
                        stepValid[0],          stepValid[1],         stepValid[2],        stepValid[3],
                        stepValid[4],          stepValid[5],                    0,                   0,
                                   0,                     0,                    0,                   0;


    return Result;
}


Eigen::Matrix4d CMeasure::calPoseforSquare(std::vector<Eigen::Vector3d> points)
{
    Eigen::Matrix4d  result = Eigen::Matrix4d::Identity();
    if(points.size() < 4)
    {
        qDebug()<<"error: input points num is less than 4";
        return result;
    }
    //目标位置
    Eigen::Vector3d position = (points[0]+points[1]+points[2]+points[3])/4.0;

    //目标法向量: 分别选取三组点求解目标法向量，取均值
    Eigen::Vector3d edge01 = points[1]-points[0];
    Eigen::Vector3d edge12 = points[2]-points[1];
    Eigen::Vector3d edge23 = points[3]-points[2];
    Eigen::Vector3d edge30 = points[0]-points[3];


    Eigen::Vector3d  normalx =  (-edge12 + edge30).normalized();
    Eigen::Vector3d  normaly =  (edge01 - edge23).normalized();
    Eigen::Vector3d  normalz = normalx.cross(normaly).normalized();




    result << normalx[0],normaly[0],normalz[0],position[0],
            normalx[1],normaly[1],normalz[1],position[1],
            normalx[2],normaly[2],normalz[2],position[2],
                       0,         0,        0,          1;



    return result;

}

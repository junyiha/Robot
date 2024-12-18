#ifndef CMEASURE_H
#define CMEASURE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <QVector>
#include <QDebug>

#include "GVL.h"

typedef struct DataofMeasurement
{
    double m_LaserDistance[4];    // 点激光数据
    bool m_bLaserDistance[4];     // 点激光数据有效性
    double m_LineDistance[6];     // 边线距离
    bool m_bLineDistance[6];      // 边线距离数据有效性
    double m_LaserGapHeight[5];   // 板高差
    double m_LaserGapDistance[5]; // 板间距
    bool m_bLaserProfile[5];      // 板高差数据有效性
    Eigen::Vector2f m_HoleDev[4]; // 孔偏差
    bool m_bHoleDev[4];           // 孔偏差有效性
    double m_JointStatusTool;     // 工具在关节空间的当前坐标

    // 重载赋值运算符
    DataofMeasurement& operator=(const DataofMeasurement& other)
    {
        if (this != &other) // 防止自赋值
        {
            // 复制数组数据
            for (int i = 0; i < 4; ++i)
            {
                m_LaserDistance[i] = other.m_LaserDistance[i];
                m_bLaserDistance[i] = other.m_bLaserDistance[i];
            }
            for (int i = 0; i < 6; ++i)
            {
                m_LineDistance[i] = other.m_LineDistance[i];
                m_bLineDistance[i] = other.m_bLineDistance[i];
            }
            for (int i = 0; i < 5; ++i)
            {
                m_LaserGapHeight[i] = other.m_LaserGapHeight[i];
                m_LaserGapDistance[i] = other.m_LaserGapDistance[i];
                m_bLaserProfile[i] = other.m_bLaserProfile[i];
            }
            for (int i = 0; i < 4; ++i)
            {
                m_HoleDev[i] = other.m_HoleDev[i];
                m_bHoleDev[i] = other.m_bHoleDev[i];
            }
            // 复制其他成员变量
            m_JointStatusTool = other.m_JointStatusTool;
        }
        return *this; // 返回当前对象的引用，以支持链式赋值
    }
} stMeasureData;

class CMeasure
{
public:
    /**
     * @brief    根据传感器测量值计算末端位置偏差
     * @param    data 传感器测量反馈数据,参见结构体定义
     * @return   不同装板阶段时的末端姿态调整量,
     * @return   [0]:TarPose0 根据点激光（高度）+轮廓激光（临板间隙）数据计算的目标调整位置 =》调整后相机可观察到螺栓
     * @return   [1]:TarPose1 根据螺栓位置+点激光计算的目标调整位置=》调整后可穿孔
     * @return   [2]:TarPose2 第三个元素为末端丝杆顶升高度，其他为为零 =》穿孔动作
     * @return   [3]:TarPose3 根据边线+点激光计算目标调整位置 =》调整后，板与边线对齐
     * @return   [4]:TarPose4 第三个元素为末端丝杆顶升高度，其他为为零 =》贴板动作
     * @return   [5]:TarPose5 根据边线+临板高差+垫板高差计算目标位置
     * @return   [6]:[0]板壁最小距离 [1]轮廓激光完备性 [2]点激光最大高差[3]孔位数据有效性，[4~5]NUll
     */
    static QVector<Eigen::Matrix4d> calPoseDeviation(stMeasureData data, double tar_distance = 0.0);

protected:
    /**
     * @brief    根据虚拟矩形角点，计算矩形中心位置和法向量姿态(欧拉角)
     * @param    传感器测量反馈数据
     * @return   矩形末端法向量相对姿态:欧拉角顺序0,1,2
     */
    static Eigen::Matrix4d calPoseforSquare(std::vector<Eigen::Vector3d>);
};

#endif // CMEASURE_H

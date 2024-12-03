#ifndef CTRANSFORMATION_H
#define CTRANSFORMATION_H
#include <Eigen/Dense>
#include <Eigen/Geometry>

enum Joint_Type
{
    Revolute = 0,
    Prismatic = 1
};

class CTransformation
{
public:
    /**
     * @brief 关节DH参数
     * @param  alpha a theta  d DH参数
     * @param  type 关节类型：滑动、转动
     * @param  locked 是否参与联动：ture 不参与， false 参与
     */
    CTransformation(double alpha, double a, double theta, double d, Joint_Type type, bool locked = true);

    /**
     * @brief 关节DH参数
     * @param val 关节位置
     * @param free 是否参与联动：ture 参与， false 不参与
     */
    void Set_Val(double Val, bool free);

    /**
     * @brief 关节DH参数
     * @param free 是否参与联动：ture 参与， false 不参与
     */
    void Set_free(bool free);

    /**
     * @brief 运算符重载
     * @return
     */
    CTransformation operator+(double Change_Val);

    /**
     * @brief 获取变换矩阵
     * @return 变换矩阵
     */
    Eigen::Matrix4d getTransformation() { return Transformation_Mat; }

    /**
     * @brief 获取关节类型
     */
    Joint_Type Get_Type() { return Type; }

    /**
     * @brief 获取关节锁定状态
     */
    bool Is_Free() { return Is_free; }

    double getJointPos();

private:
    /**
     * @brief 计算DH转换矩阵
     */
    void Cal_Transformation();

    Eigen::Matrix4d Transformation_Mat = Eigen::Matrix4d::Identity();

    double Alpha;
    double A;
    double THETA;
    double D_const;

    double Theta = 0;
    double D = 0;
    Joint_Type Type = Revolute;
    bool Is_free = true; // 是否允许运行
};

#endif // CTRANSFORMATION_H

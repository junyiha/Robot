#include "Transformation.h"


CTransformation::CTransformation(double alpha, double a, double theta, double d , Joint_Type type, bool locked)
    :Alpha(alpha),A(a),THETA(theta),D_const(d)
{


    Alpha = alpha;
    A = a;
    THETA = theta;
    D_const = d;

    Theta = theta;
    D = d;

    Type = type;
    Is_free = locked;
    Cal_Transformation();
}

void CTransformation::Set_Val(double Val,bool free)
{
    Is_free = free;
    //设置DH角度参数
    switch (Type)
    {
    case Revolute:
        Theta = Val+THETA;
        break;
    case Prismatic:
        D = Val+D_const;
        break;
    default:
        break;
    }
    Cal_Transformation();
}
void CTransformation::Set_free(bool free)
{
    Is_free = free;
}

CTransformation CTransformation::operator+(double Change_Val)
{
    switch (Type)
    {
    case Revolute:
    default:
        return CTransformation(Alpha, A, Theta+ Change_Val, D, Revolute);
        break;
    case Prismatic:
        return CTransformation(Alpha, A, Theta, D+ +Change_Val, Prismatic);
        break;
    }
}

void CTransformation::Cal_Transformation()
{
    //重新计算
    auto Transformation = Eigen::AngleAxisd(Alpha, Eigen::Vector3d::UnitX()) * Eigen::Translation3d(A, 0, 0);//α
    Transformation = Transformation * Eigen::AngleAxisd(Theta, Eigen::Vector3d::UnitZ()) * Eigen::Translation3d(0, 0, D);//θ
    Transformation_Mat = Transformation.matrix();
}

double CTransformation::getJointPos()
{
    //设置DH角度参数
    switch (Type)
    {
    case Revolute:
    default:
        return Theta - THETA;
    case Prismatic:
        return D - D_const;
    }
}


#include "RobotKinectModel.h"


CRobotKinectModel::CRobotKinectModel(std::vector<CTransformation> Input_DH_List, CTransformation Tool_DH )
{
    m_SerialLink = Input_DH_List;
    m_ToolLink   = Tool_DH;
    Valid_Index.clear();
    m_CylinderLen = 1200;
    log = spdlog::get("logger");
}

void CRobotKinectModel::setJointPos(std:: vector<std::pair<double, bool>> Joint_Value)
{

    //修改827
    //推缸行程转为旋转角度
    Joint_Value[5].first =Joint2Fkine(Joint_Value[5].first);
    m_CylinderLen = Joint_Value[CYLINDER_INDEX].first;

    if (Joint_Value.size() != m_SerialLink.size())
    {
        qDebug()<<"Joint_Value.size() != m_SerialLink.size()";
        return;
    }//Mark()这里应该抛出错误，以后再说
    for (int i = 0; i < Joint_Value.size(); i++)
    {
        m_SerialLink[i].Set_Val(Joint_Value[i].first, Joint_Value[i].second);
    }
    Transformation();
    calJacobian();

}

void CRobotKinectModel::setToolPos(double len, double rot)
{
    m_ToolLink.Set_Val(len, false); //末端工具改变长度
    m_ToolLink.Set_Val(rot,true);   //末端托盘旋转
    //Transformation();
}

Eigen::VectorXd CRobotKinectModel::getJointVel_nolimit(Eigen::VectorXd Tar_Speed,bool Ref_End)
{

    Eigen::Matrix3d End_RotateMat = Eigen::Matrix3d::Identity();//末端单位旋转矩阵
    if (Ref_End)
    {
        End_RotateMat = TransMatLisForward.back().topLeftCorner(3, 3).eval();//末端矩阵旋转
    }
    Eigen::MatrixXd End_Transformation_Mat = Eigen::MatrixXd::Zero(6,6);
    End_Transformation_Mat.topLeftCorner(3, 3) = End_RotateMat.eval();
    End_Transformation_Mat.bottomRightCorner(3, 3) = End_RotateMat.eval();


    Eigen::VectorXd Joint_Speed = JacobianMatInv * End_Transformation_Mat * Tar_Speed;
    //cout << JacobianMatInv * End_Transformation_Mat << endl << endl;

    Eigen::VectorXd Full_Joint_Speed = Eigen::VectorXd::Zero(m_SerialLink.size());

    for (int i = 0; i < Valid_Index.size(); i++)
    {
        Full_Joint_Speed[Valid_Index[i]] = Joint_Speed[i];
        //qDebug()<<"Full_Joint_Speed"<<Valid_Index[i]<<":"<<Joint_Speed[i];
    }

    //修改827
    //推缸关节速度
    double theta = Joint2Fkine(m_SerialLink[5].getJointPos());
    Full_Joint_Speed[5] = Joint2InvKine(m_SerialLink[5].getJointPos(),m_CylinderLen);


    return Full_Joint_Speed;
}

Eigen::VectorXd CRobotKinectModel::getJointVel(Eigen::VectorXd Tar_Speed,bool Ref_End)
{
    for (int i = 0; i < m_SerialLink.size(); i++)
    {
        m_SerialLink[i].Set_free(true);
    }
    //m_SerialLink[4].Set_free(false);

    calJacobian();
    Eigen::VectorXd speed = getJointVel_nolimit(Tar_Speed,Ref_End);


    bool lock = false; //判断是否需要重新计算雅可比
    //根据限位判断是否锁定轴
    for (int i = 0; i < m_SerialLink.size(); i++)
    {
        if(m_SerialLink[i].Get_Type() == Revolute) //旋转轴
        {
            if( ((LINK_0_JOINT_LIMIT_POS[i]/57.3 - m_SerialLink[i].getJointPos()) < 0.5/57.3  && speed[i]>0)||
              (  (m_SerialLink[i].getJointPos() - LINK_0_JOINT_LIMIT_NEG[i]/57.3) < 0.5/57.3 && speed[i]<0))
            {
                qDebug()<<QString("-------------Lock joint  %1----------- ").arg(i);
                qDebug()<<        "               Revolute                ";
                qDebug()<<QString("%1---LINK_0_JOINT_LIMIT_POS[ ]/57.3 :  ").arg(i)<<LINK_0_JOINT_LIMIT_POS[i]/57.3;
                qDebug()<<QString("%1---m_SerialLink[ ].getJointPos()  :  ").arg(i)<<m_SerialLink[i].getJointPos();
                qDebug()<<QString("%1---LINK_0_JOINT_LIMIT_NEG[ ]/57.3 :  ").arg(i)<<LINK_0_JOINT_LIMIT_NEG[i]/57.3;
                qDebug()<<"\n";
                //m_SerialLink[i].Set_free(false); //锁定轴
                lock = true;
            }
        }
        else // 平移轴
        {
            if( ((LINK_0_JOINT_LIMIT_POS[i]- m_SerialLink[i].getJointPos()) <1  && speed[i]>0)||
              (  (m_SerialLink[i].getJointPos() - LINK_0_JOINT_LIMIT_NEG[i])<1 && speed[i]<0))
            {
                qDebug()<<QString("-------------Lock joint  %1----------- ").arg(i);
                qDebug()<<        "               Prismatic               ";
                qDebug()<<QString("%1---LINK_0_JOINT_LIMIT_POS[ ]    :  ").arg(i)<<LINK_0_JOINT_LIMIT_POS[i];
                qDebug()<<QString("%1---m_SerialLink[].getJointPos() :  ").arg(i)<<m_SerialLink[i].getJointPos();
                qDebug()<<QString("%1---LINK_0_JOINT_LIMIT_NEG[ ]    :  ").arg(i)<<LINK_0_JOINT_LIMIT_NEG[i];
                qDebug()<<"\n";
                //m_SerialLink[i].Set_free(false); //锁定轴
                lock = true;
            }
        }
    }

    //修改827
    //设置各轴速度为0;(有冗余轴的情况下重新计算)
    if(lock == true)
    {
        log->warn("轴达到限位或被锁定，无有效解");
        return Eigen::VectorXd::Zero( m_SerialLink.size());
    }else
    {
        return speed;
    }

}


Eigen::VectorXd CRobotKinectModel::getEndPosition()
{
    Eigen::VectorXd pos = Eigen::VectorXd::Zero(6);
    if(TransMatLisForward.empty())
    {
        return  pos;
    }
    Eigen::MatrixXd RT = TransMatLisForward.back();

    Eigen::Matrix3d Rotate;
    for(int i = 0;i<3;i++)
    {
        for(int j = 0;j<3;j++)
        {
            Rotate(i,j) = RT(i,j);
        }
    }

    //Eigen::Vector3d  euler_angles =Rotate.matrix().eulerAngles(2,1,0);

//    Eigen::Vector3d PI_V;
//    PI_V.setConstant(3.1415926);
//    euler_angles = (euler_angles.array()<3.1415926/2).select(euler_angles,euler_angles-PI_V);
    //cout<<Rotate<<endl;

    //纠正2024/05/11  此处为固定角ZYX，rotx(α)*roxy(β)*rotz(γ)
    double RY = atan2( Rotate(0,2),sqrt(Rotate(1,2) * Rotate(1,2) + Rotate(2,2)*Rotate(2,2) ));
    if(fabs(RY-M_PI/2) < 0.0001){
        RY = M_PI/2 - 0.0001;
    }

    double RX = atan2( -Rotate(1,2)/cos(RY),Rotate(2,2)/cos(RY) );
    double RZ = atan2( -Rotate(0,1)/cos(RY),Rotate(0,0)/cos(RY) );

    pos[0] = RT(0,3);
    pos[1] = RT(1,3);
    pos[2] = RT(2,3);
    pos[3] = RX;
    pos[4] = RY;
    pos[5] = RZ;

    return pos;
}

void CRobotKinectModel::Transformation()
{

    TransMatLisForward.clear();
    TransMatLisBackward.clear();

    Eigen::Matrix4d End_Transformation = Eigen::Matrix4d::Identity();           //前向迭代矩阵
    Eigen::Matrix4d Back_End_Transformation = m_ToolLink.getTransformation();   //逆向迭代矩阵

    TransMatLisForward.push_back(End_Transformation.eval()); //最初基础坐标系存在一个单位变换
    TransMatLisBackward.push_front(Back_End_Transformation.eval()); //最末端基础坐标系存在工具变换

    for (int i = 0; i < m_SerialLink.size(); i++)
    {
        //从前向后乘
        End_Transformation = End_Transformation * m_SerialLink[i].getTransformation();
        TransMatLisForward.push_back(End_Transformation);
        //从后向前乘
        Back_End_Transformation = m_SerialLink[m_SerialLink.size() - 1 - i].getTransformation() * Back_End_Transformation;
        TransMatLisBackward.push_front(Back_End_Transformation);
    }
    TransMatLisForward.push_back(TransMatLisForward.back() * m_ToolLink.getTransformation());//得到末端姿态
}


void CRobotKinectModel::calJacobian()
{
    //cout<<m_SerialLink.back().getTransformation()<<endl;

    Eigen::MatrixXd Valid_Matrix;
    Valid_Index.clear();
    Eigen::MatrixXd Temp_Jacobian_Mat = Eigen::MatrixXd::Zero(6, m_SerialLink.size());//全雅克比矩阵
    for (int i = 0; i < m_SerialLink.size(); i++)
    {
        Eigen::VectorXd Jacobian_Col = Eigen::VectorXd::Zero(6);
        if (m_SerialLink[i].Is_Free())
        {
            auto Transfomation_Joint = (m_SerialLink[i] + Partical_Delta).getTransformation(); //引入微小扰动，修改关节变换矩阵
            Jacobian_Col.topRows(3) = ((TransMatLisForward[i] * Transfomation_Joint * TransMatLisBackward[i + 1] - TransMatLisBackward.front()).topRightCorner(3, 1).eval() / Partical_Delta); //偏微分
            if (m_SerialLink[i].Get_Type() == Revolute)
            {
                Jacobian_Col.bottomRows(3) = TransMatLisForward[i + 1].topRows(3).col(2);
            }
            Valid_Index.push_back(i);
        }
        Temp_Jacobian_Mat.col(i) = Jacobian_Col;
    }
    Valid_Matrix = Eigen::MatrixXd::Zero(m_SerialLink.size(), Valid_Index.size());
    for (int i = 0; i < Valid_Index.size(); i++)
    {
        Valid_Matrix(Valid_Index[i], i) = 1;
    }

    JacobianMat = Temp_Jacobian_Mat * Valid_Matrix;
    JacobianMatInv = JacobianMat.completeOrthogonalDecomposition().pseudoInverse();
}


Eigen::VectorXd CRobotKinectModel::end2base(Eigen::VectorXd end)
{
    Eigen::Vector3d position = end.head(3);
    Eigen::Vector3d euler_angles = end.tail(3);
    Eigen::Quaterniond quat = Eigen::Quaterniond(Eigen::AngleAxisd(euler_angles(0),Eigen::Vector3d::UnitX()))
                            *Eigen::Quaterniond(Eigen::AngleAxisd(euler_angles(1),Eigen::Vector3d::UnitY()))
                            *Eigen::Quaterniond(Eigen::AngleAxisd(euler_angles(2),Eigen::Vector3d::UnitZ()));

    Eigen::Vector4d endpos =  Eigen::Vector4d::Identity();
    endpos << end[0],end[1],end[2],1.0;

    Eigen::Vector4d basepos= TransMatLisBackward.back() * endpos;

    Eigen::Matrix baseangle = TransMatLisBackward.back().block(0,0,3,3)*quat;

    //Eigen::VectorXd angle = baseangle.eulerAngles(0,1,2);

    Eigen::VectorXd base ;
    base << basepos.head(3),baseangle.eulerAngles(0,1,2);
    return base;

}


Eigen::MatrixXd CRobotKinectModel::getTarEndPos(Eigen::VectorXd &deltaPos)
{
//    updateJoint_Value();
//    m_cRobotCal.Set_Joint_Val(Joint_Value);
//    m_cRobotCal.Set_Tool_Length(L6);
    //cout<<"curEndPos:"<<endl<<m_cRobotCal.End_Pose()<<endl;
    Eigen::Matrix4d tarEndPos = TransMatLisForward.back();
    auto Temp_Translation = Eigen::Translation3d(deltaPos[0], deltaPos[1], deltaPos[2])*Eigen::AngleAxisd(deltaPos[3], Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(deltaPos[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(deltaPos[5], Eigen::Vector3d::UnitZ()) ;

    tarEndPos = tarEndPos * Temp_Translation.matrix();

    return tarEndPos;
}

Eigen::VectorXd CRobotKinectModel::getTarSpeed(Eigen::Matrix4d tarEndPos)
{

    Eigen::Matrix4d thisEndPos = TransMatLisForward.back();

    Eigen::Matrix3d Rotation_Delta =  tarEndPos.topLeftCorner(3,3)*thisEndPos.topLeftCorner(3,3).transpose() ; //
    Eigen::Vector3d Translation_Delta =tarEndPos.topRightCorner(3,1) - thisEndPos.topRightCorner(3,1);

    //固定角ZYX，rotx(α)*roxy(β)*rotz(γ)
    double RY = atan2( Rotation_Delta(0,2),sqrt(Rotation_Delta(1,2) * Rotation_Delta(1,2) + Rotation_Delta(2,2)*Rotation_Delta(2,2) ));
    if(RY == M_PI/2){
        RY = M_PI/2 - 0.0001;
    }
    double RX = atan2( -Rotation_Delta(1,2)/cos(RY),Rotation_Delta(2,2)/cos(RY) );
    double RZ = atan2( -Rotation_Delta(0,1)/cos(RY),Rotation_Delta(0,0)/cos(RY) );

    Eigen::VectorXd TarSpeed(6) ;
    TarSpeed << Translation_Delta.x(), Translation_Delta.y(), Translation_Delta.z(), RX,RY,RZ; //速度 Vx Vy Vz Wx Wy Wz

    return TarSpeed;
}

Eigen::VectorXd CRobotKinectModel::getTarSpeed(Eigen::VectorXd tarEndPos)
{
    Eigen::VectorXd vel;
    vel.resize(6);
    vel.setZero();
    if(tarEndPos.size() != 6)
    {
        log->error("ERROR:Pose vector size must be 6");
        return vel;
    }

    //构建位姿矩阵:末端矢量 -> RT
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(tarEndPos[3], Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(tarEndPos[4], Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(tarEndPos[5], Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d tar_rt = Eigen::Matrix4d::Identity();
    tar_rt.block<3, 3>(0, 0) = rotation_matrix;
    tar_rt(0, 3) = tarEndPos[0];
    tar_rt(1, 3) = tarEndPos[1];
    tar_rt(2, 3) = tarEndPos[2];

    return getTarSpeed(tar_rt);

}
//修改827
double CRobotKinectModel::Joint2Fkine(double D2)
{
    double a1 = atan(330.0/400.0);
    double a3 = atan(850.0/250.0);
    double R3 = 886;
    double R4 = 518.5;
    double Theta = (a1 + a3 - acos(( D2 * D2 - R3 * R3 - R4 * R4)/(2 * R3 * R4)));
    std::cout<<"Theta: "<<Theta<<std::endl<<std::endl;
    return Theta;
}
//修改827
double CRobotKinectModel::Joint2InvKine(double Theta2Dot,double D2)
{
    //Theta2Dot → D2Dot
    double S = sqrt(1 - (D2*D2/918782-4215353/3675128)*(D2*D2/918782-4215353/3675128));
    double D2Dot = 459391 * S * Theta2Dot / D2;
    return D2Dot;
}
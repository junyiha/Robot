/**
 * @file manual.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

	// void CTask::Manual()			
	// {			
	//     // 急停			
	//     if(m_Manual.Estop == true || m_HmiCmd == eStop)			
	//     {			
	//         m_Robot->setRobotStop();			
				
	//         return;			
	//     }			
				
	//     //首次进入手动，先停止机器人，并修改状态			
	//     if(m_AutoWorking == true)			
	//     {			
	//         m_AutoWorking = false;			
	//         m_Robot->setRobotHalt();			
	//         return;			
	//     }			
				
				
	//     // 界面指令，优先响应界面指令			
	//     if(m_HmiCmd == eStart)			
	//     {			
	//         m_Robot->setLinkEnable(true);			
	//         return;			
	//     }			
	//     if(m_HmiCmd == eCLOSE)			
	//     {			
	//         m_Robot->setLinkEnable(false);			
	//         return;			
	//     }			
	//     if(m_HmiCmd == eReset)			
	//     {			
	//         m_Robot->setRobotReset();			
	//         return;			
	//     }			
				
	//     //车移动==================================================================			
				
	// 	//舵轮控制		
	//     if(fabs(m_Manual.MoveDirection - m_preManual.MoveDirection)>1)			
	//     {			
	//         m_Robot->setJointMoveAbs(STEER_LEFT_INDEX, m_Manual.MoveDirection,10);//速度需改为参数			
	//         m_Robot->setJointMoveAbs(STEER_RIGHT_INDEX,m_Manual.MoveDirection,10);//速度需改为参数			
	//     }			
				
	//     double vel_left,vel_right ;			
	//     if(m_JointGroupStatus.AxisGroup[STEER_LEFT_INDEX].eState == eAxis_STANDSTILL &&			
	//             m_JointGroupStatus.AxisGroup[STEER_RIGHT_INDEX].eState == eAxis_STANDSTILL)//左右舵轮均不动			
	//     {			
	//          if(m_Manual.bMove || m_Manual.bRotate)			
	//          {			
	//             //计算轮速			
	//             if(fabs(m_Manual.MoveDirection)>M_PI/6)//舵轮角度大于45°时禁止差速转向			
	//             {			
	//                 m_Manual.MoveDirection = 0;			
	//             }			
	//             vel_left = m_Manual.MoveVel- m_Manual.RotateVel; //正转为逆时针			
	//             vel_right = m_Manual.MoveVel+ m_Manual.RotateVel;			
				
	//             m_Robot->setJointMoveVel(WHEEL_LEFT_INDEX,vel_left);			
	//             m_Robot->setJointMoveVel(WHEEL_RIGHT_INDEX,vel_left);			
	//          }			
	//     }			
				
	//      //==================================================================================			
	//     //优先级：固定点位移动 > 单轴移动			
	//     double jointvel[20];			
	//     double endvel[6] ={0,0,0,0,0,0};			
	//     if(m_Manual.bPickBoard )//放钉位			
	//     {			
	//          for(int i= 0;i<20;i++)			
	//          {			
	//             jointvel[i] = m_Manual.BoardMoveVel*JOINT_VEL_LIMIT[i];			
	//          }			
				
	//          m_Robot->setJointGroupMoveAbs(LOAD_POSTION,jointvel);			
	//     }else if(m_Manual.bPushBoard)			
	//     {			
	//          for(int i= 0;i<20;i++)			
	//          {			
	//             jointvel[i] = m_Manual.BoardMoveVel*JOINT_VEL_LIMIT[i];			
	//          }			
	//          m_Robot->setJointGroupMoveAbs(INTALL_POSTION,jointvel);			
	//     }			
	//     else			
	//     {			
	//          if(m_Manual.bRobotMove)			
	//          {			
	//             if(m_Manual.EndMove != m_preManual.EndMove) //切换移动模式			
	//             {			
	//                 m_Robot->setRobotHalt();			
				
	//                 return;			
	//             }			
	//             if(m_Manual.EndMove == true)//末端运动			
	//             {			
	//                 for(int i= 0;i<6;i++)			
	//                 {			
	//                     endvel[i] = m_Manual.RobotMove[i]*END_VEL_LIMIT[i];			
	//                 }			
	//                 m_Robot->setLinkMoveVel(endvel);			
				
	//             }else//单轴运动			
	//             {			
	//                 //轴索引待定			
				
	//                 jointvel[0] = m_Manual.RobotMove[0]*JOINT_VEL_LIMIT[0];			
	//                 jointvel[1] = m_Manual.RobotMove[1]*JOINT_VEL_LIMIT[1];			
	//                 jointvel[2] = m_Manual.RobotMove[2]*JOINT_VEL_LIMIT[2];			
	//                 jointvel[3] = m_Manual.RobotMove[3]*JOINT_VEL_LIMIT[3];			
	//                 jointvel[4] = m_Manual.RobotMove[4]*JOINT_VEL_LIMIT[4];			
	//                 jointvel[5] = m_Manual.RobotMove[5]*JOINT_VEL_LIMIT[5];			
	//                 m_Robot->setJointGroupMoveVel(jointvel);			
	//             }			
	//          }			
	//     }			
	// }			

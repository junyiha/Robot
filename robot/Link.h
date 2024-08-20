/****************************************************************/
/*功能：移动载体类
/*创建者：AT
/*重大改动及时间：
2017.5.14：  创建
*****************************************************************/
#pragma once
#include <stdlib.h>
#include "Axis.h"
#pragma pack(push)
#pragma pack(8) 

class CLink
{
public:
	CLink(int linkindex, int freedom);
	~CLink();
public:
	ERR_ID Init(int JointIndex, stJointPara JointPar[]);											//车体初始化
	void GetStatus(_IN st_AxisGroupRead* stAxisGroupStatus);
	void StateMachine();				 									//状态机（运行函数）
	void SetAction(OUT st_AxisGroupSet*	stAxisGroupSet);					//机器人指令
	
	//LINK操作接口----------------------------------------------------------------------//
	void CommdReset();
	void CommdPower(bool enable);
	void CommdHome();
	void CommdStop();
	void CommdHalt();
	void CommdMove();

protected:
	void JugeLinkState();
	void LinkStateMachine();
	

	//可重载函数----------------------------------------------------------------------//
	//正解
	virtual void LinkForwardKin() const {}			//Link 正解
	//逆解
	virtual void MotionLineAbsolute() const{}	//沿直线运动到绝对位置	
	virtual void MotionLineVelocity() const{}	//末端速度运动
	virtual void MotionLineRelative() const {}	//相对位置模式
	virtual void MotionMode1() const {}			//自定义1
	virtual void MotionMode2() const {}			//自定义2
	virtual void MotionMode3() const {}			//自定义3
	virtual void MotionMode4() const {}			//自定义4
	virtual void MotionMode5() const {}			//自定义5
	virtual void MotionMode6() const {}			//自定义6
	virtual void MotionMode7() const {}			//自定义7
	virtual void MotionMode8() const {}			//自定义8
	virtual void MotionMode9() const {}			//自定义9
	
	virtual void SetLinkSelfAction() const{}		//Link操作，如末端工具，水炮枪逻辑控制

private:
	virtual void ActionReset();
	virtual void ActionPower();
	virtual void ActionHome();
	virtual void ActionStop();
	virtual void ActionHalt();
	virtual void ActionMove();

public:
	CAxis*			m_Joints[MAX_FREEDOM_LINK];
	int				m_Index;			//按顺序连续编号
	int				m_Freedom;

	stLinkStatus	m_Status;
	stLinkCommand	m_Command;
	stAction		m_SetAction;		//PLC执行
	stAction		m_ActAction;		//PLC当前动作
private:
	stLinkCommand	m_preCommand;
};
#pragma pack(pop)
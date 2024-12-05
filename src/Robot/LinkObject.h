#pragma once
#include "Link.h"
#pragma pack(push)
#pragma pack(8)

const double MoveVel = 10; //

class CLink_0 : public CLink
{
	CLink_0(int linkindex, int freedom) : CLink(linkindex, freedom)
	{
		X_CameraToScrew = 0;
		Y_CameraToScrew = 0;
		Z_FirstDistance = 0;
		Z_SecondDistance = 0;
		beConnect = false;
	}
	~CLink_0()
	{
	}
	// 可重载函数====================================================//
	// 正解
	virtual void LinkForwardKin(); // Link 正解
	// 逆解
	virtual void MotionLineAbsolute(); // 单轴运动
	virtual void MotionLineVelocity(); // 末端位置模式
	virtual void MotionLineRelative(); // 末端速度模式

	virtual void MotionMode1();			// X.Y.Z运动
	virtual void MotionMode2();			// 手动模式
	virtual void MotionMode3() const {} // 自定义3
	virtual void MotionMode4() const {} // 自定义4
	virtual void MotionMode5() const {} // 自定义5
	virtual void MotionMode6() const {} // 自定义6
	virtual void MotionMode7() const {} // 自定义7
	virtual void MotionMode8() const {} // 自定义8
	virtual void MotionMode9() const {} // 自定义9

	virtual void SetLinkSelfAction() const; // Link操作，如末端工具，水炮枪

public:
	double X_CameraToScrew;
	double Y_CameraToScrew;
	double Z_FirstDistance;
	double Z_SecondDistance;
	bool beConnect;
};

class CLink_1 : public CLink
{
	CLink_1(int linkindex, int freedom) : CLink(linkindex, freedom)
	{
	}
	~CLink_1()
	{
	}
	// 可重载函数====================================================//
	// 正解
	virtual void LinkForwardKin() const; // Link 正解
	// 逆解
	virtual void MotionLineAbsolute() const; // 单轴运动
	virtual void MotionLineVelocity() const; // 末端位置模式
	virtual void MotionLineRelative() const; // 末端速度模式

	virtual void MotionMode1() const {} // 自定义1
	virtual void MotionMode2() const {} // 自定义2
	virtual void MotionMode3() const {} // 自定义3
	virtual void MotionMode4() const {} // 自定义4
	virtual void MotionMode5() const {} // 自定义5
	virtual void MotionMode6() const {} // 自定义6
	virtual void MotionMode7() const {} // 自定义7
	virtual void MotionMode8() const {} // 自定义8
	virtual void MotionMode9() const {} // 自定义9

	virtual void SetLinkSelfAction() const; // Link操作，如末端工具，水炮枪
};
class CLink_2 : public CLink
{
	CLink_2(int linkindex, int freedom) : CLink(linkindex, freedom)
	{
	}
	~CLink_2()
	{
	}
	// 可重载函数====================================================//
	// 正解
	virtual void LinkForwardKin() const; // Link 正解
										 // 逆解
	virtual void MotionLineAbsolute() const; // 单轴运动
	virtual void MotionLineVelocity() const; // 末端位置模式
	virtual void MotionLineRelative() const; // 末端速度模式

	virtual void MotionMode1() const {} // 自定义1
	virtual void MotionMode2() const {} // 自定义2
	virtual void MotionMode3() const {} // 自定义3
	virtual void MotionMode4() const {} // 自定义4
	virtual void MotionMode5() const {} // 自定义5
	virtual void MotionMode6() const {} // 自定义6
	virtual void MotionMode7() const {} // 自定义7
	virtual void MotionMode8() const {} // 自定义8
	virtual void MotionMode9() const {} // 自定义9

	virtual void SetLinkSelfAction() const; // Link操作，如末端工具，水炮枪
};
class CLink_3 : public CLink
{
	CLink_3(int linkindex, int freedom) : CLink(linkindex, freedom)
	{
	}
	~CLink_3()
	{
	}
	// 可重载函数====================================================//
	// 正解
	virtual void LinkForwardKin() const; // Link 正解
										 // 逆解
	virtual void MotionLineAbsolute() const; // 单轴运动
	virtual void MotionLineVelocity() const; // 末端位置模式
	virtual void MotionLineRelative() const; // 末端速度模式

	virtual void MotionMode1() const {} // 自定义1
	virtual void MotionMode2() const {} // 自定义2
	virtual void MotionMode3() const {} // 自定义3
	virtual void MotionMode4() const {} // 自定义4
	virtual void MotionMode5() const {} // 自定义5
	virtual void MotionMode6() const {} // 自定义6
	virtual void MotionMode7() const {} // 自定义7
	virtual void MotionMode8() const {} // 自定义8
	virtual void MotionMode9() const {} // 自定义9

	virtual void SetLinkSelfAction() const; // Link操作，如末端工具，水炮枪
};
class CLink_4 : public CLink
{
	CLink_4(int linkindex, int freedom) : CLink(linkindex, freedom)
	{
	}
	~CLink_4()
	{
	}
	// 可重载函数====================================================//
	// 正解
	virtual void LinkForwardKin() const; // Link 正解
										 // 逆解
	virtual void MotionLineAbsolute() const; // 单轴运动
	virtual void MotionLineVelocity() const; // 末端位置模式
	virtual void MotionLineRelative() const; // 末端速度模式

	virtual void MotionMode1() const {} // 自定义1
	virtual void MotionMode2() const {} // 自定义2
	virtual void MotionMode3() const {} // 自定义3
	virtual void MotionMode4() const {} // 自定义4
	virtual void MotionMode5() const {} // 自定义5
	virtual void MotionMode6() const {} // 自定义6
	virtual void MotionMode7() const {} // 自定义7
	virtual void MotionMode8() const {} // 自定义8
	virtual void MotionMode9() const {} // 自定义9

	virtual void SetLinkSelfAction() const; // Link操作，如末端工具，水炮枪
};
class CLink_5 : public CLink
{
	CLink_5(int linkindex, int freedom) : CLink(linkindex, freedom)
	{
	}
	~CLink_5()
	{
	}
	// 可重载函数====================================================//
	// 正解
	virtual void LinkForwardKin() const; // Link 正解
										 // 逆解
	virtual void MotionLineAbsolute() const; // 单轴运动
	virtual void MotionLineVelocity() const; // 末端位置模式
	virtual void MotionLineRelative() const; // 末端速度模式

	virtual void MotionMode1() const {} // 自定义1
	virtual void MotionMode2() const {} // 自定义2
	virtual void MotionMode3() const {} // 自定义3
	virtual void MotionMode4() const {} // 自定义4
	virtual void MotionMode5() const {} // 自定义5
	virtual void MotionMode6() const {} // 自定义6
	virtual void MotionMode7() const {} // 自定义7
	virtual void MotionMode8() const {} // 自定义8
	virtual void MotionMode9() const {} // 自定义9

	virtual void SetLinkSelfAction() const; // Link操作，如末端工具，水炮枪
};

#pragma pack(pop)

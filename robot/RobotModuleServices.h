///////////////////////////////////////////////////////////////////////////////
// RobotModuleServices.h

#pragma once

typedef long LONG;

typedef unsigned char BYTE;
typedef unsigned long ULONG;

typedef long long DINT;

enum eMC_Motion : LONG
{
	eMC_NONE = 0,
	eMC_RESET = 1,
	eMC_HOME = 2,
	eMC_POWER = 3,
	eMC_STOP = 4,
	eMC_HALT = 5,
	eMC_MOV_ABS = 6,
	eMC_MOV_CON_ABS = 7,
	eMC_MOV_RELATIVE = 9,
	eMC_MOV_VEL = 10
};

enum eAxis_States : LONG
{
	eAxis_UNDEFINED = 0,
	eAxis_DISABLED = 1,
	eAxis_STANDSTILL = 2,
	eAxis_ERRORSTOP = 3,
	eAxis_STOPPING = 4,
	eAxis_HOMING = 5,
	eAxis_DISCRETEMOTION = 6,
	eAxis_CONTINOUSMOTION = 7,
	eAxis_SYNCHRONIZEDMOTION = 8
};

typedef struct _st_SetAxis
{
	ULONG Index;
	eMC_Motion eMC_Motion;
	bool bEnable;
	unsigned char reserved1[7];
	double Position;
	double Velocity;
	double Torque;
	double EndVelocity;
	double Distance;
} st_SetAxis, *Pst_SetAxis;

typedef struct _st_ReadAxis
{
	ULONG Index;
	eAxis_States eState;
	double Position;
	double Velocity;
	double Torque;
	double Current;
	double Acceleration;
	ULONG ErrorID;
	bool Error;
	bool bHomed;
	unsigned char reserved1[2];
} st_ReadAxis, *Pst_ReadAxis;

typedef struct _stAction
{
	bool DO0;
	bool DO1;
	bool DO2;
	bool DO3;
	LONG IDO0;
	LONG IDO1;
	LONG IDO2;
	LONG IDO3;
	unsigned char reserved1[4];
	double DA0;
	double DA1;
	double DA2;
	double DA03;

	_stAction &operator=(const _stAction &A)
	{
		DO0 = A.DO0;
		DO1 = A.DO1;
		DO2 = A.DO2;
		DO3 = A.DO3;
		IDO0 = A.IDO0;
		IDO1 = A.IDO1;
		IDO2 = A.IDO2;
		IDO3 = A.IDO3;

		for (int i = 0; i < 4; ++i)
		{
			reserved1[i] = A.reserved1[i];
		}

		DA0 = A.DA0;
		DA1 = A.DA1;
		DA2 = A.DA2;
		DA03 = A.DA03;
		return *this;
	}

} stAction, *PstAction;

typedef struct _st_AxisGroupSet
{
	ULONG Number;
	unsigned char reserved1[4];
	st_SetAxis AxisGroup[20];
	stAction RobotSetAction;
	stAction LinkSetActions[6];
} st_AxisGroupSet, *Pst_AxisGroupSet;

typedef struct _st_AxisGroupRead
{
	ULONG Number;
	unsigned char reserved1[4];
	st_ReadAxis AxisGroup[20];
	stAction RobotActAction;
	stAction LinkActActions[6];
} st_AxisGroupRead, *Pst_AxisGroupRead;

typedef BYTE Recvbuff[3000];

typedef BYTE Sendbuff[3000];

enum eConfigMode : ULONG
{
	eNone = 0,
	eInit = 1,
	eHomed = 2
};

typedef struct _st_InitParameter
{
	ULONG JointsNum;
	ULONG RecvSize;
	ULONG SendSize;
} st_InitParameter, *Pst_InitParameter;

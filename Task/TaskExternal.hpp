/*****************************************************************//**
 * \file   TaskExternal.hpp
 * \brief  
 * 
 * \author anony
 * \date   October 2024
 *********************************************************************/
#pragma once

enum class ETopState
{
    eManual = 0,        // �ֶ�
    eParallel,          // ��ƽ
    ePositioning,       // ��λ
    eReadToMagentOn,    // ������
    eDoWeld,            // ����
    eQuit               // �˳�
};

enum class ESubState
{
    eNULL = 0,          // ��״̬
    eNotReady = 0,      // δ����
    eReady,             // ����
    eMotion,            // �˶�

    eReadyToParallel,   // ����ƽ
    eDetection,         // ���

    eReadyToPositioning,// ����λ

    eReadyToDoWeld,     // ������
    eDoingWeld,         // ������
    eStopWeld,          // ����ֹͣ

    eQuiting,           // �˳���
    ePause              // ��ͣ
};

enum class EExecutionCommand
{
    eNULL = 0,          // ��ָ��
    eManual,            // �ֶ�ָ��
    eParallel,          // ��ƽ
    eTerminate,         // ��ֹ
    ePause,             // ��ͣ
    ePositioning,       // ��λ
    eMagentOn,          // ����
    eQuit,              // �˳�
    eAutoWeld,          // �Զ�����
    eMagentOff,         // �ѿ�
    eStopWeld,          // ֹͣ����
    eSideline,          // ������� == ��λ(ePositioning)
    eLift,              // ���� (�ֶ�ָ���е�)
    eAddNail,           // �Ŷ� (�ֶ�ָ���е�)
    eStop,              // ֹͣ (�ֶ�ָ���е�)
    eCrashStop,         // ��ͣ (�ֶ�ָ���е�)
};

enum class EDetectionInParallelResult
{
    eDeviationIsLessThanThreshold = 0,     // ���⴫����ƫ��С����ֵ
    eDistanceMeetsRequirement,             // ��ھ����������Ҫ��
    eNoWallDetected                        // δ��⵽����
};

enum class EDetectionInPositioningResult
{
    eDeviationIsLessThanThreshold = 0,     // ����ƫ��С����ֵ
    eEndAdjustmentDataIsValid,             // ĩ�˵������ݺϷ�
    eDataIsInvalid,                         // ���ݷǷ�
};
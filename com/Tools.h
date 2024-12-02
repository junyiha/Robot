/*****************************************************************
 * 类名称： 工具功能实现
 * 功能描述： ·硬件整合，将两个IO板与工具动作绑定，提供统一控制接口
 * 参数说明： 参数说明
 * 返回值：   返回值说明
 * 硬件功能--------------------------------------------------------
 * 硬件清单：IO板A(编号01),B（编号02）, 工具10套，磁铁*4 磁铁推杆*4 十字激光*1 焊枪切换开关*10
 * A板   工具1    工具2    工具3    工具4    工具5     磁铁吸合   推缸伸    推缸缩     十字激光  焊枪切换1~5
 *       DO0~7  DO8~15   DO16~23  DO24~31 DO32~39  DO40,43  DO41,44  DO42,DO45   DO47   DO48~52
 * a-1-5,b-6-10, A-激光
 * B板   工具6    工具7    工具8    工具9    工具10    磁铁吸合   推缸伸    推缸缩              焊枪切换6~10
 *       DO0~7  DO8~15   DO16~23  DO24~31 DO32~39  DO40,43  DO41,44  DO42,DO45          DO48~52
 *
 * 工具IO功能编号：  碰钉位  打磨位  定位气缸  打磨顶升  碰钉顶升   碰钉下降   打磨    碰钉
 *                  0       1      2       3       4          5      6       7
 ******************************************************************/

#ifndef CTOOLS_H
#define CTOOLS_H

#include "IOCom.h"
#include <QObject>
#include <Eigen/Dense>

enum E_WeldAction : byte
{
    eInitAction = 0,              // 初始姿态
    eGrind_MovorOff = 0b00000001, // 移动到打磨位 or 打磨关闭
    eGrind_OnorDown = 0b01000001, // 打磨开启 or 打磨下降
    eGrind_Up = 0b01001001,       // 打磨举升
    eWeld_MovorDwon = 0b00000010, // 移动到焊接位
    eWeld_Fix = 0b00000110,       // 定位气缸开
    eWeld_Up = 0b00010110,        // 焊接举升
    eWeld_On = 0b10010110,        // 焊接起弧
    eWeld_Down = 0b00100010,      // 焊接起弧关+焊接下降+定位气缸关
    eNone_Action = 255            // 初始状态
};
enum E_MagentAction
{
    eNONE_Magent = 0, // 不改变状态
    eMag_On = 1,
    eMag_Off = 2,
    eMag_Up = 3,
    eMag_Down = 4,
    eMag_Stop = 5

};

class CTools
{
public:
    CTools();
    IOCom m_cIOA;
    IOCom m_cIOB;

    //****************功能接口***************
    /**
     * @brief 工具控制
     * @param  index 工具编号1~10
     * @param  action 动作类型
     * @return
     */
    void SetToolsAction(quint8 index, E_WeldAction action);

    /**
     * @brief 磁铁控制
     * @param  index 磁铁编号1,2,3,4; 0表示所有磁铁
     * @return action 动作类型
     */
    void SetMagentAction(quint8 index = 0, E_MagentAction action = eNONE_Magent);

    /**
     * @brief 激光控制
     * @param  swtich 开关量
     */
    void SetCrossLasser(bool swtich);

    /**
     * @brief 焊枪切换
     * @param  index
     */
    void SetGunConnect(qint8 index);

    QVector<double> getLaserDistance();

    void close()
    {
        m_cIOA.close();
        m_cIOB.close();
    }

protected:
    std::shared_ptr<spdlog::logger> log;
};

#endif // CTOOLS_H

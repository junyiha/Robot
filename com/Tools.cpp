#include "Tools.h"

CTools::CTools()
{
    log = spdlog::get("logger");
}

void CTools::SetToolsAction(quint8 index, E_WeldAction action)
{

    if (index >= 1 && index <= 5) // A板
    {
        if (action != eNone_Action)
        {
            m_cIOA.SetIO(index - 1, action);
        }
        log->debug("SetToolsAction{}:{}", index, (quint8)action);
    }
    else if (index >= 6 && index <= 10) // B板
    {
        if (action != eNone_Action)
        {
            m_cIOB.SetIO(index - 6, action);
        }
        log->debug("SetToolsAction{}:{}", index, (quint8)action);
    }
    else
    {
        qDebug() << "SetToolsAction： err index";
    }
}

void CTools::SetMagentAction(quint8 index, E_MagentAction action)
{
    byte value_A = m_cIOA.getDOState()[5]; // 磁铁输出状态
    byte value_B = m_cIOB.getDOState()[5]; // 磁铁输出状态
    switch (index)
    {
    case 1:
        switch (action)
        {
        case eMag_On:
            value_A |= 0b00000001;
            break;

        case eMag_Off:
            value_A &= ~0b00000001;
            break;

        case eMag_Up:
            value_A |= 0b00000010;
            value_A &= ~0b00000100;
            break;

        case eMag_Down:
            value_A |= 0b00000100;
            value_A &= ~0b00000010;
            break;
        case eMag_Stop:
            value_A &= ~0b00000010;
            value_A &= ~0b00000100;
            break;

        case eNONE_Magent:
        default:
            break;
        }
        break;

    case 2:
        switch (action)
        {
        case eMag_On:
            value_A |= 0b00001000;
            break;

        case eMag_Off:
            value_A &= ~0b00001000;
            break;

        case eMag_Up:
            value_A |= 0b00010000;
            value_A &= ~0b00100000;
            break;

        case eMag_Down:
            value_A |= 0b00100000;
            value_A &= ~0b00010000;
            break;
        case eMag_Stop:
            value_A &= ~0b00000010;
            value_A &= ~0b00000100;
            break;
        case eNONE_Magent:
        default:
            break;
        }
        break;
    case 3:
        switch (action)
        {
        case eMag_On:
            value_B |= 0b00000001;
            break;

        case eMag_Off:
            value_B &= ~0b00000001;
            break;

        case eMag_Up:
            value_B |= 0b00000010;
            value_B &= ~0b00000100;
            break;

        case eMag_Down:
            value_B |= 0b00000100;
            value_B &= ~0b00000010;
            break;
        case eMag_Stop:
            value_A &= ~0b00000010;
            value_A &= ~0b00000100;
            break;
        case eNONE_Magent:
        default:
            break;
        }
        break;

    case 4:
        switch (action)
        {
        case eMag_On:
            value_B |= 0b00001000;
            break;

        case eMag_Off:
            value_B &= ~0b00001000;
            break;

        case eMag_Up:
            value_B |= 0b00010000;
            value_B &= ~0b00100000;
            break;

        case eMag_Down:
            value_B |= 0b00100000;
            value_B &= ~0b00010000;
            break;
        case eMag_Stop:
            value_A &= ~0b00000010;
            value_A &= ~0b00000100;
            break;
        case eNONE_Magent:
        default:
            break;
        }
        break;
    case 0:
        switch (action)
        {
        case eMag_On:
            value_A |= 0b00001001;
            value_B |= 0b00001001;
            value_A &= ~0b00110110;
            value_B &= ~0b00110110;
            break;

        case eMag_Off:
            value_A &= ~0b00001001;
            value_B &= ~0b00001001;
            break;

        case eMag_Up:
            value_A |= 0b00010010;
            value_A &= ~0b00100100;
            value_B |= 0b00010010;
            value_B &= ~0b00100100;
            break;

        case eMag_Down:
            /* value_A |=  0b00100100;
             value_A &= ~0b00010010;
             value_B |=  0b00100100;
             value_B &= ~0b00010010;*/
            value_A |= 0b00100100;
            value_A &= ~0b00011011;
            value_B |= 0b00100100;
            value_B &= ~0b00011011;
            break;
        case eMag_Stop:
            value_A &= ~0b00110110;
            value_B &= ~0b00110110;
            break;
        case eNONE_Magent:
        default:
            break;
        }
    }
    m_cIOA.SetIO(5, value_A);
    m_cIOB.SetIO(5, value_B);

    // qDebug() << "value_A value_B" << value_A << ";" << value_B;
}

void CTools::SetCrossLasser(bool swtich)
{
    byte value_A = m_cIOA.getDOState()[5];
    if (swtich == true)
    {
        value_A |= 0b10000000;
    }
    else
    {
        value_A &= ~0b10000000;
    }
    m_cIOA.SetIO(5, value_A);
}

void CTools::SetGunConnect(qint8 index)
{
    byte value = 0;
    if (index >= 1 && index <= 5) // A板
    {
        value = 1 << (index - 1);
        m_cIOA.SetIO(6, value);
    }
    else if (index >= 6 && index <= 10) // B板
    {
        value = 1 << (index - 6);
        m_cIOB.SetIO(6, value);
    }
    else
    {
        // 断开所有
        m_cIOA.SetIO(6, 0);
        m_cIOB.SetIO(6, 0);
    }
}

QVector<double> CTools::getLaserDistance()
{
    QVector<double> re(6);
    QVector<double> tmp(6); // 转换一下

    tmp[0] = m_cIOA.getAIState()[0];
    tmp[1] = m_cIOA.getAIState()[1];
    tmp[2] = m_cIOB.getAIState()[0];
    tmp[3] = m_cIOB.getAIState()[1];

    // log->debug("*******************LaserDistance*************: {} {} {} {} ",tmp[0],tmp[1],tmp[2],tmp[3]);

    // 视觉标定板上表面为激光零点
    std::vector<Eigen::Vector2f> cfg_laser = {
        //      k       b
        // Eigen::Vector2f(1950,1545),//0mm对应的数值，95mm对应的数值
        // Eigen::Vector2f(1957,1548),//2
        // Eigen::Vector2f(1960,1545),//3
        // Eigen::Vector2f(1950,1540),//4

        Eigen::Vector2f(1940, 1545), // 0mm对应的数值，80mm对应的数值
        Eigen::Vector2f(1942, 1548), // 2    // 15mm( 1965, 1968,)
        Eigen::Vector2f(1935, 1545), // 3
        Eigen::Vector2f(1945, 1540), // 4 1930
    };
    for (int i = 0; i < 4; ++i)
    {
        // 换算为真实距离，单位毫米
        tmp[i] = 80.0 / (cfg_laser[i].y() - cfg_laser[i].x()) * (tmp[i] - cfg_laser[i].x());
    }

    // 调整点激光顺序 1,x+，y-； 2x+，y+；3：X-,Y+; 4X-,Y-
    re[0] = tmp[1];
    re[1] = tmp[0] + 2;
    re[2] = tmp[2] + 2;
    re[3] = tmp[3];

    // log->debug("测量距离：{} {} {} {} ",re[0],re[1],re[2],re[3]);
    //  test
    //    re[0] = 14;
    //    re[1] = 15;
    //    re[2] = 15;
    //    re[3] = 15;
    return re;
}

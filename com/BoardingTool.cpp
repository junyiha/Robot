//
// Created by fjbfe on 2024/9/20.
//

#include "BoardingTool.h"

BoardingTool::BoardingTool()
{
    log = spdlog::get("logger");
}

BoardingTool::~BoardingTool()
{

}


void BoardingTool::SetLight(quint8 index, bool On)
{
    byte portvalue = m_cIOA.getDOState()[3];
    if(On == true) {
        switch (index) {
            case 0:
                portvalue |= 0b0000111;
                break;
            case 1:
                portvalue |= 0b0000001;
                break;
            case 2:
                portvalue |= 0b0000010;
                break;
            case 3:
                portvalue |= 0b0000100;
                break;
            default:
                log->warn("Invalid light index: {}", index);
        }
    }
    else
    {
        switch (index)
            {
                case 0:
                    portvalue &= ~0b0000111;
                    break;
                case 1:
                    portvalue &= ~0b0000001;
                    break;
                case 2:
                    portvalue &= ~0b0000010;
                    break;
                case 3:
                    portvalue &= ~0b0000100;
                    break;
                default:
                    log->warn("Invalid light index: {}", index);
            }

    }

    m_cIOA.SetIO(3,portvalue);
}


void BoardingTool::SetLaserMarker(bool On)
{
    byte portvalue = m_cIOA.getDOState()[3];
    if(On == true)
    {
        portvalue |= 0b01000000;
    }
    else
    {
        portvalue &= ~0b01000000;
    }
    m_cIOA.SetIO(3,portvalue);
}

void BoardingTool::SetCylinder(int push)
{
    byte portvalue = m_cIOA.getDOState()[3];
    switch (push)
    {
        case -1:
            portvalue |=  0b00100000;
            portvalue &= ~0b00010000;
            break;
        case 0:
            portvalue &= ~0b00110000;
            break;
        case 1:
            portvalue |=  0b00010000;
            portvalue &= ~0b00010000;
            break;
        default:
            log->warn("Invalid cylinder action :push = {}", push);
            break;

    }
    m_cIOA.SetIO(3,portvalue);


}



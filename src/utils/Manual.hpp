/**
 * @file Manual.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-01-21
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <QtSerialPort/QtSerialPort>

#include "utils/basic_header.hpp"

namespace Utils
{
    struct ManualData_t
    {
        // 急停指令
        bool StopCommand; // 界面停止
        bool HaltCommand; // 急停指令

        // 车运动指令
        double VechVel;    // 底盘行走: 前进/后退 23~24
        double RotateVel;  // 底盘行走: 差速转向 25~26
        bool bVechFlag;    // 底盘行走: 前进/后退标志位 int16 = 0? false: true
        bool bRotateFlag;  // 底盘行走: 差速转向标志位 int16 = 0? false: true
        double VechDirect; // 舵轮: 舵轮控制 9~10

        // 机械臂运动指令
        std::vector<double> LinkMove{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // 机器人关节角/末端位置 11~22
        bool bLinkMoveFlag;                                         // 机器人运动标志位 all int16 = 0 ? false:true
        bool bEndMove;                                              // 模式: 切换推杆模式 true--末端 | false--单轴 保开1 buffer[8] bit0,bit1

        // 设置指令[
        int Ready; // 0: 不动, 1: 举升, 2: 放下 buffer[8] bit2,bit3

        // 任务指令
        enum ETaskIndex // 任务: 执行任务
        {
            None = 0,
            SecondPush = 1,
            Quit = 2,
            Parallel = 4,
            SecondQuit = 8,
            Positioning = 16,
            Pause = 32,
            FitBoard = 64,
            Terminate = 128,
            MagentOff = 9, // 暂时保留，避免编译问题，后续删除
            DoWeld = 65,   // 暂时保留，避免编译问题，后续删除
            MagentOn = 3   // 暂时保留，避免编译问题，后续删除
        };

        int TaskIndex; // = buff[7]
    };

    class Manual : public QObject
    {
    public:
        Manual()
        {
            m_serial.setPortName(GP::Manual_Serial_Port.c_str());
            m_serial.setBaudRate(QSerialPort::Baud115200);
            m_serial.setDataBits(QSerialPort::Data8);
            m_serial.setParity(QSerialPort::NoParity);
            m_serial.setStopBits(QSerialPort::OneStop);
            m_serial.setFlowControl(QSerialPort::NoFlowControl);
            m_serial.open(QIODevice::ReadWrite);

            m_send_data.resize(7);
            m_send_data[0] = 0x07;
            m_send_data[1] = 0x00;
            m_send_data[2] = 0x0C;
            m_send_data[3] = 0x00;
            m_send_data[4] = 0x0A;
            m_send_data[5] = 0x01;
            m_send_data[6] = 0X1E;

            Run();
            Reconnection();
        }
        virtual ~Manual()
        {

        }

        bool Check() const
        {
            return m_serial.isOpen();
        }

        void GetData(ManualData_t& data)
        {
            std::lock_guard<std::mutex> locker(m_mutex);
            ////////////////////////////////--新协议--//////////////////////////////////////////////
            data.TaskIndex = static_cast<quint8>(m_recv_data[6]);
            data.bEndMove = static_cast<int>(static_cast<quint8>(m_recv_data[7]) & 0b0000'0011) == 2 ? true : false; // 修改1
            data.Ready = static_cast<int>(static_cast<quint8>(m_recv_data[7]) & 0b0000'1100) >> 2;                   // 修改2
            std::pair<double, bool> temp_val;
            temp_val = translateToVelocity(static_cast<quint8>(m_recv_data[22]), static_cast<quint8>(m_recv_data[23]));
            data.VechVel = temp_val.first;
            data.bVechFlag = temp_val.second;

            temp_val = translateToVelocity(static_cast<quint8>(m_recv_data[24]), static_cast<quint8>(m_recv_data[25]));
            data.RotateVel = temp_val.first;
            data.bRotateFlag = temp_val.second;

            data.VechDirect = (static_cast<quint8>(m_recv_data[8]) * 256 + static_cast<quint8>(m_recv_data[9]) - 2048) / 2048.0 * 90;

            auto temp_val_x = translateToVelocity(static_cast<quint8>(m_recv_data[10]), static_cast<quint8>(m_recv_data[11]));
            data.LinkMove.at(0) = temp_val_x.first;
            auto temp_val_y = translateToVelocity(static_cast<quint8>(m_recv_data[12]), static_cast<quint8>(m_recv_data[13]));
            data.LinkMove.at(1) = temp_val_y.first;
            auto temp_val_z = translateToVelocity(static_cast<quint8>(m_recv_data[14]), static_cast<quint8>(m_recv_data[15]));
            data.LinkMove.at(2) = temp_val_z.first;

            auto temp_val_rx =
                translateToVelocity(static_cast<quint8>(m_recv_data[16]), static_cast<quint8>(m_recv_data[17]));
            data.LinkMove.at(3) = temp_val_rx.first;
            auto temp_val_ry =
                translateToVelocity(static_cast<quint8>(m_recv_data[18]), static_cast<quint8>(m_recv_data[19]));
            data.LinkMove.at(4) = temp_val_ry.first;
            auto temp_val_rz =
                translateToVelocity(static_cast<quint8>(m_recv_data[20]), static_cast<quint8>(m_recv_data[21]));
            data.LinkMove.at(5) = temp_val_rz.first;

            if (temp_val_x.second || temp_val_y.second || temp_val_z.second ||
                temp_val_rx.second || temp_val_ry.second || temp_val_rz.second)
            {
                data.bLinkMoveFlag = true;
            }
            else
            {
                data.bLinkMoveFlag = false;
            }
        }

    private:
        void Run()
        {
            std::thread temp_thread = std::thread([this]() {
                while (true)
                {
                    if (!m_serial.isOpen())
                    {
                        SPDLOG_WARN("serial port is not open...");
                        QThread::msleep(1000);
                        continue;
                    }
                    m_serial.write(m_send_data);
                    if (!m_serial.waitForBytesWritten(1000))
                    {
                        SPDLOG_WARN("write timeout...");
                        continue;
                    }
                    QThread::msleep(30);
                    QByteArray recv_data = m_serial.read(32);
                    if (!CheckRecvData(recv_data))
                    {
                        SPDLOG_WARN("receive data is invalid...");
                        continue;
                    }
                    std::lock_guard<std::mutex> locker(m_mutex);
                    m_recv_data = recv_data;
                }
            });

            m_thread_pool.push_back(std::move(temp_thread));
        }

        bool CheckRecvData(QByteArray& data)
        {
            if (data.size() < 0x1D)
            {
                SPDLOG_WARN("Invalid data length, data is unused");
                return false;
            }

            uint8_t readbuff[100];
            bool is_Valid = false;
            for (int i = 0; i <= data.size() - 0x1D; i++)
            {
                if (static_cast<uchar>(data[i]) == 0x1D)
                {
                    std::copy(data.begin() + i, data.begin() + (i + 0x1D), readbuff);
                    is_Valid = true;
                    break;
                }
            }
            if (!is_Valid)
            {
                SPDLOG_WARN("Invalid data length, data is unused");
                return false;
            }

            int check_plus = 0;
            for (int i = 0; i < readbuff[0]; i++)
            {
                check_plus += readbuff[i];
            }
            if (check_plus == readbuff[0x1C]) // 校验
            {
                SPDLOG_WARN("Invalid data length, data is unused");
                return false;
            }

            return true;
        }

        std::pair<double, bool> translateToVelocity(quint8 high_byte, quint8 low_byte)
        {
            qint16 tmp = high_byte * 256 + low_byte - 2048;
            if (tmp > 200)
            {
                return std::make_pair((tmp - 200) / 1500.0, true);
            }
            else if (tmp < -200)
            {
                return std::make_pair((tmp + 200) / 1500.0, true);
            }
            else
            {
                return std::make_pair(0.0, false);
            }
        }

        void Reconnection()
        {
            std::thread temp_thread = std::thread([this]() {
                while (true)
                {
                    std::this_thread::sleep_for(std::chrono::seconds(1));

                    SPDLOG_INFO("Check serial port status");
                    if (!m_serial.isOpen())
                    {
                        m_serial.open(QIODevice::ReadWrite);
                    }
                }
            });

            m_thread_pool.push_back(std::move(temp_thread));
        }

    private:
        QSerialPort m_serial;
        QByteArray m_send_data;
        QByteArray m_recv_data;
        std::mutex m_mutex;
        std::vector<std::thread> m_thread_pool;
    };

}  // namespace Utils
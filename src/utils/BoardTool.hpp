/**
 * @file BoardTools.hpp
 * @author your name (you@domain.com)
 * @brief
 *   0000'0001 孔位相机开
 *   0000'0010 短边相机开
 *   0000'0100 长边相机开
 *   0000'0111 全部相机开
 *
 *   0001'0000 推杆前进
 *   0010'0000 推杆后退
 *
 *   0100'0000 标线激光开
 *
 *   0000'0000 关闭所有设备
 *
 * @version 0.1
 * @date 2025-02-11
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include "utils/basic_header.hpp"

#include <QtNetwork/QTcpSocket>

namespace Utils
{
    class BoardTool
    {
    public:
        BoardTool() : m_send_data(send_length, 0), m_recv_data(1024, 0)
        {
            m_send_data[0] = send_length;
            m_send_data[1] = 64;
            m_send_data[2] = 16;
            m_send_data[3] = 0;
            m_send_data[4] = 16;
            m_send_data[5] = 1;
        }
        virtual ~BoardTool()
        {
            m_exit_flag = true;
            m_thread.join();
        }

        void Connect()
        {
            QString ip = GP::IOA_IP.c_str();
            int port = GP::IOA_Port;

            m_socket.connectToHost(ip, port);

            if (m_socket.waitForConnected(5000))
            {
                qDebug() << "Successfully connected to the server!";
            }
            else
            {
                qDebug() << "Connection failed:" << m_socket.errorString();
            }
        }

        void StartLoop()
        {
            auto temp_func = [this]() {
                while (true)
                {
                    if (m_exit_flag)
                    {
                        break;
                    }
                    m_mutex.lock();
                    int sum_index = send_length - 1;
                    m_send_data[sum_index] = std::accumulate(m_send_data.begin(), m_send_data.begin() + sum_index, 0);
                    m_mutex.unlock();
                    qint64 send_len = m_socket.write(m_send_data);

                    std::this_thread::sleep_for(std::chrono::milliseconds(200));

                    if (m_socket.waitForBytesWritten(3000))
                    {
                        if (m_socket.waitForReadyRead(3000))
                        {
                            m_mutex.lock();
                            m_recv_data = m_socket.readAll();
                            m_mutex.unlock();
                        }
                        else
                        {
                            qDebug() << "No data received.";
                        }
                    }
                }
            };

            m_thread = std::thread(temp_func);
        }

        void Stop()
        {


        }

        void ControlLight(bool hole_light, bool short_light, bool long_light)
        {
            std::lock_guard<std::mutex> locker(m_mutex);
            std::bitset<8> value(m_recv_data[7]);

            hole_light ? value.set(0, 1) : value.set(0, 0);
            short_light ? value.set(1, 1) : value.set(1, 0);
            long_light ? value.set(2, 1) : value.set(2, 0);


            m_send_data[7] = value.to_ulong();
        }

        void ControlCylinder(int cmd)
        {
            std::lock_guard<std::mutex> locker(m_mutex);
            std::bitset<8> value(m_recv_data[7]);

            switch (cmd)
            {
            case 1:
            {
                value.set(4, 1);
                break;
            }
            case 0:
            {
                value.set(4, 0);
                value.set(5, 0);
                break;
            }
            case -1:
            {
                value.set(5, 1);
                break;
            }
            }


            m_send_data[7] = value.to_ulong();
        }

        void ControlMarkLaser(bool flag)
        {
            std::lock_guard<std::mutex> locker(m_mutex);
            std::bitset<8> value(m_recv_data[7]);

            flag ? value.set(6, 1) : value.set(6, 0);


            m_send_data[7] = value.to_ulong();
        }

    private:
        QTcpSocket m_socket;
        std::mutex m_mutex;
        std::thread m_thread;
        int send_length{ 0x0f };
        QByteArray m_send_data;
        QByteArray m_recv_data;
        bool m_exit_flag{ false };
    };
}  // namespace Utils
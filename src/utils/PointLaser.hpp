/**
 * @file PointLaser.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-01-22
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <thread>
#include <mutex>

#include <QtSerialPort/QtSerialPort>

#include "utils/basic_header.hpp"

namespace Utils
{
    class PointLaser : public QObject
    {
    public:
        PointLaser()
        {
            m_serial.setPortName(GP::Tool_Serial_Port.c_str());
            m_serial.setBaudRate(QSerialPort::Baud115200);
            m_serial.setDataBits(QSerialPort::Data8);
            m_serial.setParity(QSerialPort::NoParity);
            m_serial.setStopBits(QSerialPort::OneStop);
            m_serial.setFlowControl(QSerialPort::NoFlowControl);
            m_serial.open(QIODevice::ReadWrite);

            InitSendData();
            Run();
            Reconnection();
        }
        virtual ~PointLaser()
        {

        }

        bool Check() const
        {
            return m_serial.isOpen();
        }

        std::vector<double> GetData()
        {
            std::lock_guard<std::mutex> locker(m_mutex);
            std::vector<double> data;
            for (auto& item : m_recv_data)
            {
                data.push_back(item.toDouble());
            }

            return data;
        }

    private:
        void InitSendData()
        {
            for (int i = 0; i < 4; i++)
            {
                QByteArray data(8, 0);
                data[1] = 0x04;
                data[2] = 0x00;
                data[3] = 0x00;
                data[4] = 0x00;
                data[5] = 0x02;
                switch (i)
                {
                case 0:
                    data[0] = 0x01;
                    data[6] = 0x71;
                    data[7] = 0xCB;
                    break;
                case 1:
                    data[0] = 0x02;
                    data[6] = 0x71;
                    data[7] = 0xF8;
                    break;
                case 2:
                    data[0] = 0x03;
                    data[6] = 0x70;
                    data[7] = 0x29;
                    break;
                case 3:
                    data[0] = 0x04;
                    data[6] = 0x71;
                    data[7] = 0x9E;
                    break;
                }
                m_send_data.push_back(data);
            }
        }

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
                    QVector<QByteArray> temp_recv_data;
                    for (auto& data : m_send_data)
                    {
                        m_serial.write(data);
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
                        temp_recv_data.push_back(recv_data);
                    }
                    std::lock_guard<std::mutex> locker(m_mutex);
                    m_recv_data = temp_recv_data;
                }
            });

            m_thread_pool.push_back(std::move(temp_thread));
        }

        bool CheckRecvData(QByteArray& data)
        {
            int result{ 0 };
            if (data.size() < 0x08)
            {
                SPDLOG_WARN("Invalid data length, data is unused");
                return false;
            }

            if ((static_cast<int>(data[0]) == 0x01 | static_cast<int>(data[0]) == 0x02 | static_cast<int>(data[0]) == 0x03 | static_cast<int>(data[0]) == 0x04) && static_cast<int>(data[1]) == 0x04)
            { // 校验数据帧前两位
                result = (static_cast<uint32_t>(data[3]) << 24) |
                    (static_cast<uint32_t>(data[4]) << 16) |
                    (static_cast<uint32_t>(data[5]) << 8) |
                    static_cast<uint32_t>(data[6]);
            }

            result == -1 ? data = QByteArray::number(-1.0) : data = QByteArray::number(result / 1000.0);

            return true;
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
        QVector<QByteArray> m_send_data;
        QVector<QByteArray> m_recv_data;
        std::mutex m_mutex;
        std::vector<std::thread> m_thread_pool;
    };

}  // namespace Utils
/**
 * @file Serial.hpp
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
#include <QMap>

namespace Utils
{
    class Serial
    {
    public:
        bool Open(const QString port)
        {
            m_serial.setPortName(port);
            m_serial.setBaudRate(QSerialPort::Baud115200);
            m_serial.setDataBits(QSerialPort::Data8);
            m_serial.setParity(QSerialPort::NoParity);
            m_serial.setStopBits(QSerialPort::OneStop);
            m_serial.setFlowControl(QSerialPort::NoFlowControl);

            return m_serial.open(QIODevice::ReadWrite);
        }

        void Loop()
        {
            QByteArray buf(7, 0);

            buf[0] = 0x07;
            buf[1] = 0x00;
            buf[2] = 0x0C;
            buf[3] = 0x00;
            buf[4] = 0x0A;
            buf[5] = 0x01;
            buf[6] = 0X1E;

            while (true)
            {
                qint64 bytesWritten = m_serial.write(buf);
                if (!m_serial.waitForBytesWritten(1000))  // 默认为异步，此处目的：改为同步，等待写入完毕，再读数据
                {
                    qDebug() << "write timeout...";
                    break;
                }
                QThread::msleep(30);
                QByteArray recv_data = m_serial.read(32);
                qDebug() << "send data: " << buf.size() << "\n"
                    << "receive data: " << recv_data.size() << "\n";
            }
        }

    private:
        QSerialPort m_serial;
    };
}  // namespace Utils
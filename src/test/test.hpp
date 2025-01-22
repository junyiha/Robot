/**
 * @file test.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-12-18
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#ifndef TEST_HPP__
#define TEST_HPP__

#include <clocale>
#include <string>
extern "C"
{
#include <conio.h>
}

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

#include <Eigen/Eigen>

#include <QApplication>
#include <QThread>
#include <QTextCodec>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlError>
#include <QtSql/QSqlQuery>
#include <QStringList>

#include <QJsonObject>
#include <QJsonDocument>

#include <QtSerialPort>

#include <QTcpSocket>

#include "cxxopts.hpp"

#include "hmi/SceneSelectionWindow.hpp"
#include "vision/VisionInterface.h"
#include "com/LaserDistanceBojke.h"
#include "net/tcp_common.hpp"
#include "utils/Sql.hpp"
#include "utils/Serial.hpp"
#include "utils/Manual.hpp"
#include "utils/PointLaser.hpp"

#include "MvCamera.h"

int line_detect_demo(int argc, char* argv[]);

int laserDemo(int argc, char* argv[]);

int TestCom(int argc, char* argv[]);

int TestTask(int argc, char* argv[]);

int TestInitLog(int argc, char* argv[]);

int TestConfigManager(int argc, char* argv[]);

int TestTcpClient(int argc, char* argv[]);

int TestFitBoard(int argc, char* argv[]);

int TestRobot(int argc, char* argv[]);

int TestQtSQL(int argc, char* argv[]);

int TestQtJSON(int argc, char* argv[]);

int TestQtSerialPort(int argc, char* argv[]);

int TestComSerialCom(int argc, char* argv[]);

int TestQTcpSocket(int argc, char* argv[]);

int TestBoardingTool(int argc, char* argv[]);

int TestHKCamera(int argc, char* argv[]);

int TestManual(int argc, char* argv[]);

int TestPointLaser(int argc, char* argv[]);

#endif  // TEST_HPP__
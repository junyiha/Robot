/**
 * @file Sql.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-01-13
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef SQL_HPP
#define SQL_HPP
#pragma once

#include <exception>

#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlError>
#include "QtSql/QSqlQuery"
#include <QStringList>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

namespace UTILS
{
    class Sql
    {
    public:
        Sql() = delete;
        Sql(const QString path)
        {
            m_db = QSqlDatabase::addDatabase("QSQLITE");
            m_db.setDatabaseName(path);
            if (!m_db.open())
            {
                SPDLOG_ERROR("Failed to open database: {}", m_db.lastError().text().toStdString());
                throw std::exception("invalid sql");
            }
            m_query = new QSqlQuery(m_db);
        }

        virtual ~Sql()
        {
            m_db.close();
            delete m_query;
        }

        bool CreateTable(const QString data)
        {
            bool res = m_query->exec(data);
            if (!res)
            {
                SPDLOG_ERROR("Failed to create table: {}", m_db.lastError().text().toStdString());
            }

            return res;
        }

        bool InsertData(const QString data)
        {
            bool res = m_query->exec(data);
            if (!res)
            {
                SPDLOG_ERROR("Failed to insert data: {}", m_db.lastError().text().toStdString());
            }

            return res;
        }

        bool InsertData(QSqlQuery& query)
        {
            bool res = query.exec();
            if (!res)
            {
                SPDLOG_ERROR("Failed to insert data: {}", m_db.lastError().text().toStdString());
            }

            return res;
        }

    private:
        QSqlDatabase m_db;
        QSqlQuery* m_query;
    };
}  // namespace UTILS

#endif  // SQL_HPP
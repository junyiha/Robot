/**
 * @file TcpClient.hpp
 * @author your name (you@domain.com)
 * @brief 重构Tcp客户端部分
 * @version 0.1
 * @date 2024-12-11
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#ifndef TCP_CLIENT_HPP
#define TCP_CLIENT_HPP

 // 解决windows下编译告警问题
#ifdef _WIN32
#define _WIN32_WINNT 0x0601
#endif

#include <string>
#include <iostream>
#include <functional>

#define BOOST_ASIO_NO_DEPRECATED
#include "boost/asio.hpp"

namespace Network
{
    using tcp = boost::asio::ip::tcp;

    class TcpClient
    {
    public:
        TcpClient();
        virtual ~TcpClient();

    public:
        bool ConnectToServer(std::string ip, std::size_t port);
        void Disconnect();

        void SendData();
        std::size_t RecvData(std::vector<char>& buffer);


    private:
        void GrabSomeData(const boost::system::error_code& ec);

    public:
        boost::system::error_code error_code;

    private:
        boost::asio::io_context m_io_context;
        tcp::socket m_socket;
    };
}

#endif  // TCP_CLIENT_HPP
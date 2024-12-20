/**
 * @file tcp_common.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-12-20
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#ifndef TCP_COMMON_HPP
#define TCP_COMMON_HPP

#include "asio.hpp"
#include "utils/basic_header.hpp"

namespace net
{
    using tcp = asio::ip::tcp;

    class TcpClient
    {
    public:
        TcpClient(asio::io_context& io_context, const tcp::endpoint& endpoint);

        void Write(const std::vector<char>& data);

        std::vector<char> Read();

        bool IsConnect() { return is_connect.load(); }

    private:
        void do_connect();

    private:
        asio::io_context& io_context_;
        tcp::socket socket_;
        std::atomic<bool> is_connect{ false };
        tcp::endpoint endpoint_;
        std::shared_ptr<spdlog::logger> logger;
    };
} // namespace net


#endif
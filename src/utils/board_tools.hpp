/**
 * @file board_tools.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-12-20
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#ifndef BOARD_TOOLS_CPP
#define BOARD_TOOLS_CPP

#include "net/tcp_common.hpp"

namespace Tools
{
    class BoardTools
    {
    public:
        BoardTools();
        ~BoardTools();

        void Connect(const std::string& ip, const std::size_t port);
        bool IsConnect() { return tcp_client_ptr->IsConnect(); }

    private:
        std::unique_ptr<net::TcpClient> tcp_client_ptr;
        asio::io_context io_context_;
    };


}  // namespace Tools


#endif // BOARD_TOOLS_CPP
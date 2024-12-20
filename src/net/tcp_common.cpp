#include "tcp_common.hpp"

namespace net
{
    TcpClient::TcpClient(asio::io_context& io_context, const tcp::endpoint& endpoint)
        :io_context_(io_context), socket_(io_context), endpoint_(endpoint)
    {
        logger = spdlog::get("logger");
        do_connect();
    }

    void TcpClient::do_connect()
    {
        socket_.async_connect(endpoint_, [this](std::error_code ec) {
            if (!ec)
            {
                logger->info("connect succeeded\n");
                is_connect.store(true);
            }
            else
            {
                logger->info("connect failed\n");
                is_connect.store(false);
                do_connect();
            }
        });
    }

    void TcpClient::Write(const std::vector<char>& data)
    {
        if (!is_connect.load())
        {
            logger->info("fatal error: not connect\n");
            do_connect();
            return;
        }
        try
        {
            socket_.write_some(asio::buffer(data.data(), data.size()));
        }
        catch (asio::system_error& e)
        {
            logger->info("catch exception, message: {}", e.what());
            is_connect.store(false);
            socket_.close();
        }
    }

    std::vector<char> TcpClient::Read()
    {
        std::vector<char> recv_data(10 * 1024);
        if (!is_connect.load())
        {
            logger->info("fatal error: not connect\n");
            do_connect();
            return recv_data;
        }
        try
        {
            std::size_t len = socket_.read_some(asio::buffer(recv_data.data(), recv_data.size()));
            recv_data.resize(len);
        }
        catch (asio::system_error& e)
        {
            logger->info("catch exception, message: {}", e.what());
            is_connect.store(false);
            socket_.close();
        }

        return recv_data;
    }
} // namespace net
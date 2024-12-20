#include "utils/board_tools.hpp"

namespace Tools
{
    BoardTools::BoardTools()
    {
    }

    BoardTools::~BoardTools()
    {
    }


    void BoardTools::Connect(const std::string& ip, const std::size_t port)
    {
        tcp::endpoint endpoint(asio::ip::make_address(ip), port);
        tcp_client_ptr = std::make_unique<net::TcpClient>(io_context_, endpoint);
    }
}  // namespace Tools
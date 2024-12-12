#include "TcpClient.hpp"

namespace Network
{
    TcpClient::TcpClient()
        :m_socket(m_io_context)
    {

    }

    TcpClient::~TcpClient()
    {

    }

    bool TcpClient::ConnectToServer(std::string ip, std::size_t port)
    {
        tcp::endpoint endpoint(boost::asio::ip::make_address_v4(ip), port);
        m_socket.connect(endpoint, error_code);

        return m_socket.is_open();
    }

    void TcpClient::Disconnect()
    {
        m_socket.close();
    }

    void TcpClient::SendData()
    {

    }

    std::size_t TcpClient::RecvData(std::vector<char>& buffer)
    {
        return m_socket.read_some(boost::asio::buffer(buffer.data(), buffer.size()), error_code);
    }

    void TcpClient::GrabSomeData(const boost::system::error_code& ec)
    {
        error_code = ec;
    }
}
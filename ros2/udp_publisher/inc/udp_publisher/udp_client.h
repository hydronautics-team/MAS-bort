#ifndef UDP_CLIENT
#define UDP_CLIENT

#include <thread>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#define IPADDRESS "127.0.0.1"
#define UDP_PORT 5005


namespace udp_client {

using boost::asio::ip::udp;
using boost::asio::ip::address;

typedef std::function<void(std::string)> FunctionPtr;

class UdpClient {
public:
    UdpClient(FunctionPtr callback, rclcpp::Logger logger): callback_(callback), logger_(logger) {
    }

    void run() {
        socket_.open(udp::v4());
        socket_.bind(udp::endpoint(address::from_string(IPADDRESS), UDP_PORT));

        receive_next();

        RCLCPP_INFO(logger_, "Receiving");
        io_service_.run();
        RCLCPP_INFO(logger_, "Receiving exit");
    }

    void send(const std::string &data) {
        if (!socket_.is_open()) {
            RCLCPP_ERROR(logger_, "Can't send - socket is not open!");
        }
        socket_.async_send_to(boost::asio::buffer(data), remote_endpoint_,
            std::bind(&UdpClient::handle_send, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {
        if (error) {
            RCLCPP_ERROR_STREAM(logger_, "Receive failed: " << error.message());
            return;
        }

        std::string recv_str(recv_buffer_.begin(), recv_buffer_.begin() + bytes_transferred);
        RCLCPP_INFO_STREAM(logger_, "Received `" << recv_str  << "` (" << error.message() << ")");
        callback_(recv_str);

        receive_next();
    }

    void receive_next() {
        socket_.async_receive_from(boost::asio::buffer(recv_buffer_),
            remote_endpoint_,
            std::bind(&UdpClient::handle_receive, this, std::placeholders::_1, std::placeholders::_2));
    }

    void handle_send(const boost::system::error_code& error, std::size_t bytes_transferred) {
        if(error) {
            RCLCPP_ERROR_STREAM(logger_, "Send failed: " << error.message());
            return;
        }
         RCLCPP_INFO_STREAM(logger_, "Send `" << bytes_transferred  << " bytes!");
    }

private:
    FunctionPtr callback_;
    rclcpp::Logger logger_;
    boost::asio::io_service io_service_;
    udp::socket socket_{io_service_};
    boost::array<char, 1024> send_buffer_;
    boost::array<char, 1024> recv_buffer_;
    udp::endpoint remote_endpoint_;
};

}  // namespace UdpClient

#endif  // UDP_CLIENT
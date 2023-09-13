#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "udp_client.h"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("udp_message", 1);
    udp_client_ = std::make_shared<UdpClient::UdpClient>(
      [&](std::string recv_msg){ this->udp_recv_callback(recv_msg); },
      this->get_logger()
    );
    std::thread([&](){ udp_client_->receisver_task(); }).detach();
  } 

  void udp_recv_callback(std::string recv_msg) {
    auto message = std_msgs::msg::String();
    message.data = recv_msg;
    publisher_->publish(message);
  }

private:
  std::shared_ptr<UdpClient::UdpClient> udp_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

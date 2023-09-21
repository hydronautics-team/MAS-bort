#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "udp_client.h"


using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
    static constexpr int TIMER_RATE_MSEC = 100;

    MinimalPublisher()
    : Node("minimal_publisher")
    , publisher_(this->create_publisher<std_msgs::msg::String>("udp_message", 1))
    , subscription_(this->create_subscription<std_msgs::msg::String>("planner_message", 1,
        std::bind(&MinimalPublisher::planner_msg_callback, this, std::placeholders::_1)))
    , udp_client_(std::bind(&MinimalPublisher::recv_callback, this, std::placeholders::_1),
        this->get_logger()) {
        std::thread([&](){ udp_client_.run(); }).detach();
        send_timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_RATE_MSEC),
            std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void planner_msg_callback(const std_msgs::msg::String &message) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Received from planner: " << message.data);
        last_planner_message_ = message.data;
    }

    void recv_callback(std::string recv_msg) {
        // Распарсить сообщение на разные данные и запостить в топики нужно будет где-то здесь

        auto message = std_msgs::msg::String();
        message.data = recv_msg;
        publisher_->publish(message);
    }

    void timer_callback() {
        // assert(!last_planner_message_.empty());
        udp_client_.send(last_planner_message_);
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    udp_client::UdpClient udp_client_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    std::string last_planner_message_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}

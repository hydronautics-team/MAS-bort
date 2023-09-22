#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class Control : public rclcpp::Node {
public:
    static constexpr int TIMER_RATE_MSEC = 100;

    Control() : Node("control")
    , publisher_(this->create_publisher<std_msgs::msg::String>("planner_message", 1))
    {
        send_timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_RATE_MSEC),
            std::bind(&Control::timer_callback, this));
    }

    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "control_data";
        publisher_->publish(message);
    }

private:
    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Control>());
    rclcpp::shutdown();
    return 0;
}


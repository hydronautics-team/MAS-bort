#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "udp_publisher/udp_client.h"
#include "udp_publisher/protocols.h"

//#include <udp_publisher/msg/from_bort.hpp>
#include "udp_publisher/msg/from_bort.hpp"
 

class MinimalPublisher : public rclcpp::Node {
public:
    static constexpr int TIMER_RATE_MSEC = 100;

    MinimalPublisher()
    : Node("minimal_publisher")
    , publisher_(this->create_publisher<udp_publisher::msg::FromBort>("udp_message", 1))
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
        FromBort* rec = (FromBort*)recv_msg.data();
        auto message = udp_publisher::msg::FromBort();
    
        message.sender_id = rec->headerSwap.senderID;
        message.receiver_id = rec->headerSwap.receiverID;
        message.msg_size = rec->headerSwap.msgSize;

        message.mode_real = rec->auvData.modeReal;

        message.yaw_contour = rec->auvData.controlReal.yaw;
        message.pitch_contour = rec->auvData.controlReal.pitch;
        message.roll_contour = rec->auvData.controlReal.roll;
        message.march_contour = rec->auvData.controlReal.march;
        message.depth_contour = rec->auvData.controlReal.depth;
        message.lag_contour = rec->auvData.controlReal.lag;

        message.mode_auv_real = rec->auvData.modeAUV_Real;

        message.yaw = rec->auvData.ControlDataReal.yaw;
        message.pitch = rec->auvData.ControlDataReal.pitch;
        message.roll = rec->auvData.ControlDataReal.roll;
        message.march = rec->auvData.ControlDataReal.march;
        message.depth = rec->auvData.ControlDataReal.depth;
        message.lag = rec->auvData.ControlDataReal.lag;

        message.vma1 = rec->auvData.signalVMA_real.VMA1;
        message.vma2 = rec->auvData.signalVMA_real.VMA2;
        message.vma3 = rec->auvData.signalVMA_real.VMA3;
        message.vma4 = rec->auvData.signalVMA_real.VMA4;
        message.vma5 = rec->auvData.signalVMA_real.VMA5;
        message.vma6 = rec->auvData.signalVMA_real.VMA6;

        message.yaw_imu = rec->dataAH127C.yaw;
        message.pitch_imu = rec->dataAH127C.pitch;
        message.roll_imu = rec->dataAH127C.roll;
        message.x_accel_imu = rec->dataAH127C.X_accel;
        message.y_accel_imu = rec->dataAH127C.Y_accel;
        message.z_accel_imu = rec->dataAH127C.Z_accel;
        message.x_rate_imu = rec->dataAH127C.X_rate; 
        message.y_rate_imu = rec->dataAH127C.Y_rate;
        message.z_rate_imu = rec->dataAH127C.Z_rate;
        message.x_magn_imu = rec->dataAH127C.X_magn; 
        message.y_magn_imu = rec->dataAH127C.Y_magn;
        message.z_magn_imu = rec->dataAH127C.Z_magn;
        message.quat[4] = rec->dataAH127C.quat[4];

        message.temperature = rec->dataPressure.temperature;
        message.depth_sensor = rec->dataPressure.depth;
        message.pressure = rec->dataPressure.pressure;

        message.location_x = rec->dataUWB.locationX;
        message.location_y =  rec->dataUWB.locationY;
        message.distance_to_beacon[4] =  rec->dataUWB.distanceToBeacon[4];
        message.distance_to_agent[10] =  rec->dataUWB.distanceToAgent[10];

        message.start_calibration = rec->flagAH127C_bort.startCalibration;
        message.end_calibration = rec->flagAH127C_bort.endCalibration;

        message.checksum = rec->checksum;

        publisher_->publish(message);
    }

    void timer_callback() {
        // assert(!last_planner_message_.empty());
        udp_client_.send(last_planner_message_);
    }

private:
    rclcpp::Publisher<udp_publisher::msg::FromBort>::SharedPtr publisher_;
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

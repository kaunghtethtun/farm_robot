// serial_interface_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <serial/serial.h>

class SerialInterface : public rclcpp::Node {
public:
    SerialInterface() : Node("serial_interface_node") {
        encoder_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("encoder_ticks", 10);
        motor_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "motor_command", 10,
            std::bind(&SerialInterface::motorCallback, this, std::placeholders::_1));
        
        // Serial port
        serial_.setPort("/dev/ttyACM0");
        serial_.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(to);
        serial_.open();

        timer_ = this->create_wall_timer(20ms, std::bind(&SerialInterface::readSerial, this));
    }

private:
    void readSerial() {
        if (serial_.available()) {
            std::string line = serial_.readline();
            std::istringstream ss(line);
            std::string tag;
            int left, right;
            ss >> tag >> left >> right;
            if (tag == "ENC") {
                std_msgs::msg::Int32MultiArray msg;
                msg.data = {left, right};
                encoder_pub_->publish(msg);
            }
        }
    }

    void motorCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double left = msg->linear.x;   // wheel velocity
        double right = msg->angular.z; // wheel velocity
        std::stringstream ss;
        ss << "CMD " << left << " " << right << "\n";
        serial_.write(ss.str());
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motor_sub_;
    serial::Serial serial_;
    rclcpp::TimerBase::SharedPtr timer_;
};

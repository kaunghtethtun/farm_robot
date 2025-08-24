#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

class MotorInterfaceNode : public rclcpp::Node
{
public:
    MotorInterfaceNode()
    : Node("motor_interface_node")
    {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);

        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();

        // Publishers
        encoder_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("encoder_counts", 10);
        rpm_pub_     = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_rpm", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Subscriber for cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "diff_controller/cmd_vel_unstamped", 10,
            std::bind(&MotorInterfaceNode::cmdVelCallback, this, std::placeholders::_1)
        );

        joint_names_ = {"left_wheel_joint", "right_wheel_joint"};
        wheel_diameter_ = 0.08; // meters

        openSerial();

        // Timer for reading serial data frequently
        serial_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz serial read
            std::bind(&MotorInterfaceNode::readSerial, this)
        );

        // Timer to publish all topics at exact 20 Hz
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            std::bind(&MotorInterfaceNode::publishAllTopics, this)
        );

        RCLCPP_INFO(this->get_logger(), "Motor Interface Node started on %s @ %d baud", port_.c_str(), baudrate_);
    }

    ~MotorInterfaceNode()
    {
        if (serial_fd_ >= 0) close(serial_fd_);
    }

private:
    std::string port_;
    int baudrate_;
    int serial_fd_ = -1;
    std::string serial_buffer_;
    std::mutex data_mutex_; // Protect shared encoder/RPM data

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rpm_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr serial_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::vector<std::string> joint_names_;
    double wheel_diameter_;

    // Latest encoder & velocity values
    int32_t enc_left_ = 0;
    int32_t enc_right_ = 0;
    float rpm_left_ = 0.0;
    float rpm_right_ = 0.0;

    double left_encoder_rad_ = 0.0;
    double right_encoder_rad_ = 0.0;
    double left_velocity_rad_ = 0.0;
    double right_velocity_rad_ = 0.0;

    void openSerial()
    {
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }

        struct termios tty{};
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_FATAL(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
            close(serial_fd_);
            rclcpp::shutdown();
            return;
        }

        cfsetospeed(&tty, baudrateToFlag(baudrate_));
        cfsetispeed(&tty, baudrateToFlag(baudrate_));
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;

        tcflush(serial_fd_, TCIFLUSH);
        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_FATAL(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
            close(serial_fd_);
            rclcpp::shutdown();
        }
    }

    speed_t baudrateToFlag(int baudrate)
    {
        switch (baudrate) {
            case 115200: return B115200;
            case 57600:  return B57600;
            case 38400:  return B38400;
            case 19200:  return B19200;
            case 9600:   return B9600;
            default:
                RCLCPP_WARN(this->get_logger(), "Unsupported baudrate %d, defaulting to 115200", baudrate);
                return B115200;
        }
    }

    void readSerial()
    {
        char buffer[256];
        int n = ::read(serial_fd_, buffer, sizeof(buffer) - 1);
        if (n > 0) {
            buffer[n] = '\0';
            serial_buffer_ += buffer;

            size_t newline_pos;
            while ((newline_pos = serial_buffer_.find('\n')) != std::string::npos) {
                std::string line = serial_buffer_.substr(0, newline_pos);
                serial_buffer_.erase(0, newline_pos + 1);

                if (!line.empty() && line.back() == '\r') line.pop_back();

                parseLine(line);
            }
        }
    }

    void parseLine(const std::string &line)
    {
        // Expected format: 3,<enc_left>,<enc_right>,<rpm_left>,<rpm_right>
        if (line.rfind("3,", 0) != 0) return;

        std::istringstream ss(line);
        std::string token;
        std::vector<double> values;
        while (std::getline(ss, token, ',')) {
            try { values.push_back(std::stod(token)); } catch (...) { return; }
        }
        if (values.size() != 5) return;

        std::lock_guard<std::mutex> lock(data_mutex_);
        enc_left_ = static_cast<int32_t>(values[2]);
        enc_right_ = static_cast<int32_t>(values[1]);
        rpm_left_ = static_cast<float>(values[4]);
        rpm_right_ = static_cast<float>(values[3]);
	
	RCLCPP_INFO(this->get_logger(), "Right Encoder Count (%d), Left Encoder Count (%d)",enc_right_,enc_left_);
                
        // Convert to radians
        double counts_per_rev = 1320;
        left_encoder_rad_ = enc_left_ * 2.0 * M_PI / counts_per_rev;
        right_encoder_rad_ = enc_right_ * 2.0 * M_PI / counts_per_rev;
        left_velocity_rad_ = rpm_left_ * 2.0 * M_PI / 60.0;
        right_velocity_rad_ = rpm_right_ * 2.0 * M_PI / 60.0;
    }

    void publishAllTopics()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Encoder counts
        std_msgs::msg::Int32MultiArray enc_msg;
        enc_msg.data = {enc_left_, enc_right_};
        encoder_pub_->publish(enc_msg);

        // RPM
        std_msgs::msg::Float32MultiArray rpm_msg;
        rpm_msg.data = {rpm_left_, rpm_right_};
        rpm_pub_->publish(rpm_msg);

        // Joint states
        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.name = joint_names_;
        joint_msg.position = {left_encoder_rad_, right_encoder_rad_};
        joint_msg.velocity = {left_velocity_rad_, right_velocity_rad_};
        joint_state_pub_->publish(joint_msg);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double x = msg->linear.x;
        double theta = msg->angular.z;

        double coefficient = (x == 0.0) ? 4.0 : 2.1;
        double track_width = 0.2; // meters

        double right = x + (theta * track_width / 2);
        double left  = x - (theta * track_width / 2);

        double rpm_right = right * 60 / (M_PI * wheel_diameter_) *coefficient;
        double rpm_left  = left  * 60 / (M_PI * wheel_diameter_) *coefficient;

        rpm_right = std::clamp(rpm_right, -40.0, 40.0);
        rpm_left  = std::clamp(rpm_left, -40.0, 40.0);

        std::ostringstream ss;
        ss << "1," << rpm_right << "," << rpm_left << ";\n";
        std::string out = ss.str();
        ::write(serial_fd_, out.c_str(), out.size());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}


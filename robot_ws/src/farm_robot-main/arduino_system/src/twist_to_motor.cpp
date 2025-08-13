#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sstream>
#include <vector>

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

        // Subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "diff_controller/cmd_vel_unstamped", 10,
            std::bind(&MotorInterfaceNode::cmdVelCallback, this, std::placeholders::_1)
        );

        // Open serial
        openSerial();

        // Timer to read serial data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MotorInterfaceNode::readSerial, this)
        );

        RCLCPP_INFO(this->get_logger(), "Motor Interface Node started on %s @ %d baud", port_.c_str(), baudrate_);
    }

    ~MotorInterfaceNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    std::string port_;
    int baudrate_;
    int serial_fd_;
    std::string serial_buffer_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rpm_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

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

                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }

                parseLine(line);
            }
        } else if (n < 0 && errno != EAGAIN) {
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", strerror(errno));
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
            try {
                values.push_back(std::stod(token));
            } catch (...) {
                return;
            }
        }

        if (values.size() != 5) return;
        int32_t enc_left = static_cast<int32_t>(values[2]);
        int32_t enc_right = static_cast<int32_t>(values[1]);
        float rpm_left = static_cast<float>(values[4]);
        float rpm_right = static_cast<float>(values[3]);

        // Publish encoder counts
        std_msgs::msg::Int32MultiArray enc_msg;
        enc_msg.data = {enc_left, enc_right};
        encoder_pub_->publish(enc_msg);

        // Publish RPM
        std_msgs::msg::Float32MultiArray rpm_msg;
        rpm_msg.data = {rpm_left, rpm_right};
        rpm_pub_->publish(rpm_msg);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double x = static_cast<double>(msg->linear.x);     // m/s
        double theta = static_cast<double>(msg->angular.z); // rad/s
        
        double coefficient = 2.1;
        if (x == 0.0) {
            coefficient = 4.0;
        }
        
        double wheel_diameter = 0.08; // meters
        double track_width = 0.2;    // meters
        double pi = 3.14159265358979323846;

        // Convert linear and angular velocities to wheel velocities
        double right = 1.0 * x + (theta * track_width / 2);
        double left  = 1.0 * x - (theta * track_width / 2);

        // ms to rpm
        double rpm_right = right *60 / (pi * wheel_diameter);
        double rpm_left  = left  *60 / (pi * wheel_diameter);

	rpm_right *= coefficient;
	rpm_left *= coefficient;
	
	if (rpm_right > 60) { rpm_right = 60;}
	else if (rpm_right < -60) {rpm_right = -60;}
	
	if (rpm_left > 60) { rpm_left = 60;}
	else if (rpm_left < -60) {rpm_left = -60;}
        
        std::ostringstream ss;
        ss << "1" << "," << rpm_right << "," << rpm_left << ";" << "\n";
        std::string out = ss.str();
        ::write(serial_fd_, out.c_str(), out.size());

        RCLCPP_INFO(this->get_logger(), "Published: Rvel=%f, Lvel=%f", rpm_left, rpm_right);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sstream>

class SerialNode : public rclcpp::Node
{
public:
  SerialNode() : Node("serial_node")
  {
    encoder_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("encoder_ticks", 10);
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "motor_command", 10,
      std::bind(&SerialNode::cmdCallback, this, std::placeholders::_1)
    );

    serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_fd_, &tty) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
      close(serial_fd_);
      rclcpp::shutdown();
      return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
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
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
      close(serial_fd_);
      rclcpp::shutdown();
      return;
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&SerialNode::readSerial, this)
    );
  }

  ~SerialNode()
  {
    if (serial_fd_ >= 0)
    {
      close(serial_fd_);
    }
  }

private:
  int serial_fd_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::string serial_buffer_;

  void readSerial()
  {
    char buffer[256];
    int n = ::read(serial_fd_, buffer, sizeof(buffer) - 1);
    if (n > 0)
    {
      buffer[n] = '\0';
      serial_buffer_ += buffer;

      size_t newline_pos;
      while ((newline_pos = serial_buffer_.find('\n')) != std::string::npos)
      {
        std::string line = serial_buffer_.substr(0, newline_pos);
        serial_buffer_.erase(0, newline_pos + 1);

        // Trim carriage return if present
        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }

        if (line.rfind("3,", 0) == 0)
        {
          std::istringstream ss(line);
          std::string token;
          std::vector<int> values;

          while (std::getline(ss, token, ','))
          {
            try {
              values.push_back(std::stoi(token));
            } catch (...) {
              RCLCPP_WARN(this->get_logger(), "Invalid token: '%s'", token.c_str());
            }
          }

          if (values.size() == 5)
          {
            std_msgs::msg::Int32MultiArray msg;
            msg.data = {values[1], values[2], values[3], values[4]};
            encoder_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published: Renc=%d, Lenc=%d, Rvel=%d, Lvel=%d",
                        values[1], values[2], values[3], values[4]);
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "Invalid number of values in line: %s", line.c_str());
          }
        }
      }
    }
    else if (n < 0 && errno != EAGAIN)
    {
      RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", strerror(errno));
    }
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double v_left = static_cast<double>(msg->linear.x);
    double v_right = static_cast<double>(msg->angular.z);
    double rpm_left = (v_left / (2 * M_PI * 0.03)) * 60.0;
    double rpm_right = (v_right / (2 * M_PI * 0.03)) * 60.0;
    //double rpm_left = 100*v_left;
    //double rpm_right = 100*v_right;
    if(rpm_left > 80) rpm_left = 60;
    if(rpm_right > 80) rpm_right = 60;
    std::ostringstream ss;
    ss << "1" << "," << rpm_right << "," << rpm_left << ";" << "\n";
    std::string out = ss.str();
    ::write(serial_fd_, out.c_str(), out.size());
    RCLCPP_INFO(this->get_logger(), "Published: Rvel=%f, Lvel=%f",
                    rpm_right, rpm_left);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}

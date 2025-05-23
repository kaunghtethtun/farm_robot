#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

class SerialNode : public rclcpp::Node
{
public:
  SerialNode() : Node("serial_node")
  {
    // Open serial port
    serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }

    // Configure port
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
    tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver and set local mode
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;              // 8-bit chars
    tty.c_cflag &= ~PARENB;          // No parity
    tty.c_cflag &= ~CSTOPB;          // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;         // No hardware flow control
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    tty.c_oflag &= ~OPOST;           // Raw output

    tcflush(serial_fd_, TCIFLUSH);
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
      close(serial_fd_);
      rclcpp::shutdown();
      return;
    }

    // Create timer to poll data
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&SerialNode::poll_serial, this));

    RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");

    // Send test byte
    const char *test_data = "A\n";
    ::write(serial_fd_, test_data, strlen(test_data));
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

  void poll_serial()
  {
    char buffer[256];
    int n = ::read(serial_fd_, buffer, sizeof(buffer) - 1);
    if (n > 0)
    {
      buffer[n] = '\0';
      RCLCPP_INFO(this->get_logger(), "Received: '%s'", buffer);
    }
    else if (n < 0 && errno != EAGAIN)
    {
      RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", strerror(errno));
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}

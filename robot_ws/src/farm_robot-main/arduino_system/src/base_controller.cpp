#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class OdomNode : public rclcpp::Node
{
public:
  OdomNode()
  : Node("odom_node"),
    x_(0.0), y_(0.0), theta_(0.0),
    last_left_count_(0), last_right_count_(0),
    have_prev_encoder_(false),
    left_rpm_(0.0f), right_rpm_(0.0f),
    imu_received_(false), have_last_imu_yaw_(false)
  {
    // Parameters
    wheel_radius_  = this->declare_parameter<double>("wheel_radius", 0.033);   // meters
    wheel_base_    = this->declare_parameter<double>("wheel_base", 0.21);     // meters
    ticks_per_rev_ = this->declare_parameter<int>("ticks_per_rev", 1317);
    publish_rate_  = this->declare_parameter<double>("publish_rate", 20.0);   // Hz
    pub_tf_        = this->declare_parameter<bool>("pub_tf", true);
    rpm_alpha_     = this->declare_parameter<double>("rpm_alpha", 0.5);       // blending encoder vs RPM
    imu_alpha_     = this->declare_parameter<double>("imu_alpha", 0.7);       // blending IMU vs encoders
    odom_frame_    = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_    = this->declare_parameter<std::string>("base_frame", "base_footprint");

    // Subscribers
    encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "encoder_counts", 10,
      std::bind(&OdomNode::encoderCallback, this, std::placeholders::_1));

    rpm_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "wheel_rpm", 10,
      std::bind(&OdomNode::rpmCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/out", 10,
      std::bind(&OdomNode::imuCallback, this, std::placeholders::_1));

    // Publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // TF broadcaster
    if (pub_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    last_time_ = this->now();

    // Timer for publishing odometry
    auto period_ms = static_cast<int>(1000.0 / publish_rate_);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                     std::bind(&OdomNode::update, this));

    RCLCPP_INFO(this->get_logger(), "OdomNode started (20 Hz, pub_tf=%s)", pub_tf_ ? "true" : "false");
  }

private:
  // --- Callbacks ---
  void encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (msg->data.size() < 2) return;
    left_count_ = msg->data[0];
    right_count_ = msg->data[1];
  }

  void rpmCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (msg->data.size() < 2) return;
    left_rpm_ = msg->data[0];
    right_rpm_ = msg->data[1];
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    imu_yaw_ = yaw;
    imu_received_ = true;
    if (!have_last_imu_yaw_) {
      last_imu_yaw_ = imu_yaw_;
      have_last_imu_yaw_ = true;
    }
  }

  // --- Main update loop ---
  void update()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) return;

    if (!have_prev_encoder_) {
      last_left_count_ = left_count_;
      last_right_count_ = right_count_;
      have_prev_encoder_ = true;
      last_time_ = now;
      return;
    }

    // Encoder deltas
    int delta_left = static_cast<int>(left_count_ - last_left_count_);
    int delta_right = static_cast<int>(right_count_ - last_right_count_);
    last_left_count_ = left_count_;
    last_right_count_ = right_count_;

    double dist_per_rev = 2.0 * M_PI * wheel_radius_;
    double dist_left_enc = (static_cast<double>(delta_left) / ticks_per_rev_) * dist_per_rev;
    double dist_right_enc = (static_cast<double>(delta_right) / ticks_per_rev_) * dist_per_rev;

    double vel_left_rpm = (static_cast<double>(left_rpm_) / 60.0) * dist_per_rev;
    double vel_right_rpm = (static_cast<double>(right_rpm_) / 60.0) * dist_per_rev;

    double fused_left = (1.0 - rpm_alpha_) * dist_left_enc + rpm_alpha_ * (vel_left_rpm * dt);
    double fused_right = (1.0 - rpm_alpha_) * dist_right_enc + rpm_alpha_ * (vel_right_rpm * dt);

    double delta_s = (fused_right + fused_left) / 2.0;
    double delta_theta_enc = (fused_right - fused_left) / wheel_base_;

    // Use IMU yaw delta if available
    double delta_theta = delta_theta_enc;
    if (imu_received_ && have_last_imu_yaw_) {
      double delta_theta_imu = std::atan2(std::sin(imu_yaw_ - last_imu_yaw_),
                                          std::cos(imu_yaw_ - last_imu_yaw_));
      delta_theta = imu_alpha_ * delta_theta_imu + (1.0 - imu_alpha_) * delta_theta_enc;
      last_imu_yaw_ = imu_yaw_;
    }

    // Update position
    double theta_mid = theta_ + delta_theta / 2.0;
    x_ += delta_s * std::cos(theta_mid);
    y_ += delta_s * std::sin(theta_mid);
    theta_ = std::atan2(std::sin(theta_ + delta_theta), std::cos(theta_ + delta_theta));

    // Velocities
    double vx = delta_s / dt;
    double vth = delta_theta / dt;

    // --- Publish odometry ---
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;

    // Covariance (tuned example)
    for (int i = 0; i < 36; ++i) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0] = 1e-2;   // x
    odom.pose.covariance[7] = 1e-2;   // y
    odom.pose.covariance[35] = 3e-2;  // yaw

    odom_pub_->publish(odom);

    if (pub_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now;
      t.header.frame_id = odom_frame_;
      t.child_frame_id = base_frame_;
      t.transform.translation.x = x_;
      t.transform.translation.y = y_;
      t.transform.translation.z = 0.0;
      t.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(t);
    }

    last_time_ = now;
  }

  // --- Members ---
  double wheel_radius_, wheel_base_;
  int ticks_per_rev_;
  double publish_rate_;
  bool pub_tf_;
  double rpm_alpha_, imu_alpha_;
  std::string odom_frame_, base_frame_;

  double x_, y_, theta_;
  int64_t last_left_count_, last_right_count_;
  bool have_prev_encoder_;
  int64_t left_count_{0}, right_count_{0};
  float left_rpm_, right_rpm_;
  rclcpp::Time last_time_;

  double imu_yaw_;
  double last_imu_yaw_;
  bool imu_received_;
  bool have_last_imu_yaw_;

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rpm_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::mutex mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


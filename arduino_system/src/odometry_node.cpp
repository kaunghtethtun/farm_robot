// odometry_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // <--- Required for tf2::toMsg

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode() : Node("odometry_node"), x_(0), y_(0), theta_(0),
                     last_left_ticks_(0), last_right_ticks_(0) {
        encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "encoder_ticks", 10,
            std::bind(&OdometryNode::encoderCallback, this, std::placeholders::_1));
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&OdometryNode::cmdVelCallback, this, std::placeholders::_1));

        motor_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("motor_command", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        last_time_ = this->now();
    }

private:
    void encoderCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        int left_ticks = msg->data[0];
        int right_ticks = msg->data[1];

        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        double delta_left = tickToDistance(left_ticks - last_left_ticks_);
        double delta_right = tickToDistance(right_ticks - last_right_ticks_);

        last_left_ticks_ = left_ticks;
        last_right_ticks_ = right_ticks;

        double delta_s = (delta_right + delta_left) / 2.0;
        double delta_theta = (delta_right - delta_left) / WHEEL_BASE;

        x_ += delta_s * cos(theta_ + delta_theta / 2.0);
        y_ += delta_s * sin(theta_ + delta_theta / 2.0);
        theta_ += delta_theta;

        // Publish Odometry
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation = tf2::toMsg(q);
        odom.twist.twist.linear.x = delta_s / dt;
        odom.twist.twist.angular.z = delta_theta / dt;
        odom_pub_->publish(odom);

        // TF
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Convert cmd_vel to wheel speeds
        double v = msg->linear.x;
        double w = msg->angular.z;
        RCLCPP_INFO(this->get_logger(), "Published: vel=%f, vel=%f",v, w);
        double v_left = v - (w * WHEEL_BASE / 2.0);
        double v_right = v + (w * WHEEL_BASE / 2.0);
        //RCLCPP_INFO(this->get_logger(), "Published: v_left=%f, v_right=%f",v_left, v_right);

        geometry_msgs::msg::Twist motor_cmd;
        motor_cmd.linear.x = v_left;
        motor_cmd.angular.z = v_right;
        motor_pub_->publish(motor_cmd);
    }

    double tickToDistance(int ticks) {
        return 2 * M_PI * WHEEL_RADIUS * ticks / TICKS_PER_REV;
    }

    // Constants
    const double WHEEL_RADIUS = 0.03;   // meters
    const double WHEEL_BASE = 0.16;     // meters
    const int TICKS_PER_REV = 940;

    double x_, y_, theta_;
    int last_left_ticks_, last_right_ticks_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryNode>());
  rclcpp::shutdown();
  return 0;
}



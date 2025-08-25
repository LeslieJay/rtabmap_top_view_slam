/*
 * @Autor: wei.canming
 * @Version: 1.0
 * @Date: 2025-07-23 15:34:11
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-07-25 14:03:16
 * @Description: 
 */
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class FakeOdomPublisher : public rclcpp::Node
{
public:
    FakeOdomPublisher()
    : Node("fake_odom_publisher"), x_(0.0), y_(0.0), theta_(0.0)
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        timer_ = this->create_wall_timer(100ms, std::bind(&FakeOdomPublisher::timer_callback, this));
        last_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "虚假里程计发布器已启动，模拟0.1m/s直线前进");
    }

private:
    void timer_callback()
    {
        // 计算时间间隔
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        // 以0.1m/s沿x方向前进
        double vx = 0.1;
        x_ += vx * dt;

        // 发布TF变换：odom -> base_link
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(t);

        // 构造Odom消息
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // 位置
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        // 姿态（无旋转）
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // 速度
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        odom_pub_->publish(odom_msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;
    double x_, y_, theta_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}
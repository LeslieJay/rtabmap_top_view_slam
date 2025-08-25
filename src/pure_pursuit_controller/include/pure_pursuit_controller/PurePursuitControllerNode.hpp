/*
 * @Autor: wei.canming
 * @Version: 1.0
 * @Date: 2025-07-16 16:53:58
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-07-17 08:56:20
 * @Description: 
 */
#ifndef PURE_PURSUIT_CONTROLLER_NODE_HPP
#define PURE_PURSUIT_CONTROLLER_NODE_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

namespace pure_pursuit_controller {

class PurePursuitControllerNode : public rclcpp::Node {
public:
    // 构造函数
    explicit PurePursuitControllerNode();
    
    // 析构函数
    ~PurePursuitControllerNode() = default;

private:
    // 回调函数
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    
    // 控制相关函数
    bool step(geometry_msgs::msg::Twist& twist);
    geometry_msgs::msg::PoseStamped getCurrentPose() const;
    double getLookAheadDistance(const geometry_msgs::msg::PoseStamped& pose) const;
    double getLookAheadAngle(const geometry_msgs::msg::PoseStamped& pose) const;
    double getLookAheadThreshold() const;
    double getArcDistance(const geometry_msgs::msg::PoseStamped& pose) const;
    int getNextWayPoint(int wayPoint) const;
    int getClosestWayPoint() const;
    bool getInterpolatedPose(int wayPoint, geometry_msgs::msg::PoseStamped& interpolatedPose) const;
    
    // 参数声明和获取
    void declareParameters();
    void getParameters();
    
    // 订阅者
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    
    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_velocity_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_trajectory_publisher_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // TF相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 参数
    int queue_depth_;
    std::string path_topic_name_;
    std::string odometry_topic_name_;
    std::string cmd_velocity_topic_name_;
    std::string cmd_trajectory_topic_name_;
    std::string pose_frame_id_;
    
    double frequency_;
    int initial_waypoint_;
    double velocity_;
    double look_ahead_ratio_;
    double epsilon_;
    
    // 状态变量
    nav_msgs::msg::Path current_reference_path_;
    geometry_msgs::msg::Twist current_velocity_;
    int next_waypoint_;
};

} // namespace pure_pursuit_controller

#endif // PURE_PURSUIT_CONTROLLER_NODE_HPP 
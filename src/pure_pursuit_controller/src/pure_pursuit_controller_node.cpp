#include "pure_pursuit_controller/PurePursuitControllerNode.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pure_pursuit_controller {

PurePursuitControllerNode::PurePursuitControllerNode()
    : Node("pure_pursuit_controller"),
      next_waypoint_(-1) {
    
    // 声明和获取参数
    declareParameters();
    getParameters();
    
    // 初始化TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 创建订阅者
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic_name_, 
        rclcpp::QoS(queue_depth_),
        std::bind(&PurePursuitControllerNode::pathCallback, this, std::placeholders::_1));
        
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_name_,
        rclcpp::QoS(queue_depth_),
        std::bind(&PurePursuitControllerNode::odometryCallback, this, std::placeholders::_1));
    
    // 创建发布者
    cmd_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        cmd_velocity_topic_name_,
        rclcpp::QoS(queue_depth_));
        
    cmd_trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        cmd_trajectory_topic_name_,
        rclcpp::QoS(queue_depth_));
    
    // 创建定时器
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0/frequency_),
        std::bind(&PurePursuitControllerNode::timerCallback, this));
}

void PurePursuitControllerNode::declareParameters() {
    // ROS参数
    this->declare_parameter("ros.queue_depth", 100);
    this->declare_parameter("ros.path_topic_name", "/reference_path");
    this->declare_parameter("ros.odometry_topic_name", "/robot_state/odometry");
    this->declare_parameter("ros.cmd_velocity_topic_name", "/command_velocity");
    this->declare_parameter("ros.cmd_trajectory_topic_name", "/local_planner_solution_trajectory");
    this->declare_parameter("ros.pose_frame_id", "base_link");
    
    // 控制器参数
    this->declare_parameter("controller.frequency", 20.0);
    this->declare_parameter("controller.initial_waypoint", -1);
    this->declare_parameter("controller.velocity", 0.2);
    this->declare_parameter("controller.look_ahead_ratio", 1.0);
    this->declare_parameter("controller.epsilon", 1e-6);
}

void PurePursuitControllerNode::getParameters() {
    // ROS参数
    queue_depth_ = this->get_parameter("ros.queue_depth").as_int();
    path_topic_name_ = this->get_parameter("ros.path_topic_name").as_string();
    odometry_topic_name_ = this->get_parameter("ros.odometry_topic_name").as_string();
    cmd_velocity_topic_name_ = this->get_parameter("ros.cmd_velocity_topic_name").as_string();
    cmd_trajectory_topic_name_ = this->get_parameter("ros.cmd_trajectory_topic_name").as_string();
    pose_frame_id_ = this->get_parameter("ros.pose_frame_id").as_string();
    
    // 控制器参数
    frequency_ = this->get_parameter("controller.frequency").as_double();
    initial_waypoint_ = this->get_parameter("controller.initial_waypoint").as_int();
    velocity_ = this->get_parameter("controller.velocity").as_double();
    look_ahead_ratio_ = this->get_parameter("controller.look_ahead_ratio").as_double();
    epsilon_ = this->get_parameter("controller.epsilon").as_double();
}

void PurePursuitControllerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    current_reference_path_ = *msg;
    next_waypoint_ = -1;
}

void PurePursuitControllerNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_velocity_ = msg->twist.twist;
}

void PurePursuitControllerNode::timerCallback() {
    geometry_msgs::msg::Twist cmd_velocity;
    
    if (step(cmd_velocity)) {
        const size_t num_points = 20;
        double look_ahead_threshold = getLookAheadThreshold();
        
        auto cmd_trajectory = std::make_unique<visualization_msgs::msg::Marker>();
        cmd_trajectory->header.frame_id = pose_frame_id_;
        cmd_trajectory->header.stamp = this->now();
        cmd_trajectory->ns = "solution_trajectory";
        cmd_trajectory->type = visualization_msgs::msg::Marker::LINE_STRIP;
        cmd_trajectory->action = visualization_msgs::msg::Marker::ADD;
        cmd_trajectory->scale.x = 0.12;
        
        cmd_trajectory->color.r = 0.0;
        cmd_trajectory->color.g = 0.0;
        cmd_trajectory->color.b = 1.0;
        cmd_trajectory->color.a = 1.0;
        
        cmd_trajectory->frame_locked = true;
        cmd_trajectory->points.resize(num_points);
        
        for (size_t i = 0; i < num_points; ++i) {
            geometry_msgs::msg::Pose pose;
            double dt = look_ahead_threshold * static_cast<double>(i) / static_cast<double>(num_points);
            
            pose.orientation.z = cmd_velocity.angular.z * dt;
            pose.position.x = cmd_velocity.linear.x * std::cos(pose.orientation.z) * dt;
            pose.position.y = cmd_velocity.linear.x * std::sin(pose.orientation.z) * dt;
            
            cmd_trajectory->points[i] = pose.position;
        }
        
        cmd_trajectory_publisher_->publish(std::move(cmd_trajectory));
        cmd_velocity_publisher_->publish(cmd_velocity);
    }
}

geometry_msgs::msg::PoseStamped PurePursuitControllerNode::getCurrentPose() const {
    geometry_msgs::msg::PoseStamped pose, transformed_pose;
    pose.header.frame_id = pose_frame_id_;
    pose.header.stamp = this->now();
    
    try {
        transformed_pose = tf_buffer_->transform(pose, current_reference_path_.header.frame_id);
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "PurePursuitControllerNode::getCurrentPose: %s", ex.what());
    }
    
    return transformed_pose;
}

bool PurePursuitControllerNode::step(geometry_msgs::msg::Twist& twist) {
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    
    next_waypoint_ = getNextWayPoint(next_waypoint_);
    
    if (next_waypoint_ >= 0) {
        geometry_msgs::msg::PoseStamped pose;
        
        if (getInterpolatedPose(next_waypoint_, pose)) {
            double look_ahead_distance = getLookAheadDistance(pose);
            double look_ahead_angle = getLookAheadAngle(pose);
            
            double angular_velocity = 0.0;
            
            if (std::abs(std::sin(look_ahead_angle)) >= epsilon_) {
                double radius = 0.5 * (look_ahead_distance/std::sin(look_ahead_angle));
                double linear_velocity = velocity_;
                
                if (std::abs(radius) >= epsilon_)
                    angular_velocity = linear_velocity/radius;
                    
                twist.linear.x = linear_velocity;
                twist.angular.z = angular_velocity;
                
                return true;
            }
        }
    }
    
    return false;
}

double PurePursuitControllerNode::getLookAheadDistance(
    const geometry_msgs::msg::PoseStamped& pose) const {
    geometry_msgs::msg::PoseStamped origin = getCurrentPose();
    geometry_msgs::msg::PoseStamped transformed_pose;
    
    try {
        transformed_pose = tf_buffer_->transform(
            pose, 
            current_reference_path_.header.frame_id);
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), 
            "PurePursuitControllerNode::getLookAheadDistance: %s", ex.what());
        return -1.0;
    }
    
    tf2::Vector3 v1(origin.pose.position.x,
                    origin.pose.position.y,
                    origin.pose.position.z);
                    
    tf2::Vector3 v2(transformed_pose.pose.position.x,
                    transformed_pose.pose.position.y,
                    transformed_pose.pose.position.z);
    
    return tf2::tf2Distance(v1, v2);
}

double PurePursuitControllerNode::getLookAheadAngle(
    const geometry_msgs::msg::PoseStamped& pose) const {
    geometry_msgs::msg::PoseStamped origin = getCurrentPose();
    geometry_msgs::msg::PoseStamped transformed_pose;
    
    try {
        transformed_pose = tf_buffer_->transform(
            pose, 
            current_reference_path_.header.frame_id);
    }
    catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(),
            "PurePursuitControllerNode::getLookAheadAngle: %s", ex.what());
        return -1.0;
    }
    
    tf2::Vector3 v1(origin.pose.position.x,
                    origin.pose.position.y,
                    origin.pose.position.z);
                    
    tf2::Vector3 v2(transformed_pose.pose.position.x,
                    transformed_pose.pose.position.y,
                    transformed_pose.pose.position.z);
    
    return tf2::tf2Angle(v1, v2);
}

double PurePursuitControllerNode::getLookAheadThreshold() const {
    return look_ahead_ratio_ * current_velocity_.linear.x;
}

double PurePursuitControllerNode::getArcDistance(
    const geometry_msgs::msg::PoseStamped& pose) const {
    double look_ahead_distance = getLookAheadDistance(pose);
    double look_ahead_angle = getLookAheadAngle(pose);
    
    if (std::abs(std::sin(look_ahead_angle)) >= epsilon_)
        return look_ahead_distance/std::sin(look_ahead_angle) * look_ahead_angle;
    else
        return look_ahead_distance;
}

int PurePursuitControllerNode::getNextWayPoint(int wayPoint) const {
    if (!current_reference_path_.poses.empty()) {
        if (next_waypoint_ >= 0) {
            geometry_msgs::msg::PoseStamped origin = getCurrentPose();
            tf2::Vector3 v1(origin.pose.position.x,
                           origin.pose.position.y,
                           origin.pose.position.z);
            double look_ahead_threshold = getLookAheadThreshold();
            
            for (size_t i = next_waypoint_; i < current_reference_path_.poses.size(); ++i) {
                tf2::Vector3 v2(current_reference_path_.poses[i].pose.position.x,
                               current_reference_path_.poses[i].pose.position.y,
                               current_reference_path_.poses[i].pose.position.z);
                               
                if (tf2::tf2Distance(v1, v2) > look_ahead_threshold)
                    return i;
            }
            
            return next_waypoint_;
        }
        else
            return 0;
    }
    
    return -1;
}

int PurePursuitControllerNode::getClosestWayPoint() const {
    if (!current_reference_path_.poses.empty()) {
        int closest_waypoint = -1;
        double min_distance = -1.0;
        
        for (size_t i = 0; i < current_reference_path_.poses.size(); ++i) {
            double distance = getArcDistance(current_reference_path_.poses[i]);
            
            if ((min_distance < 0.0) || (distance < min_distance)) {
                closest_waypoint = i;
                min_distance = distance;
            }
        }
        
        return closest_waypoint;
    }
    
    return -1;
}

bool PurePursuitControllerNode::getInterpolatedPose(
    int wayPoint,
    geometry_msgs::msg::PoseStamped& interpolatedPose) const {
    if (!current_reference_path_.poses.empty()) {
        if (wayPoint > 0) {
            double l_t = getLookAheadThreshold();
            double p_t = getLookAheadDistance(
                current_reference_path_.poses[next_waypoint_-1]);
            
            if (p_t < l_t) {
                geometry_msgs::msg::PoseStamped p_0 = getCurrentPose();
                geometry_msgs::msg::PoseStamped p_1 = 
                    current_reference_path_.poses[wayPoint-1];
                geometry_msgs::msg::PoseStamped p_2 = 
                    current_reference_path_.poses[wayPoint];
                
                tf2::Vector3 v1(p_2.pose.position.x-p_0.pose.position.x,
                               p_2.pose.position.y-p_0.pose.position.y,
                               p_2.pose.position.z-p_0.pose.position.z);
                               
                tf2::Vector3 v2(p_1.pose.position.x-p_0.pose.position.x,
                               p_1.pose.position.y-p_0.pose.position.y,
                               p_1.pose.position.z-p_0.pose.position.z);
                               
                tf2::Vector3 v0(p_2.pose.position.x-p_1.pose.position.x,
                               p_2.pose.position.y-p_1.pose.position.y,
                               p_2.pose.position.z-p_1.pose.position.z);
                
                double l0 = v0.length();
                double l1 = v1.length();
                double l2 = v2.length();
                
                v0.normalize();
                v2.normalize();
                
                double alpha1 = M_PI-tf2::tf2Angle(v0, v2);
                double beta2 = std::asin(l2*std::sin(alpha1)/l_t);
                double beta0 = M_PI-alpha1-beta2;
                double l_s = l2*std::sin(beta0)/std::sin(beta2);
                
                tf2::Vector3 p_s(p_1.pose.position.x+v0[0]*l_s,
                                p_1.pose.position.y+v0[1]*l_s,
                                p_1.pose.position.z+v0[2]*l_s);
                
                interpolatedPose.pose.position.x = p_s[0];
                interpolatedPose.pose.position.y = p_s[1];
                interpolatedPose.pose.position.z = p_s[2];
                
                return true;
            }
        }
        
        interpolatedPose = current_reference_path_.poses[wayPoint];
        return true;
    }
    
    return false;
}

} // namespace pure_pursuit_controller


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<pure_pursuit_controller::PurePursuitControllerNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pure_pursuit_controller"), "Exception: %s", e.what());
        return 1;
    }
    catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("pure_pursuit_controller"), "Unknown Exception");
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

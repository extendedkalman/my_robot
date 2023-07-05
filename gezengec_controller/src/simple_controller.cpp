#include "gezengec_controller/simple_controller.h"
#include <std_msgs/Float64.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

SimpleController::SimpleController(const ros::NodeHandle &nh,
                                   double radius,
                                   double separation)
                                  : nh_(nh),
                                  wheel_radius_(radius),
                                  wheel_seperation_(separation),
                                  left_wheel_prev_pos_(0.0),
                                  right_wheel_prev_pos_(0.0),
                                  x_(0.0),
                                  y_(0.0),
                                  theta_(0.0)
{
    ROS_INFO_STREAM("Using wheel radius " << radius);
    ROS_INFO_STREAM("Using wheel separation " << separation);
    
    prev_time_ = ros::Time::now();

    right_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_right_controller/command", 10);
    left_cmd_pub_ = nh_.advertise<std_msgs::Float64>("wheel_left_controller/command", 10);
    vel_sub_ = nh_.subscribe("gezengec_controller/cmd_vel", 1000, &SimpleController::velCallback, this);
    joint_sub_ = nh_.subscribe("joint_states", 1000, &SimpleController::jointCallback, this);
    speed_conversion_ << radius/2, radius/2, radius/separation, -radius/separation;
    ROS_INFO_STREAM("The conversion matrix is \n" << speed_conversion_);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint";
}


void SimpleController::velCallback(const geometry_msgs::Twist &msg)
{
    // Implements the differential kinematic model
    // Given v and w, calculate the velocities of the wheels
    Eigen::Vector2d robot_speed(msg.linear.x, msg.angular.z);
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;
    std_msgs::Float64 right_speed;
    right_speed.data = wheel_speed.coeff(0);
    std_msgs::Float64 left_speed;
    left_speed.data = wheel_speed.coeff(1);

    right_cmd_pub_.publish(right_speed);
    left_cmd_pub_.publish(left_speed);
}

void SimpleController::jointCallback(const sensor_msgs::JointState &state)
{
    double dp_left = state.position.at(0) - left_wheel_prev_pos_;
    double dp_right = state.position.at(1) - right_wheel_prev_pos_;
    double dt = (state.header.stamp - prev_time_).toSec();

    left_wheel_prev_pos_ = state.position.at(0);
    right_wheel_prev_pos_ = state.position.at(1);
    prev_time_ = state.header.stamp;

    double fi_left = dp_left / dt;
    double fi_right = dp_right / dt;

    double linear = (wheel_radius_ * fi_right + wheel_radius_* fi_left) / 2;
    double angular = (wheel_radius_ * fi_right - wheel_radius_* fi_left) / wheel_seperation_;

    double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left) / 2.0;
    double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_seperation_;

    theta_ += d_theta;
    x_ += d_s * std::cos(theta_);
    y_ += d_s * std::sin(theta_);

    //ROS_INFO_STREAM("linear: " << linear << " angular: " << angular);
    //ROS_INFO_STREAM("x: " << x);
    //ROS_INFO_STREAM("y: " << y);
    //ROS_INFO_STREAM("theta: " << theta_);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();

    odom_msg_.header.stamp = ros::Time::now();

    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    
    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z = angular;
    odom_pub_.publish(odom_msg_);

    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;

    transform_stamped_.transform.rotation.x = q.x();
    transform_stamped_.transform.rotation.y = q.y();
    transform_stamped_.transform.rotation.z = q.z();
    transform_stamped_.transform.rotation.w = q.w();
    transform_stamped_.header.stamp = ros::Time::now();

    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(transform_stamped_);


}
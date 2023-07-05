#include "gezengec_controller/simple_controller.h"
#include <std_msgs/Float64.h>
#include <Eigen/Geometry>


SimpleController::SimpleController(const ros::NodeHandle &nh,
                                   double radius,
                                   double separation)
                                  : nh_(nh),
                                  wheel_radius_(radius),
                                  wheel_seperation_(separation),
                                  left_wheel_prev_pos_(0.0),
                                  right_wheel_prev_pos_(0.0)
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

    ROS_INFO_STREAM("linear: " << linear << " angular: " << angular);

}
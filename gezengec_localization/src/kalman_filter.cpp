#include "gezengec_localization/kalman_filter.h"

KalmanFilter::KalmanFilter(const ros::NodeHandle &nh):nh_{nh},
                mean_{0.0},
                variance_{1000.0},
                imu_angular_z_{0.0},
                is_first_odom_{true},
                last_angular_z_{0.0},
                motion_{0.0},
                motion_variance_{4.0},
                measurement_variance_{0.5}
{
    odom_sub_ = nh_.subscribe("/gezengec_controller/odom_noisy", 1000, &KalmanFilter::odomCallback, this);
    imu_sub_ = nh_.subscribe("/imu", 1000, &KalmanFilter::imuCallback, this);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("gezengec_controller/odom_kalman", 10);

}

void KalmanFilter::imuCallback(const sensor_msgs::Imu &imu_msg)
{
    imu_angular_z_ = imu_msg.angular_velocity.z;

}

void KalmanFilter::odomCallback(const nav_msgs::Odometry& odom_msg)
{
    kalman_odom_ = odom_msg;
    if(is_first_odom_)
    {
        mean_ = odom_msg.twist.twist.angular.z;
        last_angular_z_ = odom_msg.twist.twist.angular.z;
        is_first_odom_ = false;
        return;
    }

    motion_ = odom_msg.twist.twist.angular.z - last_angular_z_;

    state_prediction();
    measurement_update();

    kalman_odom_.twist.twist.angular.z = mean_;
    odom_pub_.publish(kalman_odom_);
    last_angular_z_ = odom_msg.twist.twist.angular.z;
}

void KalmanFilter::measurement_update()
{
    mean_ = (measurement_variance_ * mean_ + variance_ * imu_angular_z_) / (variance_ + measurement_variance_);
    variance_ = (variance_ * measurement_variance_) / (variance_ + measurement_variance_); 
}

void KalmanFilter::state_prediction()
{
    motion_ = mean_ + motion_;
    variance_ = variance_ + motion_variance_;
}
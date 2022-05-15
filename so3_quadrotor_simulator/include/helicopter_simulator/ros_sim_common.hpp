#pragma once
#include "Helicopter.hpp"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>

void stateToOdomMsg(const Helicopter::State &_state, nav_msgs::Odometry &odom)
{
    auto state = _state.kstate;
    odom.pose.pose.position.x = state.x(0);
    odom.pose.pose.position.y = state.x(1);
    odom.pose.pose.position.z = state.x(2);

    Eigen::Quaterniond q(state.R);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = state.v(0);
    odom.twist.twist.linear.y = state.v(1);
    odom.twist.twist.linear.z = state.v(2);

    odom.twist.twist.angular.x = state.omega(0);
    odom.twist.twist.angular.y = state.omega(1);
    odom.twist.twist.angular.z = state.omega(2);
}

void quadToImuMsg(const Helicopter &quad, sensor_msgs::Imu &imu)
{
    auto state = quad.getState().kstate;
    Eigen::Quaterniond q(state.R);
    imu.orientation.x = q.x();
    imu.orientation.y = q.y();
    imu.orientation.z = q.z();
    imu.orientation.w = q.w();

    imu.angular_velocity.x = state.omega(0);
    imu.angular_velocity.y = state.omega(1);
    imu.angular_velocity.z = state.omega(2);

    imu.linear_acceleration.x = quad.getAcc().x();
    imu.linear_acceleration.y = quad.getAcc().y();
    imu.linear_acceleration.z = quad.getAcc().z();
}
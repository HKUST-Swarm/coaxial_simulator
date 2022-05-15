#pragma once
#include <Eigen/Dense>
#include "common.hpp"

struct ActuatorParam {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

class Actuator {
public:
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Actuator() {
    }

    Actuator(ActuatorParam param): position(param.position), orientation(param.orientation)
    {}

    virtual void operator() (const KinematicsState & kstate, const Eigen::VectorXd &x, Eigen::VectorXd &dxdt) const = 0;
    virtual int get_state_dim() const = 0;
    virtual Eigen::VectorXd get_state() const = 0;
    virtual int get_control_dim() const = 0;
    virtual void set_state(const Eigen::VectorXd &x) = 0;
    virtual int print_controls(int start_index) const = 0;
    virtual int print_states(int start_index) const = 0;
    virtual int set_controls(const Eigen::VectorXd& input) = 0;
};


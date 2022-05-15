#pragma once
#include <Eigen/Dense>
#include "actuator.hpp"

#define MOTOR_STATE_DIM 1

struct MotorParam {
    double throttle_rpm_k = 4764.72;//Thrust to speed with load
    double throttle_rpm0 = 532.79; //Idle speed
    double tau = 0.03;
    double gear_ratio = 1.0; //Gear ratio
};

struct MotorState {
    double current_rpm;
};

struct MotorControls {
    double throttle = 15.2;
    double voltage;
};

template<typename T>
inline T rpm2rps(const T & rpm) {
    return rpm/60;
}

template<typename T>
inline T rpm2radps(const T & rpm) {  //Omega
    return rpm/60*2*M_PI;
}

class Motor: public Actuator {
    MotorParam param; 
    MotorState state;
    MotorControls drive_state;
    double target_rpm = 0;
public:
    Motor(MotorParam _param): param(_param) {
        state.current_rpm = param.throttle_rpm0; //The motor is start at idle speed to avoid NaN issue.
    }

    int set_controls(const Eigen::VectorXd& input) override { 
        drive_state.throttle = input(0);
        target_rpm = (drive_state.throttle*param.throttle_rpm_k + param.throttle_rpm0)/param.gear_ratio;
        return 1;
    }

    virtual void operator() (const KinematicsState & kstate, const Eigen::VectorXd &x, Eigen::VectorXd &dxdt) const override {
        //For motor we have
        //RPM = 1/(tau*s+1) RPM_T
        //d(RPM)/dt=(RPM_T-RPM)/tau
        dxdt(0) = (target_rpm - x(0))/param.tau;
    }

    virtual int get_state_dim() const override {
        return MOTOR_STATE_DIM;
    }

    virtual void set_state(const Eigen::VectorXd &x) {
        state.current_rpm = x(0);
    }

    virtual Eigen::VectorXd get_state() const {
        Eigen::VectorXd x(MOTOR_STATE_DIM);
        x(0) = state.current_rpm;
        return x;
    }

    double rpm_from_state(const Eigen::VectorXd &x) const {
        return x(0);
    }

    int get_control_dim() const override {
        return 1;
    }

    virtual int print_controls(int start_index) const override {
        printf("CTRL AXIS %d: Motor RPM\n", start_index);
        return start_index + 1;
    }

    virtual int print_states(int start_index) const override {
        printf("STATE AXIS %d: Motor RPM\n", start_index);
        return start_index + 1;
    }
};

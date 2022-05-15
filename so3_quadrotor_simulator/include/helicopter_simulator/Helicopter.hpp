#pragma once

#include "propeller.hpp"
#include "helicopter_rotor.hpp"
#include <boost/array.hpp>
#include <vector>

enum HeliType {
    FixedPitchHelicopter,
    VariablePitchHelicopter,
    CoaxialFixedPitchHelicopter, //Default up free flapping, down semirigid rotor, control on down.
    CoaxialVariablePitchHelicopter,
    HELI_TYPE_COUNT
};

const std::string HeliTypeStr[] = {
    "FixedPitchHelicopter",
    "VariablePitchHelicopter",
    "CoaxialFixedPitchHelicopter",
    "CoaxialVariablePitchHelicopter",
    "NONE"
};

struct HelicopterParam {
    double mass = 1.4;
    double Cd = 0.6 * 0.2;
    double Cang = 0.0005;
    Eigen::Matrix3d J_ = Eigen::Matrix3d::Identity();  // Inertia
};

class Helicopter {
public:
    struct State {
        KinematicsState kstate;
        Eigen::VectorXd actuator_internal_states; //For rotors, props etc/
    };
    
    typedef Eigen::VectorXd InternalState;
    typedef std::vector<double> InternalStateBoost;

    Helicopter(HeliType _type);
    // Runs the actual dynamics simulation with a time step of dt
    void step(double dt);

    // For internal use, but needs to be public for odeint
    void operator()(const Eigen::VectorXd& x, Eigen::VectorXd& dxdt,
        const double /* t */);

    void operator()(const Helicopter::InternalStateBoost& x, Helicopter::InternalStateBoost& dxdt,
        const double /* t */);
        
    int get_state_dim() const;
    int get_actuator_internal_state_dim() const;
    void setInput(const Eigen::VectorXd& input);
    const State& getState() const;
    void setState(const State& state);
    void setStatePos(const Eigen::Vector3d& Pos);
    int get_control_dim() const;

    void print_controls() const;
    void print_states() const;

    Eigen::Matrix3d getInertia() const;

    void setExternalForce(Eigen::Vector3d ext_force);

    void setExternalMoment(Eigen::Vector3d ext_moment);

    Eigen::Vector3d getAcc() const;

protected:

    Eigen::Vector3d acc_imu;

    State state_;
    HelicopterParam param;
    HeliType heli_type;
    std::vector<HeliRotor> rotors;
    std::vector<Propeller> propellers;

    InternalState internal_state_;
    InternalStateBoost internal_state_boost;

    State internalstate2state(const Helicopter::InternalState& internal_state_);

    EnvironmentParam eparam;

    Eigen::Vector3d external_force_, external_moment_;

    void updateInternalState(void);

    std::pair<Eigen::Vector3d,Eigen::Vector3d> sum_forces_and_moments(const Helicopter::InternalState& x, const State& cur_state);

    void create_default(HeliType type);

    void copyinternalstate2boost();
    void copybooststate2internal();

    double t = 0;
};
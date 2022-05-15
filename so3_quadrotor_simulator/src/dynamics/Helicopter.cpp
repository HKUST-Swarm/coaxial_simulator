#include "helicopter_simulator/helicopter_rotor.hpp"
#include "helicopter_simulator/Helicopter.hpp"
#include <iostream>
#include "ode/boost/numeric/odeint.hpp"

namespace odeint = boost::numeric::odeint;

Helicopter::Helicopter(HeliType _type) : heli_type(_type), external_force_(0, 0, 0), external_moment_(0, 0, 0) {
    create_default(_type);
}

void Helicopter::create_default(HeliType type) {
    if (type == FixedPitchHelicopter) {
        //Rotor and a propeller for tail
        RotorParam rparam;
        rparam.aparam.position = Eigen::Vector3d(0, 0, 0.1);
        rparam.aparam.orientation = Eigen::Quaterniond::Identity();

        HeliRotor rotor(rparam);
        rotors.emplace_back(rotor);

        //Below is SNAIL motor with 5inch propeller
        PropParam pparam;
        pparam.aparam.position = Eigen::Vector3d(-0.42, 0, 0);
        pparam.aparam.orientation = Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitZ());

        pparam.motor_param.throttle_rpm_k = 2400*15.2*0.66; //0.66 is to simulate load
        pparam.motor_param.throttle_rpm0 = 100;
        pparam.motor_param.tau = 0.2; //From manual

        Propeller prop(pparam);

        propellers.emplace_back(prop);
         
    } else if (type == CoaxialFixedPitchHelicopter) {

        RotorParam rparam;
        rparam.aparam.position = Eigen::Vector3d(0, 0, 0.044);
        rparam.aparam.orientation = Eigen::Quaterniond::Identity();

        HeliRotor rotor(rparam);
        rotors.emplace_back(rotor);

        RotorParam rparam2;
        rparam2.aparam.position = Eigen::Vector3d(0, 0, 0.151);
        rparam2.aparam.orientation = Eigen::Quaterniond::Identity();
        rparam2.clockwise = -1;
        rparam2.enable_cyclic_control = false;
        rparam2.kB = 0;
        HeliRotor rotor2(rparam2);
        rotors.emplace_back(rotor2);
        
        param.J_(0, 0) = 0.009695;
        param.J_(1, 1) = 0.00971657;
        param.J_(2, 2) = 0.00117827;
   
    } else {
        printf("Aircraft not supported yet!\n");
        exit(-1);
    }


    state_.actuator_internal_states.resize(get_actuator_internal_state_dim());
    state_.actuator_internal_states.setZero();

    int state_index = 0;
    for (auto& rotor : rotors) {
        state_.actuator_internal_states.segment(state_index, rotor.get_state_dim()) = rotor.get_state();
        state_index += rotor.get_state_dim();
    }

    for (auto& prop : propellers) {
        internal_state_.segment(KINEMATICS_STATE_DIM + state_index, prop.get_state_dim()) = prop.get_state();
        state_index += prop.get_state_dim();
    }

    internal_state_.resize(get_state_dim());
    internal_state_boost.resize(get_state_dim());
    updateInternalState();

    print_controls();
    print_states();

}

void Helicopter::copyinternalstate2boost() {
    memcpy(internal_state_boost.data(), internal_state_.data(), sizeof(double)*internal_state_boost.size());
}

void Helicopter::copybooststate2internal() {
    memcpy(internal_state_.data(), internal_state_boost.data(), sizeof(double)*internal_state_boost.size());
}


Helicopter::State Helicopter::internalstate2state(const Helicopter::InternalState& internal_state_) {
    Helicopter::State state_;
    auto& kstate = state_.kstate;
    kstate.x = internal_state_.segment<DIM_POS>(0);
    kstate.v = internal_state_.segment<DIM_VEL>(DIM_POS);

    for (int i = 0; i < 3; i++) {
        kstate.R(i, 0) = internal_state_(6 + i);
        kstate.R(i, 1) = internal_state_(9 + i);
        kstate.R(i, 2) = internal_state_(12 + i);
    }

    kstate.omega = internal_state_.segment<DIM_OMG>(DIM_POS + DIM_VEL + DIM_R);

    state_.actuator_internal_states = internal_state_.segment(KINEMATICS_STATE_DIM, get_actuator_internal_state_dim());

    // Re-orthonormalize R (polar decomposition)
    Eigen::LLT<Eigen::Matrix3d> llt(kstate.R.transpose() * kstate.R);
    Eigen::Matrix3d P = llt.matrixL();
    Eigen::Matrix3d R = kstate.R * P.inverse();
    kstate.R = R;

    //TODO:
    int state_index = 0;
    for (auto& rotor : rotors) {
        rotor.set_state(state_.actuator_internal_states.segment(state_index, rotor.get_state_dim()));
        state_index += rotor.get_state_dim();
    }

    for (auto& prop : propellers) {
        prop.set_state(state_.actuator_internal_states.segment(state_index, prop.get_state_dim()));
        state_index += prop.get_state_dim();
    }

    return state_;
}

void Helicopter::step(double dt) {
    copyinternalstate2boost();

    auto save = internal_state_;

    odeint::integrate(boost::ref(*this), internal_state_boost, 0.0, dt, dt);


    copybooststate2internal();

    for (int i = 0; i < get_state_dim(); ++i) {
        if (std::isnan(internal_state_(i))) {
            print_states();
            exit(-1);
            break;
        }
    }

    state_ = internalstate2state(internal_state_);

    // Don't go below zero, simulate floor
    if (state_.kstate.x(2) < 0.0 && state_.kstate.v(2) < 0) {
        state_.kstate.x(2) = 0;
        state_.kstate.v(2) = 0;
    }
    updateInternalState();

    static int c = 0;
    if (c++ % ((int)(1.0/dt)) == 0) {
        print_states();
        print_controls();
    }
    t += dt;
}

void Helicopter::operator()(const Helicopter::InternalStateBoost& x, Helicopter::InternalStateBoost& dxdt,
        const double t) {
    //This function is a little bit ugly, try to make it good.
    Helicopter::InternalState _dxdt = Eigen::Map<Helicopter::InternalState>(dxdt.data(), dxdt.size());
    Eigen::Map<const Helicopter::InternalState> _x(x.data(), x.size());
    (*this)(_x, _dxdt, t);
    memcpy(dxdt.data(), _dxdt.data(), sizeof(double)*dxdt.size());
}

void Helicopter::operator()(const Helicopter::InternalState& x, Helicopter::InternalState& dxdt,
    const double /* t */) {
    State cur_state = internalstate2state(x);
    auto cur_kstate = cur_state.kstate;

    Eigen::Vector3d x_dot, v_dot, omega_dot;
    Eigen::Matrix3d R_dot;
    Eigen::Array4d motor_rpm_dot;
    Eigen::Vector3d vnorm;
    Eigen::Array4d motor_rpm_sq;
    Eigen::Matrix3d omega_vee(Eigen::Matrix3d::Zero());
    auto R = cur_kstate.R;

    omega_vee(2, 1) = cur_kstate.omega(0);
    omega_vee(1, 2) = -cur_kstate.omega(0);
    omega_vee(0, 2) = cur_kstate.omega(1);
    omega_vee(2, 0) = -cur_kstate.omega(1);
    omega_vee(1, 0) = cur_kstate.omega(2);
    omega_vee(0, 1) = -cur_kstate.omega(2);

    //Forces from rotors
    //Moments from rotors.
    auto ret = sum_forces_and_moments(x, cur_state);
    Eigen::Vector3d forces = ret.first;
    Eigen::Vector3d moments = ret.second;

    x_dot = cur_kstate.v;
    v_dot = forces / param.mass - Eigen::Vector3d(0, 0, eparam.g);


    acc_imu = v_dot + Eigen::Vector3d(0, 0, eparam.g);

    R_dot = R * omega_vee;
    omega_dot = param.J_.inverse() * (moments - cur_kstate.omega.cross(param.J_ * cur_kstate.omega) + external_moment_);
    // std::cout << "moments" << moments.transpose() << "mass" << param.mass << "acc" << v_dot.transpose() << std::endl;

    dxdt.segment<DIM_POS>(0) = x_dot;
    dxdt.segment<DIM_VEL>(DIM_POS) = v_dot;

    for (int i = 0; i < 3; i++) {
        dxdt(6 + i) = R_dot(i, 0);
        dxdt(9 + i) = R_dot(i, 1);
        dxdt(12 + i) = R_dot(i, 2);
    }

    dxdt.segment<DIM_OMG>(DIM_POS + DIM_VEL + DIM_R) = omega_dot;

    int state_index = KINEMATICS_STATE_DIM;
    for (auto& rotor : rotors) {
        Eigen::VectorXd _dxdt(rotor.get_state_dim());
        rotor(cur_kstate, x.segment(state_index, rotor.get_state_dim()), _dxdt);
        dxdt.segment(state_index, rotor.get_state_dim()) = _dxdt;
        state_index += rotor.get_state_dim();
    }

    for (auto& prop : propellers) {
        Eigen::VectorXd _dxdt(prop.get_state_dim());
        prop(cur_kstate, x.segment(state_index, prop.get_state_dim()), _dxdt);
        dxdt.segment(state_index, prop.get_state_dim()) = _dxdt;
        state_index += prop.get_state_dim();
    }

    // for (int i = 0; i < 22; ++i) {
    //     if (std::isnan(dxdt[i])) {
    //         dxdt[i] = 0;
    //     }
    // }

    // std::cout << "x = " << x.transpose() << std::endl;
    // std::cout << "dxdt = " << dxdt.transpose() << std::endl;
}

std::pair<Eigen::Vector3d,Eigen::Vector3d> Helicopter::sum_forces_and_moments(const Helicopter::InternalState& x, const State& cur_state) {
    auto kstate = cur_state.kstate;
    
    Eigen::Vector3d forces_b(0, 0, 0);
    Eigen::Vector3d moments_b(0, 0, 0);

    int state_index = KINEMATICS_STATE_DIM;
    for (auto& rotor : rotors) {
        auto _force = rotor.get_force(kstate, x.segment(state_index, rotor.get_state_dim()));
        state_index += rotor.get_state_dim();

        auto _force_b = rotor.orientation*_force;
        forces_b += _force_b;
        moments_b += rotor.position.cross(_force_b);
    }

    for (auto& prop : propellers) {
        auto _force = prop.get_force(kstate, x.segment(state_index, prop.get_state_dim()));
        state_index += prop.get_state_dim();

        auto _force_b = prop.orientation*_force;
        forces_b += _force_b;
        moments_b += prop.position.cross(_force_b);
    }

    auto vnorm = cur_state.kstate.v;
    if (vnorm.norm() != 0) {
        vnorm.normalize();
    }
    
    Eigen::Vector3d sum_force = kstate.R * forces_b + external_force_ -
        0.5 * param.Cd * vnorm * kstate.v.norm() * kstate.v.norm();

    state_index = KINEMATICS_STATE_DIM;
    for (auto& rotor : rotors) {
        moments_b += rotor.get_torque(kstate, x.segment(state_index, rotor.get_state_dim()));
        state_index += rotor.get_state_dim();
    }

    for (auto& prop : propellers) {
        moments_b += prop.get_torque(kstate, x.segment(state_index, prop.get_state_dim()));
        state_index += prop.get_state_dim();
    }

    moments_b -= kstate.omega*param.Cang;

    return std::make_pair(sum_force, moments_b + external_moment_);
}


void Helicopter::setInput(const Eigen::VectorXd& input) {
    assert(input.size() == get_control_dim() && "Input size must equal to control dim");
    int dim_count = 0;
    for (auto& rotor : rotors) {
        int _dim = rotor.get_control_dim();
        if (_dim > 0) {
            rotor.set_controls(input.segment(dim_count, _dim));
        }
        dim_count += _dim;
    }

    for (auto& prop : propellers) {
        int _dim = prop.get_control_dim();
        if (_dim > 0) {
            prop.set_controls(input.segment(dim_count, _dim));
        }
        dim_count += _dim;
    }
}


const Helicopter::State& Helicopter::getState() const {
    return state_;
}

void Helicopter::setState(const Helicopter::State& state) {
    auto& kstate = state_.kstate;
    kstate.x = state.kstate.x;
    kstate.v = state.kstate.v;
    kstate.R = state.kstate.R;
    kstate.omega = state.kstate.omega;
    state_.actuator_internal_states = state.actuator_internal_states;
    updateInternalState();
}

void Helicopter::setStatePos(const Eigen::Vector3d& Pos) {
    state_.kstate.x = Pos;

    printf("Set drone pos to %.2f %.2f %.2f\n", Pos.x(), Pos.y(), Pos.z());
    updateInternalState();

    this->print_states();
}

void Helicopter::updateInternalState(void) {
    auto& kstate = state_.kstate;
    for (int i = 0; i < 3; i++) {
        internal_state_[0 + i] = kstate.x(i);
        internal_state_[3 + i] = kstate.v(i);
        internal_state_[6 + i] = kstate.R(i, 0);
        internal_state_[9 + i] = kstate.R(i, 1);
        internal_state_[12 + i] = kstate.R(i, 2);
        internal_state_[15 + i] = kstate.omega(i);
    }

    int state_index = 0;
    for (auto& rotor : rotors) {
        internal_state_.segment(KINEMATICS_STATE_DIM + state_index, rotor.get_state_dim()) = 
            state_.actuator_internal_states.segment(state_index, rotor.get_state_dim());
        state_index += rotor.get_state_dim();
    }

    for (auto& prop : propellers) {
        internal_state_.segment(KINEMATICS_STATE_DIM + state_index, prop.get_state_dim()) = 
            state_.actuator_internal_states.segment(state_index, prop.get_state_dim());
        state_index += prop.get_state_dim();
    }

    copyinternalstate2boost();
}

int Helicopter::get_actuator_internal_state_dim() const {
    int dim_count = 0;
    for (auto& rotor : rotors) {
        dim_count += rotor.get_state_dim();
    }

    for (auto& prop : propellers) {
        dim_count += prop.get_state_dim();
    }

    return dim_count;
}

int Helicopter::get_control_dim() const {
    int dim_count = 0;
    for (auto& rotor : rotors) {
        dim_count += rotor.get_control_dim();
    }

    for (auto& prop : propellers) {
        dim_count += prop.get_control_dim();
    }

    return dim_count;
}

void Helicopter::print_controls() const {
    printf("=============CONTROLS FOR %s =============\n", HeliTypeStr[heli_type].c_str());
    int dim_count = 0;
    for (auto& rotor : rotors) {
        dim_count = rotor.print_controls(dim_count);
    }

    for (auto& prop : propellers) {
        dim_count = prop.print_controls(dim_count);
    }
    assert(dim_count == get_control_dim());
    printf("=============TOTAL CTRL DIM %d =============\n\n", dim_count);

}

void Helicopter::print_states() const {
    printf("t%.3f\n=============STATES FOR %s =============\n", t, HeliTypeStr[heli_type].c_str());
    std::cout << "POS AXIS 0-2: " << state_.kstate.x.transpose() << std::endl;
    std::cout << "VEL AXIS 3-5: " << state_.kstate.v.transpose() << std::endl;
    std::cout << "R AXIS 6-14\n" << state_.kstate.R << std::endl;
    std::cout << "ANG_VEL AXIS 15-17: " << state_.kstate.omega.transpose() << std::endl;

    int dim_count = KINEMATICS_STATE_DIM;
    for (auto& rotor : rotors) {
        dim_count = rotor.print_states(dim_count);
    }

    for (auto& prop : propellers) {
        dim_count = prop.print_states(dim_count);
    }
    printf("=============TOTAL STATE DIM %d =============\n\n", dim_count);

    std::cout << "InternalState" << internal_state_.transpose() << std::endl;

}


int Helicopter::get_state_dim() const {
    return KINEMATICS_STATE_DIM + get_actuator_internal_state_dim();
}

Eigen::Matrix3d Helicopter::getInertia() const {
    return param.J_;
}

void Helicopter::setExternalForce(Eigen::Vector3d ext_force) {
    external_force_ = ext_force;
}

void Helicopter::setExternalMoment(Eigen::Vector3d ext_moment) {
    external_moment_ = ext_moment;
}

Eigen::Vector3d Helicopter::getAcc() const {
    return acc_imu;
}

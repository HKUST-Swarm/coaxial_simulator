#pragma once
#include <Eigen/Dense>
#include "motor.hpp"
#include <iostream>

#define TPP_DIM 2


struct RotorInternalState {
    //State of Tip-path-plane.
    //a is the "pitch" angle of the TPP and b is the roll angle.
    double a = 0; 
    double b = 0; 
};

struct RotorParam {
    double cla = 5.7; //From Seddon, John M., and Simon Newman. Basic helicopter aerodynamics. Vol. 40. John Wiley & Sons, 2011.
    double cl0 = 0; //Zero deg thrust
    double c_blade = 0.03; //Blade width
    double Ib = 0.00052124;
    double kB = 1; //Torque coeff
    double R = 0.326; //Radius of the rotor.
    double C_T0 = 0.06534; //Coeff for thrust
    double k_CT = -0.06534/0.3;//J=0.3 no thrust
    double C_Q = 0.00218; //Coeff for torque
    double Alonlat = 8.0/180.0*M_PI; //Max cyclic pitch
    double Acoll = 12.0/180.0*M_PI; //Max cyclic pitch
    MotorParam motor_param;
    double inflow_speed = 0; //Speed of inflow air.
    int clockwise = 1; //See from top. if counter-clockwise, then -1
    double Irotorhead = 0.00118500; //moment inertial of the whole rotator head. 0.00116347 of up, 0.00118500 for low
    bool fixed_pitch = true;
    bool enable_cyclic_control = true; //Now collective MUST enable after cyclic is enabled.
    ActuatorParam aparam;
};

struct RotorControls {
    MotorControls motor_drive_state;
    double rpm = -1; // rotor speed, RPM. If >0, override motor spd.
    double u_lat = 0; //Lat control, e.g. roll control from -1 to 1
    double u_lon = 0;//lon control, e.g. pitch control from -1 to 1
    double u_collective = 0;//collective control  from -1 to 1
};

class HeliRotor : public Actuator {
    EnvironmentParam eparam;
    RotorInternalState rotor_state;
    RotorParam rparam;
    RotorControls control_state;
    Motor motor;
public:
    HeliRotor(RotorParam _rparam):
        Actuator(_rparam.aparam),
        rparam(_rparam), motor(rparam.motor_param)
    {}
    
    int set_controls(const Eigen::VectorXd& input)  override {
        // Throttle
        control_state.motor_drive_state.throttle = input(0);
        motor.set_controls(input.segment<1>(0));
        if (rparam.enable_cyclic_control) {
            control_state.u_lat = input(1); //Lat control, e.g. roll control from -1 to 1
            control_state.u_lon = input(2);//lon control, e.g. pitch control from -1 to 1
            if (!rparam.fixed_pitch) {
                control_state.u_collective = input(3);
                return 4;
            } else {
                return 3;
            }
        }
        return 1;
    }


    //STATE VECTOR
    //[OMG, MOTORRPM, A, B]
    virtual int get_state_dim()  const override {
        int motor_dim = motor.get_state_dim();
        return motor_dim + TPP_DIM;
    }
    
    int get_control_dim()  const override {
        if (!rparam.enable_cyclic_control) {
            return 1;
        }
        
        if (rparam.fixed_pitch) {
            return 3;
        } else {
            return 4;
        }
    }
    
    void set_state(const Eigen::VectorXd &x) override {
        motor.set_state(x.segment<MOTOR_STATE_DIM>(0));
        rotor_state.a = x(MOTOR_STATE_DIM);
        rotor_state.b = x(MOTOR_STATE_DIM+1);
    }

    virtual Eigen::VectorXd get_state() const override {
        Eigen::VectorXd x(get_state_dim());
        x.segment<MOTOR_STATE_DIM>(0) = motor.get_state();
        x(MOTOR_STATE_DIM) = rotor_state.a;
        x(MOTOR_STATE_DIM + 1) = rotor_state.b;
        return x;
    }

    void operator() (const KinematicsState & kstate, const Eigen::VectorXd &x, Eigen::VectorXd &dxdt) const override {
        //Compute dx/dt by x
        //Eq. (4.27) in From Seddon et. al.
        double p = kstate.omega.x();
        double q = - kstate.omega.y(); //The equ here is in NED. q need to be inverse

        double rpm_cur = motor.rpm_from_state(x.segment<MOTOR_STATE_DIM>(0));
        double a_cur = x(MOTOR_STATE_DIM);
        double b_cur = x(MOTOR_STATE_DIM+1);
        
        Eigen::VectorXd motor_dxdt(MOTOR_STATE_DIM);
        motor(kstate, x.segment<MOTOR_STATE_DIM>(0), motor_dxdt);
        dxdt.segment<MOTOR_STATE_DIM>(0) = motor_dxdt;

        double _Ab = Ab(rpm_cur);
        double _Ba = -_Ab;
        double _tau_f = tau_f(rpm_cur);

        double a_dot = (-a_cur - _tau_f*q + _Ab*b_cur + control_state.u_lon*rparam.Alonlat)/_tau_f;
        double b_dot = (-b_cur - _tau_f*p + _Ba*a_cur + control_state.u_lat*rparam.Alonlat)/_tau_f;
        dxdt(MOTOR_STATE_DIM) = a_dot;
        dxdt(MOTOR_STATE_DIM+1) = b_dot;

        if (std::isnan(dxdt.maxCoeff())) {
            std::cout << "x" << x.transpose() << std::endl;
            std::cout << "dx" << dxdt.transpose() << std::endl;
            printf("rpm_cur %f tau_f %f, ab_cur %f %f, _Ab %f, A %f u %f %f pq %f %f\n\n\n", 
                rpm_cur, _tau_f, a_cur, b_cur, _Ab, 
                rparam.Alonlat, control_state.u_lon, control_state.u_lat, p, q);
            exit(-1);
        }
    }

    double J(const KinematicsState & kstate, double rpm_cur) const {
        double v = kstate.v.dot(kstate.R*orientation*Eigen::Vector3d::UnitZ());
        return v / (rpm2rps(rpm_cur)*diameter());
    }

    //FLU
    Eigen::Vector3d get_force(const KinematicsState & kstate, const Eigen::VectorXd &x) const {
        double rpm_cur = motor.rpm_from_state(x.segment<MOTOR_STATE_DIM>(0));
        double T = thrust(rpm_cur, J(kstate, rpm_cur));
        double a_cur = x(MOTOR_STATE_DIM);
        double b_cur = x(MOTOR_STATE_DIM+1);
        //NED(')->FLU
        //Vx = Vx'
        //Vy = -Vy'
        //Vz = -Vz'
        //Eq. (4.31) in From Seddon et. al.
        return Eigen::Vector3d(-a_cur, -b_cur, 1)*T;
    }

    Eigen::Vector3d get_torque(const KinematicsState & kstate, const Eigen::VectorXd &x) const {
        double rpm_cur = motor.rpm_from_state(x.segment<MOTOR_STATE_DIM>(0));
        double T = thrust(rpm_cur, J(kstate, rpm_cur));
        double a_cur = x(MOTOR_STATE_DIM);
        double b_cur = x(MOTOR_STATE_DIM+1);
        
        //NED(')->FLU
        //Vx = Vx'
        //Vy = -Vy'
        //Vz = -Vz'
        //Eq. (4.31) in From Seddon et. al.
        Eigen::Vector3d torque_TPP(b_cur, -a_cur, 0);
        Eigen::Vector3d torque = torque_TPP*rparam.kB;

        double CQ = rparam.C_Q;
        double rho = eparam.rho;
        double n = rpm2rps(rpm_cur);
        double Q = CQ*rho*n*n*pow(diameter(),5)*rparam.clockwise; //Torque from rotor generated by drag.

        Eigen::VectorXd motor_dxdt(MOTOR_STATE_DIM);
        motor(kstate, x.segment<MOTOR_STATE_DIM>(0), 
            motor_dxdt);

         //Torque from rotor generated by angular acceleration. Not motor dstate/dt is RPM/s, but we need rad/s
        Q = Q + motor_dxdt(0)*rpm2radps(rparam.Irotorhead)*rparam.clockwise;
        torque.z() = torque.z() + Q;

        // printf("rpm %f Q %.2f ulat %.2f ulon %.2f a_cur %.2f b_cur %.2f, torque %.2f %.2f %.2f\n",
        //         rpm_cur, Q, control_state.u_lat, control_state.u_lon,
        //         a_cur*57.3, b_cur*57.3, torque.x(), torque.y(), torque.z());
        return torque;
    }

    inline double thrust(double rpm_cur, double J) const {
        double CT = rparam.C_T0 + rparam.k_CT*J;
        double rho = eparam.rho;
        double n = rpm2rps(rpm_cur);

        double thrust =  CT*rho*n*n*pow(diameter(), 4);
        return thrust;
    }
    
    //main rotor cross coupling term
    inline double Ab(const double & rpm_cur) const {
        double lmdb = lambda_b(rpm_cur);
        return 8*(lmdb*lmdb-1)/gamma();
    }

    //main rotor cross coupling term
    inline double Ba(const double & rpm_cur) const {
        return -Ab(rpm_cur);
    }

    //flapping frequency ratio
    inline double lambda_b (const double & rpm_cur) const {
        double Omg = rpm2radps(rpm_cur);
        return sqrt(rparam.kB/(Omg*Omg*rparam.Ib) + 1);
    }
    
    //main rotor time constant
    inline double tau_f(const double & rpm_cur) const {
        return 16.0/(rpm2radps(rpm_cur)*gamma());
    }

    //Lock number
    inline double gamma() const {
        return eparam.rho*rparam.cla*rparam.c_blade*pow(rparam.R, 4)/rparam.Ib;
    }

    inline double diameter() const {
        return 2*rparam.R;
    }

    inline double area() const {
        return M_PI*rparam.R*rparam.R;
    }

    virtual int print_controls(int start_index) const override {
        printf("CTRL AXIS %d: ROTOR THROTTLE %f\n", start_index, control_state.motor_drive_state.throttle);
        if (rparam.enable_cyclic_control) {
            printf("CTRL AXIS %d: ROTOR Cyclic Lat/Roll %f\n", start_index + 1, control_state.u_lat);
            printf("CTRL AXIS %d: ROTOR Cyclic Lon/Pitch %f\n", start_index + 2, control_state.u_lat);
            if (rparam.fixed_pitch) {
                return start_index + 3;
            } else {
                printf("CTRL AXIS %d: ROTOR Collective\n", start_index + 3);
                return start_index + 4;
            }

        }
        return start_index + 1;
    }

    virtual int print_states(int start_index) const override {
        printf("STATE AXIS %d: Rotor RPM %.2f\n", start_index, motor.get_state()(0));
        printf("CTRL AXIS %d: ROTOR Cyclic Lat/Roll %.2f\n", start_index + 1, rotor_state.a);
        printf("CTRL AXIS %d: ROTOR Cyclic Lon/Pitch %.2f\n", start_index + 2, rotor_state.b);
        return start_index + 3;
    }
};

#include <helicopter_simulator/ros_sim_common.hpp>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <helicopter_simulator/Helicopter.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <uav_utils/geometry_utils.h>
#include "helicopter_simulator/attitude_control.hpp"

using namespace Eigen;

typedef struct _Command
{
    Vector3d angular_velocity_sp = Vector3d::Zero();
    double thrust_sp = 0.0;
    Quaterniond target_quat;
} Command;

typedef struct _Disturbance
{
    Vector3d f;
    Vector3d m;
} Disturbance;

VectorXd control_angvel_naive(_Command command, const Helicopter &heli, double dt)
{
    double pRP_rate = 3;
    double pYaw_rate = 0.01;
    double pRP = 1.0;
    double Rud_mix = 0.2;
    const Helicopter::State state = heli.getState();
    auto err_rate = command.angular_velocity_sp - state.kstate.omega;
    double Ail = float_constrain(err_rate.x() * pRP_rate, -1., 1.);
    double Ele = float_constrain(err_rate.y() * pRP_rate, -1., 1.);
    double Rud = float_constrain(err_rate.z() * pYaw_rate, -1., 1.);
    double Thr = command.thrust_sp * 0.5;

    VectorXd controls;
    controls.resize(heli.get_control_dim());

    // For coaxial
    controls(0) = Thr + Rud * Rud_mix; // Throttle of lower rotor
    controls(1) = Ail;                 // Roll of cyclic
    controls(2) = -Ele;                // Pitch of cylic
    controls(3) = Thr - Rud * Rud_mix; // Throttle of upper rotor

    return controls;
}


class HeliSimulatorNode
{
protected:

    ros::Publisher odom_pub, imu_pub;
    ros::Subscriber f_sub, m_sub, j_sub;
    Helicopter _heli;
    ros::Timer timer;
    double current_yaw = 0;
    
    AttiControl control;

    double simulation_rate = 100.0;
    double odom_rate = 100.0;
    std::string quad_name;
    ros::Duration odom_pub_duration;

    double t_sim = 0;
    ros::Time t_start;
    int count = 0;
    
    int manual_ctrl_mode = 0; // 0 Rate/Thrust; 1 Atti/Thrust; 2 Atti/Alt.

    Command command;
    Disturbance disturbance;

    void joy_callback(const sensor_msgs::Joy cmd)
    {
        double max_angle = 30 / 57.3;

        // Joystick is AETR.
        command.target_quat =
            AngleAxisd(current_yaw, Vector3d::UnitZ()) *
            AngleAxisd(cmd.axes[1] * max_angle, Vector3d::UnitY()) *
            AngleAxisd(cmd.axes[0] * max_angle, Vector3d::UnitX());

        command.angular_velocity_sp.x() = cmd.axes[0] * M_PI;
        command.angular_velocity_sp.y() = cmd.axes[1] * M_PI;
        command.angular_velocity_sp.z() = -cmd.axes[3] * M_PI;
        command.thrust_sp = (cmd.axes[2] + 1) / 2;
    }

    void force_disturbance_callback(const geometry_msgs::Vector3::ConstPtr &f)
    {
        disturbance.f(0) = f->x;
        disturbance.f(1) = f->y;
        disturbance.f(2) = f->z;
    }

    void moment_disturbance_callback(const geometry_msgs::Vector3::ConstPtr &m)
    {
        disturbance.m(0) = m->x;
        disturbance.m(1) = m->y;
        disturbance.m(2) = m->z;
    }


    VectorXd control_ang(const Helicopter &heli, double dt)
    {
        auto state = heli.getState().kstate;
        control.setYawSpdSP(command.angular_velocity_sp.z());
        auto rate_sp = control.control_attitude(dt, command.target_quat, Eigen::Quaterniond(state.R));

        Eigen::Vector3d attctrl = control.control_attitude_rates(dt, rate_sp, state.omega);
        double Rud_mix = 0.2;

        VectorXd controls;
        controls.resize(heli.get_control_dim());
        double Thr = command.thrust_sp * 0.5;

        // For coaxial
        controls(0) = Thr + attctrl(2) * Rud_mix; // Throttle of lower rotor
        controls(1) = attctrl(0);                 // Roll of cyclic
        controls(2) = -attctrl(1);                // Pitch of cylic
        controls(3) = Thr - attctrl(2) * Rud_mix; // Throttle of upper rotor
        return controls;
    }

    VectorXd control_angvel(const Helicopter &heli, double dt)
    {
        auto state = heli.getState().kstate;
        Eigen::Vector3d attctrl = control.control_attitude_rates(dt, command.angular_velocity_sp, state.omega);
        double Rud_mix = 0.2;

        VectorXd controls;
        controls.resize(heli.get_control_dim());
        double Thr = command.thrust_sp * 0.5;

        // For coaxial
        controls(0) = Thr + attctrl(2) * Rud_mix; // Throttle of lower rotor
        controls(1) = attctrl(0);                 // Roll of cyclic
        controls(2) = -attctrl(1);                // Pitch of cylic
        controls(3) = Thr - attctrl(2) * Rud_mix; // Throttle of upper rotor
        return controls;
    }

    void init(ros::NodeHandle &n)
    {

        double _init_x, _init_y, _init_z;
        n.param("simulator/init_state_x", _init_x, 0.0);
        n.param("simulator/init_state_y", _init_y, 0.0);
        n.param("simulator/init_state_z", _init_z, 3.0);

        Vector3d position = Vector3d(_init_x, _init_y, _init_z);
        _heli.setStatePos(position);

        n.param("rate/simulation", simulation_rate, 1000.0);
        ROS_ASSERT(simulation_rate > 0);

        n.param("manual_ctrl_mode", manual_ctrl_mode, 0);

        n.param("rate/odom", odom_rate, 100.0);
        odom_pub_duration = ros::Duration(1 / odom_rate);

        n.param("quadrotor_name", quad_name, std::string("quadrotor"));

        ROS_INFO("[HeliSimulatorNode] start name %s sim_rate %f odom_rate %f", quad_name.c_str(), simulation_rate, odom_rate);

        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
        imu_pub = n.advertise<sensor_msgs::Imu>("imu", 10);
        f_sub = n.subscribe("force_disturbance", 100, &HeliSimulatorNode::force_disturbance_callback, this,
                            ros::TransportHints().tcpNoDelay());
        m_sub = n.subscribe("moment_disturbance", 100, &HeliSimulatorNode::moment_disturbance_callback, this,
                            ros::TransportHints().tcpNoDelay());
        j_sub = n.subscribe("/joy", 1, &HeliSimulatorNode::joy_callback, this,
                            ros::TransportHints().tcpNoDelay());

        t_start = ros::Time::now();
        timer = n.createTimer(ros::Duration(1.0/simulation_rate), &HeliSimulatorNode::timerCallback, this);
    }

public:
    HeliSimulatorNode(ros::NodeHandle &n) : _heli(CoaxialFixedPitchHelicopter)
    {
        init(n);
    }

    void timerCallback(const ros::TimerEvent &e)
    {
        double dt = 1.0/simulation_rate;
        t_sim += dt;

        ros::Time next_odom_pub_time = ros::Time::now();

        VectorXd _ctrls;
        if (manual_ctrl_mode == 0) {
            _ctrls = control_angvel(_heli, dt);
        } else if (manual_ctrl_mode == 1) {
            _ctrls = control_ang(_heli, dt);
        }

        _heli.setInput(_ctrls);
        _heli.setExternalForce(disturbance.f);
        _heli.setExternalMoment(disturbance.m);
        _heli.step(dt);

        auto state = _heli.getState();

        Eigen::Vector3d _ypr = R_to_ypr(state.kstate.R);
        current_yaw = _ypr(0);

        ros::Time tnow = ros::Time::now();

        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "/simulator";
        odom_msg.child_frame_id = "/" + quad_name;

        sensor_msgs::Imu imu;
        imu.header.frame_id = "/simulator";
        if (tnow >= next_odom_pub_time)
        {
            next_odom_pub_time += odom_pub_duration;
            odom_msg.header.stamp = tnow;
            stateToOdomMsg(state, odom_msg);
            quadToImuMsg(_heli, imu);
            odom_pub.publish(odom_msg);
            imu_pub.publish(imu);
        }

        if (count % ((int)simulation_rate) == 0) {
            printf("[HeliSimulatorNode] %d T Abs %.1f Sim %.1f\n", count, (ros::Time::now() - t_start).toSec(), t_sim);
        }
        count ++;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "helicopter_simulator_so3");

    ros::NodeHandle n("~");
    HeliSimulatorNode heli(n);
    ros::spin();

    return 0;
}

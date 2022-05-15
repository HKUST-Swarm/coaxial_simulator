#pragma once

#include <Eigen/Dense>

#define KINEMATICS_STATE_DIM 18
#define DIM_POS 3
#define DIM_VEL 3
#define DIM_R 9
#define DIM_OMG 3

struct EnvironmentParam {
    double rho = 1.225;
    double g = 9.805;
};

struct KinematicsState {
    Eigen::Vector3d x = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    Eigen::VectorXd internal_states;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename T>
T float_constrain(T v, T min, T max) {
    if (v > max) {
        return max;
    }
    if (v < min) {
        return min;
    }
    return v;
}

template<typename T>
int signNoZero(T val)
{
	return (T(0) <= val) - (val < T(0));
}

Eigen::Vector3d dcm_z(Eigen::Quaterniond q)
{
    return q.toRotationMatrix().block<3, 1>(0, 2);
}

Eigen::Quaterniond canonical(const Eigen::Quaterniond & q)
{
    if (q.w() < 0) {
        return Eigen::Quaterniond(-q.w(),-q.x(),-q.y(),-q.z());
    } else {
        return Eigen::Quaterniond(q.w(),q.x(),q.y(),q.z());
    }

    return q;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> R_to_ypr(const Eigen::DenseBase<Derived>& R) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
    EIGEN_STATIC_ASSERT(Derived::RowsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
    EIGEN_STATIC_ASSERT(Derived::ColsAtCompileTime == 3, THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

    Eigen::Matrix<typename Derived::Scalar, 3, 1> n = R.col(0);
    Eigen::Matrix<typename Derived::Scalar, 3, 1> o = R.col(1);
    Eigen::Matrix<typename Derived::Scalar, 3, 1> a = R.col(2);

    Eigen::Matrix<typename Derived::Scalar, 3, 1> ypr(3);
    typename Derived::Scalar y = atan2(n(1), n(0));
    typename Derived::Scalar p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    typename Derived::Scalar r =
        atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr;
}

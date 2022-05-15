#include <Eigen/Dense>
#include "common.hpp"

#define AXIS_INDEX_ROLL
class AttiControl
{

protected:
    Eigen::Vector3d _rates_prev;          /**< angular rates on previous step */
    Eigen::Vector3d _rates_prev_filtered; /**< angular rates on previous step (low-pass filtered) */
    Eigen::Vector3d _rates_int;           /**< angular rates integral error */
    Eigen::Vector3d _att_control;         /**< attitude control vector */

    Eigen::Vector3d _attitude_p; /**< gain for attitude */
    Eigen::Vector3d _rate_p; /**< gain for rate */
    Eigen::Vector3d _rate_i; /**< gain for rate */
    Eigen::Vector3d _rate_d; /**< gain for rate */

    Eigen::Vector3d _rate_int_lim;

    Eigen::Vector3d _rate_ff;

    float _yaw_w = 0;
    float _yawspeed_setpoint = 0.4;
public:
    AttiControl():
        _rates_prev(0, 0, 0),
        _rates_prev_filtered(0, 0, 0),
        _rates_int(0, 0, 0),
        _att_control(0, 0, 0),
        _attitude_p(2, 2, 1),
        _rate_p(0.5, 0.5, 0.01),
        _rate_i(0.1, 0.1, 0.05),
        _rate_d(0.0, 0.0, 0.0),
        _rate_int_lim(0.2, 0.2, 0.1), 
        _rate_ff(0, 0, 0)
    {
    }

    void setYawSpdSP(double yawspd_sp) {
        _yawspeed_setpoint = yawspd_sp;
    }

    Eigen::Vector3d control_attitude(double dt, Eigen::Quaterniond qd, Eigen::Quaterniond q_cur)
    {

        // calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
        auto e_z = dcm_z(q_cur);
        auto e_z_d = dcm_z(qd);
        auto qd_red = Eigen::Quaterniond::FromTwoVectors(e_z, e_z_d);

        if (fabsf(qd_red.x()) > (1.f - 1e-5f) || fabsf(qd_red.y()) > (1.f - 1e-5f))
        {
            // In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
            // full attitude control anyways generates no yaw input and directly takes the combination of
            // roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
            qd_red = qd;
        }
        else
        {
            // transform rotation from current to desired thrust vector into a world frame reduced desired attitude
            qd_red *= q_cur;
        }

        // mix full and reduced desired attitude
        auto q_mix = qd_red.inverse() * qd;
        q_mix = canonical(q_mix);
        // catch numerical problems with the domain of acosf and asinf
        q_mix.x() = float_constrain(q_mix.x(), -1., 1.);
        q_mix.z() = float_constrain(q_mix.z(), -1., 1.);
        qd = qd_red * Eigen::Quaterniond(cosf(_yaw_w * acosf(q_mix.x())), 0, 0, sinf(_yaw_w * asinf(q_mix.z())));

        // quaternion attitude control law, qe is rotation from q to qd
        auto qe = q_cur.inverse() * qd;

        // using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
        // also taking care of the antipodal unit quaternion ambiguity
        Eigen::Vector3d eq = 2.f * canonical(qe).coeffs().block<3, 1>(0, 0);

        // calculate angular rates setpoint
        Eigen::Vector3d rate_setpoint = eq.array() * _attitude_p.array();

        // Feed forward the yaw setpoint rate.
        // yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
        // but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
        // Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
        // and multiply it by the yaw setpoint rate (yawspeed_setpoint).
        // This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
        // such that it can be added to the rates setpoint.
        rate_setpoint += dcm_z(q_cur.inverse()) * _yawspeed_setpoint;

        return rate_setpoint;
    }

    Eigen::Vector3d control_attitude_rates(float dt, Eigen::Vector3d _rates_sp, Eigen::Vector3d rates)
    {
        Eigen::Vector3d rates_p_scaled = _rate_p;
        Eigen::Vector3d rates_i_scaled = _rate_i;
        Eigen::Vector3d rates_d_scaled = _rate_d;

        /* angular rates error */
        Eigen::Vector3d rates_err = _rates_sp - rates;
        Eigen::Vector3d rates_filtered = rates;

        /* apply low-pass filtering to the rates for D-term */


        _att_control = rates_p_scaled.cwiseProduct(rates_err) +
                    _rates_int +
                    rates_d_scaled.cwiseProduct(rates_filtered - _rates_prev_filtered) / dt +
                    _rate_ff.cwiseProduct(_rates_sp);

        _rates_prev = rates;
        _rates_prev_filtered = rates;

        for (int i = 0; i < 3; i++) {
            // Perform the integration using a first order method and do not propagate the result if out of range or invalid
            float rate_i = _rates_int(i) + rates_i_scaled(i) * rates_err(i) * dt;

            if (rate_i > -_rate_int_lim(i) && rate_i < _rate_int_lim(i)) {
                _rates_int(i) = rate_i;

            }
            _rates_int(i) = float_constrain(_rates_int(i), -_rate_int_lim(i), _rate_int_lim(i));

            _att_control(i) = float_constrain(_att_control(i), -1., 1.);
        }

        return _att_control;
    }



};
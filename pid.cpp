#include "pid.h"
#include <algorithm>

PID::PID() = default;
PID::~PID() = default;

void PID::set_gains(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::set_setpoint(double r)
{
    r_ = r;
}

void PID::set_output_limits(double umin, double umax)
{
    if (umin > umax)
    {
        std::swap(umin, umax);
    }
    umin_ = umin;
    umax_ = umax;
}

void PID::set_derivative_filter(double alpha)
{
    // Clamp alpha to [0,1]
    if (alpha < 0.0)
        alpha = 0.0;
    if (alpha > 1.0)
        alpha = 1.0;
    alpha_ = alpha;
}

void PID::reset()
{
    integ_ = 0.0;
    y_prev_ = 0.0;
    d_prev_ = 0.0;
    u_prev_ = 0.0;
    first_ = true;
}

double PID::update(double y, double dt)
{
    if (dt <= 0.0)
    {
        // Do nothing if dt invalid; return last output.
        return u_prev_;
    }

    const double error = r_ - y;

    // Proportional
    const double P = kp_ * error;

    // Integral with clamping anti-windup
    integ_ += ki_ * error * dt;

    // Derivative on measurement with optional filtering
    double d_meas = 0.0;
    if (first_)
    {
        d_meas = 0.0;
    }
    else
    {
        d_meas = (y_prev_ - y) / dt; // negative sign for derivative on measurement
    }
    double D_unfiltered = kd_ * d_meas;
    double D = alpha_ * D_unfiltered + (1.0 - alpha_) * d_prev_;

    // Unsaturated output
    double u = P + integ_ + D;

    // Saturation
    double u_sat = std::clamp(u, umin_, umax_);

    // Anti-windup: clamp integrator if control saturates in same direction
    if (u != u_sat)
    {
        // Back-calculation simple clamp: adjust integrator so that P + I + D == u_sat
        // I_new = u_sat - (P + D)
        integ_ = u_sat - (P + D);
    }

    // Save states
    y_prev_ = y;
    d_prev_ = D;
    u_prev_ = u_sat;
    first_ = false;

    return u_sat;
}

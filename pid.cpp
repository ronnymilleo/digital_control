#include "pid.h"
#include "exceptions.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <utility>

namespace DigitalControl
{

PID::PID() = default;

PID::PID(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd)
{
    if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd))
    {
        throw InvalidParameterException("PID gains must be finite numbers");
    }
    if (kp < 0.0 || ki < 0.0 || kd < 0.0)
    {
        throw InvalidParameterException("PID gains must be non-negative");
    }
}

PID::~PID() = default;

// Move constructor
PID::PID(PID &&other) noexcept
{
    swap(other);
}

// Move assignment operator
PID &PID::operator=(PID &&other) noexcept
{
    if (this != &other)
    {
        PID temp(std::move(other));
        swap(temp);
    }
    return *this;
}

void PID::swap(PID &other) noexcept
{
    std::swap(kp_, other.kp_);
    std::swap(ki_, other.ki_);
    std::swap(kd_, other.kd_);
    std::swap(r_, other.r_);
    std::swap(integ_, other.integ_);
    std::swap(y_prev_, other.y_prev_);
    std::swap(d_prev_, other.d_prev_);
    std::swap(u_prev_, other.u_prev_);
    std::swap(umin_, other.umin_);
    std::swap(umax_, other.umax_);
    std::swap(alpha_, other.alpha_);
    std::swap(first_, other.first_);
}

void PID::set_gains(double kp, double ki, double kd)
{
    if (!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd))
    {
        throw InvalidParameterException("PID gains must be finite numbers");
    }
    if (kp < 0.0 || ki < 0.0 || kd < 0.0)
    {
        throw InvalidParameterException("PID gains must be non-negative");
    }
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::set_setpoint(double r)
{
    if (!std::isfinite(r))
    {
        throw InvalidParameterException("Setpoint must be a finite number");
    }
    r_ = r;
}

void PID::set_output_limits(double umin, double umax)
{
    if (!std::isfinite(umin) || !std::isfinite(umax))
    {
        throw InvalidParameterException("Output limits must be finite numbers");
    }
    if (umin > umax)
    {
        std::swap(umin, umax);
    }
    umin_ = umin;
    umax_ = umax;
}

void PID::set_derivative_filter(double alpha)
{
    if (!std::isfinite(alpha))
    {
        throw InvalidParameterException("Derivative filter alpha must be a finite number");
    }
    // Clamp alpha to [0,1]
    alpha_ = std::clamp(alpha, 0.0, 1.0);
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
    if (!std::isfinite(y))
    {
        throw InvalidParameterException("Measurement y must be a finite number");
    }
    if (!std::isfinite(dt) || dt < 0.0)
    {
        throw InvalidParameterException("Time step dt must be a finite non-negative number");
    }

    if (dt <= std::numeric_limits<double>::epsilon())
    {
        // Do nothing if dt is effectively zero; return last output.
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
    const double D_unfiltered = kd_ * d_meas;
    const double D = alpha_ * D_unfiltered + (1.0 - alpha_) * d_prev_;

    // Unsaturated output
    double u = P + integ_ + D;

    // Check for numerical issues
    if (!std::isfinite(u))
    {
        throw NumericalException("PID computation resulted in non-finite control output");
    }

    // Saturation
    const double u_sat = std::clamp(u, umin_, umax_);

    // Anti-windup: clamp integrator if control saturates in same direction
    if (std::abs(u - u_sat) > std::numeric_limits<double>::epsilon())
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

std::ostream &operator<<(std::ostream &os, const PID &pid)
{
    os << "PID{kp=" << pid.kp_ << ", ki=" << pid.ki_ << ", kd=" << pid.kd_ << ", r=" << pid.r_
       << ", u_prev=" << pid.u_prev_ << ", limits=[" << pid.umin_ << ", " << pid.umax_ << "]"
       << ", alpha=" << pid.alpha_ << "}";
    return os;
}

} // namespace DigitalControl

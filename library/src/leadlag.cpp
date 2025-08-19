#include "leadlag.h"
#include "exceptions.h"
#include <cmath>
#include <iostream>
#include <limits>

namespace DigitalControl
{

LeadLag::LeadLag() : LeadLag(1.0, 0.1, 1.0)
{
}

LeadLag::LeadLag(double K, double zero, double pole) : K_(K), z_(zero), p_(pole)
{
    if (!std::isfinite(K) || !std::isfinite(zero) || !std::isfinite(pole))
    {
        throw InvalidParameterException("LeadLag parameters must be finite");
    }
    if (zero <= 0.0 || pole <= 0.0)
    {
        throw InvalidParameterException("LeadLag zero and pole must be positive");
    }
    if (std::abs(K) < std::numeric_limits<double>::epsilon())
    {
        throw InvalidParameterException("LeadLag gain cannot be zero");
    }
}

LeadLag::LeadLag(LeadLag&& other) noexcept
{
    swap(other);
}

LeadLag& LeadLag::operator=(LeadLag&& other) noexcept
{
    if (this != &other)
    {
        swap(other);
    }
    return *this;
}

void LeadLag::swap(LeadLag& other) noexcept
{
    std::swap(K_, other.K_);
    std::swap(z_, other.z_);
    std::swap(p_, other.p_);
    std::swap(b0_, other.b0_);
    std::swap(b1_, other.b1_);
    std::swap(a0_, other.a0_);
    std::swap(a1_, other.a1_);
    std::swap(r_, other.r_);
    std::swap(e_prev_, other.e_prev_);
    std::swap(u_prev_, other.u_prev_);
    std::swap(u_sat_prev_, other.u_sat_prev_);
    std::swap(umin_, other.umin_);
    std::swap(umax_, other.umax_);
    std::swap(first_, other.first_);
    std::swap(last_dt_, other.last_dt_);
}

void LeadLag::set_parameters(double K, double zero, double pole)
{
    if (!std::isfinite(K) || !std::isfinite(zero) || !std::isfinite(pole))
    {
        throw InvalidParameterException("LeadLag parameters must be finite");
    }
    if (zero <= 0.0 || pole <= 0.0)
    {
        throw InvalidParameterException("LeadLag zero and pole must be positive");
    }
    if (std::abs(K) < std::numeric_limits<double>::epsilon())
    {
        throw InvalidParameterException("LeadLag gain cannot be zero");
    }

    K_ = K;
    z_ = zero;
    p_ = pole;

    // Force recomputation of discrete coefficients on next update
    last_dt_ = 0.0;
}

void LeadLag::set_setpoint(double r)
{
    if (!std::isfinite(r))
    {
        throw InvalidParameterException("Setpoint must be finite");
    }
    r_ = r;
}

void LeadLag::set_output_limits(double umin, double umax)
{
    if (!std::isfinite(umin) || !std::isfinite(umax))
    {
        throw InvalidParameterException("Output limits must be finite");
    }
    if (umin >= umax)
    {
        throw InvalidParameterException("Lower limit must be less than upper limit");
    }
    umin_ = umin;
    umax_ = umax;
}

void LeadLag::compute_discrete_coefficients(double dt)
{
    // Using Tustin's method (bilinear transform)
    // s = (2/dt) * (z-1)/(z+1)
    //
    // Continuous: H(s) = K * (s + z) / (s + p)
    // Discrete:   H(z) = (b0 + b1*z^-1) / (a0 + a1*z^-1)

    const double T  = dt;
    const double T2 = 2.0 / T;

    // Pre-warp the frequencies (optional, but improves accuracy)
    // For simplicity, we'll use basic Tustin without pre-warping

    // Numerator: K * (s + z) -> K * (T2*(z-1)/(z+1) + z)
    // After bilinear transform and simplification:
    b0_ = K_ * (T2 + z_);
    b1_ = K_ * (-T2 + z_);

    // Denominator: (s + p) -> (T2*(z-1)/(z+1) + p)
    // After bilinear transform and simplification:
    a0_ = T2 + p_;
    a1_ = -T2 + p_;

    // Normalize by a0
    b0_ /= a0_;
    b1_ /= a0_;
    a1_ /= a0_;
    a0_ = 1.0;
}

double LeadLag::update(double y, double dt)
{
    if (!std::isfinite(y) || !std::isfinite(dt))
    {
        throw InvalidParameterException("Controller inputs must be finite");
    }
    if (dt <= 0.0)
    {
        throw InvalidParameterException("Time step must be positive");
    }

    // Recompute discrete coefficients if dt changed
    if (std::abs(dt - last_dt_) > std::numeric_limits<double>::epsilon() || first_)
    {
        compute_discrete_coefficients(dt);
        last_dt_ = dt;
    }

    // Calculate error
    const double e = r_ - y;

    // Discrete-time difference equation:
    // u[k] = b0*e[k] + b1*e[k-1] - a1*u[k-1]
    double u = b0_ * e + b1_ * e_prev_ - a1_ * u_prev_;

    // Store unsaturated output for next iteration
    u_prev_ = u;

    // Apply saturation
    u           = saturate(u);
    u_sat_prev_ = u;

    // Update state
    e_prev_ = e;
    first_  = false;

    if (!std::isfinite(u))
    {
        throw NumericalException("Controller computation resulted in non-finite output");
    }

    return u;
}

void LeadLag::reset()
{
    e_prev_     = 0.0;
    u_prev_     = 0.0;
    u_sat_prev_ = 0.0;
    first_      = true;
    last_dt_    = 0.0;
}

double LeadLag::saturate(double u) const
{
    if (u > umax_)
    {
        return umax_;
    }
    else if (u < umin_)
    {
        return umin_;
    }
    else
    {
        return u;
    }
}

double LeadLag::get_phase_at_frequency(double omega) const
{
    if (omega <= 0.0)
    {
        throw InvalidParameterException("Frequency must be positive");
    }

    // Phase of (jw + z) / (jw + p)
    // = atan(w/z) - atan(w/p)
    const double phase_zero = std::atan(omega / z_);
    const double phase_pole = std::atan(omega / p_);
    return phase_zero - phase_pole;
}

double LeadLag::get_magnitude_at_frequency(double omega) const
{
    if (omega <= 0.0)
    {
        throw InvalidParameterException("Frequency must be positive");
    }

    // Magnitude of K * |jw + z| / |jw + p|
    const double mag_zero = std::sqrt(omega * omega + z_ * z_);
    const double mag_pole = std::sqrt(omega * omega + p_ * p_);
    return K_ * mag_zero / mag_pole;
}

std::ostream& operator<<(std::ostream& os, const LeadLag& comp)
{
    os << "LeadLag(K=" << comp.K_ << ", zero=" << comp.z_ << ", pole=" << comp.p_
       << ", type=" << (comp.is_lead() ? "lead" : (comp.is_lag() ? "lag" : "unity")) << ", limits=[" << comp.umin_
       << ", " << comp.umax_ << "])";
    return os;
}

} // namespace DigitalControl

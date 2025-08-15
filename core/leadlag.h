#pragma once

#include "controller.h"
#include "exceptions.h"
#include <iosfwd>
#include <memory>

namespace DigitalControl
{

/**
 * Lead/Lag Compensator implementation
 *
 * Implements the transfer function: K * (s + z) / (s + p)
 * - Lead compensator: z < p (adds phase lead, increases bandwidth)
 * - Lag compensator: z > p (adds phase lag, improves steady-state error)
 *
 * Discrete implementation using Tustin's method (bilinear transform)
 */
class LeadLag : public Controller
{
  public:
    // Constructors and destructor
    LeadLag();
    explicit LeadLag(double K, double zero, double pole);
    ~LeadLag() override = default;

    // Move semantics
    LeadLag(LeadLag &&other) noexcept;
    LeadLag &operator=(LeadLag &&other) noexcept;

    // Delete copy semantics
    LeadLag(const LeadLag &) = delete;
    LeadLag &operator=(const LeadLag &) = delete;

    // Configuration methods
    void set_parameters(double K, double zero, double pole);
    void set_setpoint(double r) override;
    void set_output_limits(double umin, double umax);

    // Control update
    double update(double y, double dt) override;
    void reset() override;

    // Getters
    double get_setpoint() const override
    {
        return r_;
    }
    double get_last_output() const override
    {
        return u_prev_;
    }
    double get_gain() const
    {
        return K_;
    }
    double get_zero() const
    {
        return z_;
    }
    double get_pole() const
    {
        return p_;
    }
    double get_min_output() const
    {
        return umin_;
    }
    double get_max_output() const
    {
        return umax_;
    }

    // Type checking
    bool is_lead() const
    {
        return z_ < p_;
    }
    bool is_lag() const
    {
        return z_ > p_;
    }

    // Get phase contribution at frequency omega (rad/s)
    double get_phase_at_frequency(double omega) const;

    // Get magnitude contribution at frequency omega (rad/s)
    double get_magnitude_at_frequency(double omega) const;

    // Stream output for debugging
    friend std::ostream &operator<<(std::ostream &os, const LeadLag &comp);

  private:
    // Parameters
    double K_ = 1.0; // DC gain
    double z_ = 0.1; // Zero location (rad/s)
    double p_ = 1.0; // Pole location (rad/s)

    // Discrete-time coefficients (computed from continuous parameters)
    double b0_ = 0.0; // Numerator coefficients
    double b1_ = 0.0;
    double a0_ = 1.0; // Denominator coefficients
    double a1_ = 0.0;

    // State
    double r_ = 0.0;          // setpoint
    double e_prev_ = 0.0;     // previous error
    double u_prev_ = 0.0;     // previous output (before saturation)
    double u_sat_prev_ = 0.0; // previous saturated output

    // Limits
    double umin_ = -1e9;
    double umax_ = 1e9;

    bool first_ = true;
    double last_dt_ = 0.0; // Store last dt to detect changes

    void compute_discrete_coefficients(double dt);
    void swap(LeadLag &other) noexcept;
    double saturate(double u) const;
};

// Factory functions
inline std::unique_ptr<LeadLag> make_leadlag(double K = 1.0, double zero = 0.1, double pole = 1.0)
{
    return std::make_unique<LeadLag>(K, zero, pole);
}

inline std::unique_ptr<LeadLag> make_lead_compensator(double K = 1.0, double zero = 0.1, double pole = 1.0)
{
    // Ensure it's a lead compensator (zero < pole)
    if (zero >= pole)
    {
        throw InvalidParameterException("Lead compensator requires zero < pole");
    }
    return std::make_unique<LeadLag>(K, zero, pole);
}

inline std::unique_ptr<LeadLag> make_lag_compensator(double K = 1.0, double zero = 1.0, double pole = 0.1)
{
    // Ensure it's a lag compensator (zero > pole)
    if (zero <= pole)
    {
        throw InvalidParameterException("Lag compensator requires zero > pole");
    }
    return std::make_unique<LeadLag>(K, zero, pole);
}

} // namespace DigitalControl

#pragma once

#include "controller.h"
#include <iosfwd>
#include <memory>

namespace DigitalControl
{

/**
 * PID Controller implementation with anti-windup and derivative filtering
 */
class PID : public Controller
{
  public:
    // Constructors and destructor
    PID();
    explicit PID(double kp, double ki, double kd);
    ~PID() override;

    // Move semantics
    PID(PID &&other) noexcept;
    PID &operator=(PID &&other) noexcept;

    // Delete copy semantics (controllers should not be copied casually)
    PID(const PID &) = delete;
    PID &operator=(const PID &) = delete;

    // Configuration methods
    void set_gains(double kp, double ki, double kd);
    void set_setpoint(double r) override;
    void set_output_limits(double umin, double umax);
    void set_derivative_filter(double alpha); // 0..1, 1 = no filtering

    // Control update
    [[nodiscard]] double update(double y, double dt) override; // returns u
    void reset() override;

    // Getters
    [[nodiscard]] double get_setpoint() const override
    {
        return r_;
    }
    [[nodiscard]] double get_last_output() const override
    {
        return u_prev_;
    }
    [[nodiscard]] double get_kp() const
    {
        return kp_;
    }
    [[nodiscard]] double get_ki() const
    {
        return ki_;
    }
    [[nodiscard]] double get_kd() const
    {
        return kd_;
    }
    [[nodiscard]] double get_integral_term() const
    {
        return integ_;
    }
    [[nodiscard]] double get_min_output() const
    {
        return umin_;
    }
    [[nodiscard]] double get_max_output() const
    {
        return umax_;
    }
    [[nodiscard]] double get_derivative_filter() const
    {
        return alpha_;
    }

    // Stream output for debugging
    friend std::ostream &operator<<(std::ostream &os, const PID &pid);

  private:
    // Gains
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;

    // State
    double r_ = 0.0;      // setpoint
    double integ_ = 0.0;  // integral term state
    double y_prev_ = 0.0; // previous measurement for derivative
    double d_prev_ = 0.0; // filtered derivative state
    double u_prev_ = 0.0; // last saturated output

    // Config
    double umin_ = -1e9;
    double umax_ = 1e9;
    double alpha_ = 1.0; // derivative filter blending factor

    bool first_ = true;

    void swap(PID &other) noexcept;
};

// Factory function
[[nodiscard]] inline std::unique_ptr<PID> make_pid(double kp = 0.0, double ki = 0.0, double kd = 0.0)
{
    return std::make_unique<PID>(kp, ki, kd);
}

} // namespace DigitalControl

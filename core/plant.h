#pragma once

#include "exceptions.h"
#include <cmath>
#include <iosfwd>
#include <limits>
#include <memory>

namespace DigitalControl
{

/**
 * Generic plant interface
 */
struct IPlant
{
    virtual ~IPlant() = default;

    // Advance the plant one step given input u and timestep dt; returns new output y
    virtual double step(double u, double dt) = 0;

    // Get current output without advancing the state
    virtual double get_output() const = 0;

    // Reset plant to initial conditions
    virtual void reset() = 0;
};

/**
 * First-order plant: y' = (-(y - K*u))/tau
 */
class FirstOrderPlant : public IPlant
{
  public:
    constexpr explicit FirstOrderPlant(double K = 1.0, double tau = 1.0, double y0 = 0.0)
        : K_(K), tau_(tau), y0_(y0), y_(y0)
    {
    }

    double step(double u, double dt) override
    {
        if (!std::isfinite(u) || !std::isfinite(dt))
        {
            throw InvalidParameterException("Plant inputs must be finite");
        }
        if (dt < 0.0)
        {
            throw InvalidParameterException("Time step must be non-negative");
        }
        if (dt <= std::numeric_limits<double>::epsilon())
            return y_;

        const double dydt = (-(y_ - K_ * u)) / tau_;
        y_ += dt * dydt;

        if (!std::isfinite(y_))
        {
            throw NumericalException("Plant computation resulted in non-finite output");
        }
        return y_;
    }

    double get_output() const override
    {
        return y_;
    }

    void reset() override
    {
        y_ = y0_;
    }

    // Getters
    constexpr double get_gain() const
    {
        return K_;
    }
    constexpr double get_time_constant() const
    {
        return tau_;
    }
    constexpr double get_initial_value() const
    {
        return y0_;
    }

    friend std::ostream &operator<<(std::ostream &os, const FirstOrderPlant &plant);

  private:
    double K_;
    double tau_;
    double y0_;
    double y_;
};

/**
 * Second-order mass-spring-damper plant:
 * m * x_dd + b * x_d + k * x = K * u
 * Output y = x
 */
class SecondOrderPlant : public IPlant
{
  public:
    constexpr explicit SecondOrderPlant(double m = 1.0, double b = 0.5, double k = 4.0, double K = 1.0, double x0 = 0.0,
                                        double v0 = 0.0)
        : m_(m), b_(b), k_(k), K_(K), x0_(x0), v0_(v0), x_(x0), v_(v0)
    {
    }

    double step(double u, double dt) override
    {
        if (!std::isfinite(u) || !std::isfinite(dt))
        {
            throw InvalidParameterException("Plant inputs must be finite");
        }
        if (dt < 0.0)
        {
            throw InvalidParameterException("Time step must be non-negative");
        }
        if (dt <= std::numeric_limits<double>::epsilon())
            return x_;

        // x_dd = (K*u - b*v - k*x) / m
        const double a = (K_ * u - b_ * v_ - k_ * x_) / m_;

        // Integrate (semi-implicit Euler for a bit more stability):
        v_ += dt * a;
        x_ += dt * v_;

        if (!std::isfinite(x_) || !std::isfinite(v_))
        {
            throw NumericalException("Plant computation resulted in non-finite state");
        }
        return x_;
    }

    double get_output() const override
    {
        return x_;
    }

    void reset() override
    {
        x_ = x0_;
        v_ = v0_;
    }

    // Getters
    constexpr double get_mass() const
    {
        return m_;
    }
    constexpr double get_damping() const
    {
        return b_;
    }
    constexpr double get_stiffness() const
    {
        return k_;
    }
    constexpr double get_gain() const
    {
        return K_;
    }
    double get_position() const
    {
        return x_;
    }
    double get_velocity() const
    {
        return v_;
    }

    friend std::ostream &operator<<(std::ostream &os, const SecondOrderPlant &plant);

  private:
    double m_;
    double b_;
    double k_;
    double K_;
    double x0_;
    double v0_;
    double x_; // position (output)
    double v_; // velocity
};

// Factory functions
inline std::unique_ptr<FirstOrderPlant> make_first_order_plant(double K = 1.0, double tau = 1.0, double y0 = 0.0)
{
    return std::make_unique<FirstOrderPlant>(K, tau, y0);
}

inline std::unique_ptr<SecondOrderPlant> make_second_order_plant(double m = 1.0, double b = 0.5, double k = 4.0,
                                                                 double K = 1.0, double x0 = 0.0, double v0 = 0.0)
{
    return std::make_unique<SecondOrderPlant>(m, b, k, K, x0, v0);
}

} // namespace DigitalControl

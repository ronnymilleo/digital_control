#pragma once

namespace DigitalControl
{

/**
 * Abstract base class for digital controllers
 */
class Controller
{
  public:
    virtual ~Controller() = default;

    // Set the desired reference/setpoint r[k]
    virtual void set_setpoint(double r) = 0;

    // Update the controller with the current measurement y[k] and sample time dt.
    // Returns the control signal u[k].
    [[nodiscard]] virtual double update(double y, double dt) = 0;

    // Reset internal state (integrator, filters, etc.).
    virtual void reset() = 0;

    // Get current setpoint
    [[nodiscard]] virtual double get_setpoint() const = 0;

    // Get last control output
    [[nodiscard]] virtual double get_last_output() const = 0;
};

} // namespace DigitalControl

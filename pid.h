#pragma once

#include "controller.h"

class PID : public Controller {
public:
    PID();
    ~PID();

    void set_gains(double kp, double ki, double kd);
    void set_setpoint(double r) override;
    void set_output_limits(double umin, double umax);
    void set_derivative_filter(double alpha); // 0..1, 1 = no filtering

    double update(double y, double dt) override; // returns u
    void reset() override;

private:
    // Gains
    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;

    // State
    double r_ = 0.0;        // setpoint
    double integ_ = 0.0;    // integral term state
    double y_prev_ = 0.0;   // previous measurement for derivative
    double d_prev_ = 0.0;   // filtered derivative state
    double u_prev_ = 0.0;   // last saturated output

    // Config
    double umin_ = -1e9;
    double umax_ = 1e9;
    double alpha_ = 1.0;    // derivative filter blending factor

    bool first_ = true;
};

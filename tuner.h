#pragma once

#include <functional>
#include <iosfwd>
#include <memory>

#include "plant.h"

namespace DigitalControl
{

struct TuneResult
{
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double score{0.0};

    // Stream output
    friend std::ostream &operator<<(std::ostream &os, const TuneResult &result);
};

// Factory concept: callable that returns a fresh plant instance pointer with the same initial conditions.
// Signature: std::unique_ptr<IPlant> factory()

TuneResult auto_tune_pid_step_default(const std::function<std::unique_ptr<IPlant>()> &plant_factory, double r,
                                      double dt, double t_end, double umin, double umax, double alpha = 0.7,
                                      int kp_points = 8, int ki_points = 6, int kd_points = 5);

} // namespace DigitalControl

#include "tuner.h"
#include "pid.h"

#include <algorithm>
#include <cmath>
#include <limits>

static double simulate_score(IPlant &plant, PID &pid, double r, double dt, double t_end)
{
    pid.reset();
    pid.set_setpoint(r);

    double y = 0.0;   // assume plant returns its internal y on step
    double iae = 0.0; // integral of absolute error
    for (double t = 0.0; t <= t_end + 1e-12; t += dt)
    {
        double u = pid.update(y, dt);
        y = plant.step(u, dt);
        iae += std::abs(r - y) * dt;
    }
    return iae;
}

TuneResult auto_tune_pid_step_default(const std::function<std::unique_ptr<IPlant>()> &plant_factory, double r,
                                      double dt, double t_end, double umin, double umax, double alpha, int kp_points,
                                      int ki_points, int kd_points)
{

    // Define coarse ranges based on simple heuristics
    const double kp_min = 0.1, kp_max = 20.0;
    const double ki_min = 0.0, ki_max = 10.0;
    const double kd_min = 0.0, kd_max = 5.0;

    TuneResult best{0, 0, 0, std::numeric_limits<double>::infinity()};

    for (int i = 0; i < kp_points; ++i)
    {
        const double denom_kp = std::max(1.0, static_cast<double>(kp_points - 1));
        double kp = kp_min + (kp_max - kp_min) * (static_cast<double>(i) / denom_kp);
        for (int j = 0; j < ki_points; ++j)
        {
            const double denom_ki = std::max(1.0, static_cast<double>(ki_points - 1));
            double ki = ki_min + (ki_max - ki_min) * (static_cast<double>(j) / denom_ki);
            for (int k = 0; k < kd_points; ++k)
            {
                const double denom_kd = std::max(1.0, static_cast<double>(kd_points - 1));
                double kd = kd_min + (kd_max - kd_min) * (static_cast<double>(k) / denom_kd);

                auto plant = plant_factory();
                PID pid;
                pid.set_gains(kp, ki, kd);
                pid.set_output_limits(umin, umax);
                pid.set_derivative_filter(alpha);

                double score = simulate_score(*plant, pid, r, dt, t_end);
                if (score < best.score)
                {
                    best = {kp, ki, kd, score};
                }
            }
        }
    }

    return best;
}

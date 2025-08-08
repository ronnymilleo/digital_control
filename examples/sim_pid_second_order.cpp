#include <fstream>
#include <iomanip>
#include <iostream>

#include "pid.h"
#include "plant.h"

int main()
{
    // Simulation parameters
    const double dt = 0.001;   // small dt for second-order dynamics
    const double t_end = 10.0; // 10 seconds
    const double r_step = 1.0; // desired setpoint (position)

    // Controller setup
    PID pid;
    // Slightly higher damping with D, modest I
    pid.set_gains(8.0, 2.0, 1.5);
    pid.set_output_limits(-50.0, 50.0);
    pid.set_derivative_filter(0.7);
    pid.set_setpoint(r_step);

    // Plant: mass-spring-damper (m, b, k, K, x0, v0)
    SecondOrderPlant plant(1.0, 1.5, 8.0, 1.0, 0.0, 0.0);

    std::ofstream csv("sim_pid_second_order.csv");
    csv << "t,r,y,u\n";

    double y = 0.0; // position output

    for (double t = 0.0; t <= t_end + 1e-12; t += dt)
    {
        double u = pid.update(y, dt);
        y = plant.step(u, dt);
        csv << std::fixed << std::setprecision(6) << t << "," << r_step << "," << y << "," << u << "\n";
    }

    std::cout << "Second-order simulation complete. Output written to sim_pid_second_order.csv\n";
    return 0;
}

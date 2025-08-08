#include <iostream>
#include <fstream>
#include <iomanip>

#include "pid.h"
#include "plant.h"

int main() {
    // Simulation parameters
    const double dt = 0.01;     // 10 ms
    const double t_end = 10.0;  // 10 seconds
    const double r_step = 1.0;  // desired setpoint

    // Controller setup
    PID pid;
    pid.set_gains(1.0, 0.5, 0.1);      // example gains
    pid.set_output_limits(-10.0, 10.0); // actuator saturation
    pid.set_derivative_filter(0.5);     // some filtering
    pid.set_setpoint(r_step);

    // Plant setup (K=1, tau=1, y0=0)
    FirstOrderPlant plant(1.0, 1.0, 0.0);

    std::ofstream csv("sim_pid_first_order.csv");
    csv << "t,r,y,u\n";

    double y = 0.0;

    for (double t = 0.0; t <= t_end + 1e-12; t += dt) {
        // Controller computes control from measurement y
        double u = pid.update(y, dt);

        // Plant evolves
        y = plant.step(u, dt);

        csv << std::fixed << std::setprecision(6)
            << t << "," << r_step << "," << y << "," << u << "\n";
    }

    std::cout << "Simulation complete. Output written to sim_pid_first_order.csv\n";
    return 0;
}


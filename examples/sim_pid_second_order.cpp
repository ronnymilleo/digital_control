#include <fstream>
#include <iomanip>
#include <iostream>

#include "exceptions.h"
#include "pid.h"
#include "plant.h"
#include "tuner.h"
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>

int main()
{
    try
    {
        // Simulation parameters
        const double dt     = 0.001; // small dt for second-order dynamics
        const double t_end  = 10.0;  // 10 seconds
        const double r_step = 1.0;   // desired setpoint (position)

        // Load tuned gains from JSON if available, otherwise tune and save
        nlohmann::json             j;
        const std::string          json_path = "tuned_second_order.json";
        DigitalControl::TuneResult tuned{};
        if (std::filesystem::exists(json_path))
        {
            std::ifstream jf(json_path);
            jf >> j;
            if (j.contains("kp") && j.contains("ki") && j.contains("kd"))
            {
                tuned.kp    = j["kp"].get<double>();
                tuned.ki    = j["ki"].get<double>();
                tuned.kd    = j["kd"].get<double>();
                tuned.score = j.value("score", 0.0);
            }
            else
            {
                auto factory = []() { return DigitalControl::make_second_order_plant(1.0, 1.5, 8.0, 1.0, 0.0, 0.0); };
                tuned        = DigitalControl::auto_tune_pid_step_default(factory, r_step, dt, t_end, -50.0, 50.0, 0.7);
                j            = {{"kp", tuned.kp}, {"ki", tuned.ki}, {"kd", tuned.kd}, {"score", tuned.score}};
                std::ofstream jo(json_path);
                jo << j.dump(2) << std::endl;
            }
        }
        else
        {
            auto factory = []() { return DigitalControl::make_second_order_plant(1.0, 1.5, 8.0, 1.0, 0.0, 0.0); };
            tuned        = DigitalControl::auto_tune_pid_step_default(factory, r_step, dt, t_end, -50.0, 50.0, 0.7);
            j            = {{"kp", tuned.kp}, {"ki", tuned.ki}, {"kd", tuned.kd}, {"score", tuned.score}};
            std::ofstream jo(json_path);
            jo << j.dump(2) << std::endl;
        }

        // Controller setup with tuned gains
        auto pid = DigitalControl::make_pid(tuned.kp, tuned.ki, tuned.kd);
        pid->set_output_limits(-50.0, 50.0);
        pid->set_derivative_filter(0.7);
        pid->set_setpoint(r_step);

        // Plant: mass-spring-damper (m, b, k, K, x0, v0)
        auto plant = DigitalControl::make_second_order_plant(1.0, 1.5, 8.0, 1.0, 0.0, 0.0);

        std::ofstream csv("sim_pid_second_order.csv");
        csv << "t,r,y,u\n";

        double y = plant->get_output(); // position output

        for (double t = 0.0; t <= t_end + 1e-12; t += dt)
        {
            double u = pid->update(y, dt);
            y        = plant->step(u, dt);
            csv << std::fixed << std::setprecision(6) << t << "," << r_step << "," << y << "," << u << "\n";
        }

        std::cout << "Second-order simulation with " << tuned << "\n";
        std::cout << "Controller: " << *pid << "\n";
        std::cout << "Plant: " << *plant << "\n";
        std::cout << "Simulation complete. Output written to sim_pid_second_order.csv\n";
    }
    catch (const DigitalControl::DigitalControlException& e)
    {
        std::cerr << "Control error: " << e.what() << std::endl;
        return 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

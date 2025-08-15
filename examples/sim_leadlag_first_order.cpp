#include "leadlag.h"
#include "pid.h"
#include "plant.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <vector>

// Structure to store simulation data
struct SimData
{
    double time;
    double setpoint;
    double output;
    double control;
    double error;
};

// Simulate a control system and return data
std::vector<SimData> simulate_system(DigitalControl::Controller &controller, DigitalControl::IPlant &plant,
                                     double setpoint, double sim_time, double dt)
{
    std::vector<SimData> data;
    controller.set_setpoint(setpoint);
    controller.reset();
    plant.reset();

    const int steps = static_cast<int>(sim_time / dt);

    for (int i = 0; i <= steps; ++i)
    {
        const double t = i * dt;
        const double y = plant.get_output();
        const double u = controller.update(y, dt);
        plant.step(u, dt);

        SimData point;
        point.time = t;
        point.setpoint = setpoint;
        point.output = y;
        point.control = u;
        point.error = setpoint - y;
        data.push_back(point);
    }

    return data;
}

// Save simulation data to CSV
void save_to_csv(const std::vector<SimData> &data, const std::string &filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return;
    }

    // Write header
    file << "time,setpoint,output,control,error\n";

    // Write data
    for (const auto &point : data)
    {
        file << std::fixed << std::setprecision(6) << point.time << "," << point.setpoint << "," << point.output << ","
             << point.control << "," << point.error << "\n";
    }

    file.close();
    std::cout << "Saved simulation data to " << filename << "\n";
}

// Calculate performance metrics
void print_performance_metrics(const std::vector<SimData> &data, const std::string &controller_name)
{
    if (data.empty())
        return;

    // Find maximum overshoot
    double max_output = 0.0;
    double setpoint = data[0].setpoint;
    for (const auto &point : data)
    {
        if (point.output > max_output)
        {
            max_output = point.output;
        }
    }
    double overshoot = ((max_output - setpoint) / setpoint) * 100.0;

    // Find settling time (2% criterion)
    double settling_time = 0.0;
    const double tolerance = 0.02 * setpoint;
    for (auto it = data.rbegin(); it != data.rend(); ++it)
    {
        if (std::abs(it->error) > tolerance)
        {
            settling_time = it->time;
            break;
        }
    }

    // Calculate steady-state error (average of last 10% of data)
    double ss_error = 0.0;
    int ss_count = data.size() / 10;
    for (int i = data.size() - ss_count; i < data.size(); ++i)
    {
        ss_error += std::abs(data[i].error);
    }
    ss_error /= ss_count;

    // Print metrics
    std::cout << "\n=== " << controller_name << " Performance Metrics ===" << std::endl;
    std::cout << "Overshoot: " << std::fixed << std::setprecision(2) << overshoot << "%" << std::endl;
    std::cout << "Settling time (2%): " << std::fixed << std::setprecision(3) << settling_time << " seconds"
              << std::endl;
    std::cout << "Steady-state error: " << std::scientific << std::setprecision(3) << ss_error << std::endl;
}

int main()
{
    std::cout << "=== Lead/Lag Compensator Simulation ===" << std::endl;
    std::cout << "Comparing Lead, Lag, and PID controllers on a first-order plant\n" << std::endl;

    // Simulation parameters
    const double dt = 0.001;      // 1ms time step
    const double sim_time = 10.0; // 10 seconds
    const double setpoint = 1.0;  // Unit step

    // Create plant: G(s) = K/(tau*s + 1)
    const double plant_gain = 2.0;
    const double plant_tau = 1.0;

    // Create controllers
    std::cout << "Creating controllers..." << std::endl;

    // 1. Lead compensator: improves transient response
    // G_lead(s) = K * (s + z)/(s + p), where z < p
    auto lead_comp = DigitalControl::make_lead_compensator(5.0, 0.5, 5.0);
    lead_comp->set_output_limits(-10.0, 10.0);
    std::cout << "Lead compensator: K=5.0, zero=0.5, pole=5.0" << std::endl;

    // 2. Lag compensator: improves steady-state error
    // G_lag(s) = K * (s + z)/(s + p), where z > p
    auto lag_comp = DigitalControl::make_lag_compensator(5.0, 5.0, 0.5);
    lag_comp->set_output_limits(-10.0, 10.0);
    std::cout << "Lag compensator: K=5.0, zero=5.0, pole=0.5" << std::endl;

    // 3. Lead-Lag compensator: combines benefits of both
    auto leadlag_comp = DigitalControl::make_leadlag(5.0, 1.0, 2.0);
    leadlag_comp->set_output_limits(-10.0, 10.0);
    std::cout << "Lead-Lag compensator: K=5.0, zero=1.0, pole=2.0" << std::endl;

    // 4. PID controller for comparison
    auto pid = DigitalControl::make_pid(2.0, 1.0, 0.2);
    pid->set_output_limits(-10.0, 10.0);
    std::cout << "PID controller: Kp=2.0, Ki=1.0, Kd=0.2" << std::endl;

    // Run simulations
    std::cout << "\nRunning simulations..." << std::endl;

    // Lead compensator simulation
    auto plant1 = DigitalControl::make_first_order_plant(plant_gain, plant_tau, 0.0);
    auto lead_data = simulate_system(*lead_comp, *plant1, setpoint, sim_time, dt);
    save_to_csv(lead_data, "leadlag_lead.csv");
    print_performance_metrics(lead_data, "Lead Compensator");

    // Lag compensator simulation
    auto plant2 = DigitalControl::make_first_order_plant(plant_gain, plant_tau, 0.0);
    auto lag_data = simulate_system(*lag_comp, *plant2, setpoint, sim_time, dt);
    save_to_csv(lag_data, "leadlag_lag.csv");
    print_performance_metrics(lag_data, "Lag Compensator");

    // Lead-Lag compensator simulation
    auto plant3 = DigitalControl::make_first_order_plant(plant_gain, plant_tau, 0.0);
    auto leadlag_data = simulate_system(*leadlag_comp, *plant3, setpoint, sim_time, dt);
    save_to_csv(leadlag_data, "leadlag_combined.csv");
    print_performance_metrics(leadlag_data, "Lead-Lag Compensator");

    // PID controller simulation
    auto plant4 = DigitalControl::make_first_order_plant(plant_gain, plant_tau, 0.0);
    auto pid_data = simulate_system(*pid, *plant4, setpoint, sim_time, dt);
    save_to_csv(pid_data, "leadlag_pid.csv");
    print_performance_metrics(pid_data, "PID Controller");

    // Frequency response analysis
    std::cout << "\n=== Frequency Response Analysis ===" << std::endl;

    // Calculate phase and magnitude at key frequencies
    std::vector<double> frequencies = {0.01, 0.1, 1.0, 10.0, 100.0}; // rad/s

    std::cout << "\nLead Compensator Frequency Response:" << std::endl;
    std::cout << "ω (rad/s)\tMagnitude (dB)\tPhase (deg)" << std::endl;
    for (double omega : frequencies)
    {
        double mag = dynamic_cast<DigitalControl::LeadLag *>(lead_comp.get())->get_magnitude_at_frequency(omega);
        double phase = dynamic_cast<DigitalControl::LeadLag *>(lead_comp.get())->get_phase_at_frequency(omega);
        double mag_db = 20.0 * std::log10(mag);
        double phase_deg = phase * 180.0 / std::numbers::pi;
        std::cout << std::fixed << std::setprecision(2) << omega << "\t\t" << mag_db << "\t\t" << phase_deg
                  << std::endl;
    }

    std::cout << "\nLag Compensator Frequency Response:" << std::endl;
    std::cout << "ω (rad/s)\tMagnitude (dB)\tPhase (deg)" << std::endl;
    for (double omega : frequencies)
    {
        double mag = dynamic_cast<DigitalControl::LeadLag *>(lag_comp.get())->get_magnitude_at_frequency(omega);
        double phase = dynamic_cast<DigitalControl::LeadLag *>(lag_comp.get())->get_phase_at_frequency(omega);
        double mag_db = 20.0 * std::log10(mag);
        double phase_deg = phase * 180.0 / std::numbers::pi;
        std::cout << std::fixed << std::setprecision(2) << omega << "\t\t" << mag_db << "\t\t" << phase_deg
                  << std::endl;
    }

    std::cout << "\n=== Simulation Complete ===" << std::endl;
    std::cout << "Data saved to: leadlag_lead.csv, leadlag_lag.csv, leadlag_combined.csv, leadlag_pid.csv" << std::endl;
    std::cout << "\nYou can plot the results using Python:" << std::endl;
    std::cout << "  python scripts/plot_leadlag.py" << std::endl;

    return 0;
}

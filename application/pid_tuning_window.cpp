#include "pid_tuning_window.h"
#include "../core/pid.h"
#include "../core/plant.h"
#include <algorithm>
#include <cmath>
#include <implot.h>
#include <spdlog/spdlog.h>

void PIDTuningWindow::Create()
{
    // Initialize window properties
    m_title = "PID Tuning";
    m_open = true;
    m_flags = ImGuiWindowFlags_None;

    // Initialize PID parameters with default values
    m_pid_params.p = 1.0f;
    m_pid_params.i = 0.1f;
    m_pid_params.d = 0.01f;

    // Initial simulation
    SimulateStepResponse();

    spdlog::info("PIDTuningWindow created");
}

void PIDTuningWindow::Destroy()
{
    // Clean up any resources if needed
    m_open = false;
    spdlog::info("PIDTuningWindow destroyed");
}

void PIDTuningWindow::DrawContents()
{
    ImGui::Text("PID Controller Tuning");
    ImGui::Separator();
    ImGui::Spacing();

    // PID parameter sliders
    ImGui::Text("Controller Gains:");
    if (ImGui::SliderFloat("Proportional (P)", &m_pid_params.p, 0.0f, 10.0f, "%.3f"))
    {
        m_needs_update = true;
    }
    if (ImGui::SliderFloat("Integral (I)", &m_pid_params.i, 0.0f, 5.0f, "%.3f"))
    {
        m_needs_update = true;
    }
    if (ImGui::SliderFloat("Derivative (D)", &m_pid_params.d, 0.0f, 2.0f, "%.3f"))
    {
        m_needs_update = true;
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Plant parameters
    ImGui::Text("Plant Parameters (First Order):");
    if (ImGui::SliderFloat("Plant Gain", &m_plant_gain, 0.1f, 5.0f, "%.2f"))
    {
        m_needs_update = true;
    }
    if (ImGui::SliderFloat("Time Constant", &m_plant_time_constant, 0.1f, 5.0f, "%.2f"))
    {
        m_needs_update = true;
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Simulation parameters
    ImGui::Text("Simulation Settings:");
    if (ImGui::SliderFloat("Setpoint", &m_setpoint, 0.0f, 2.0f, "%.2f"))
    {
        m_needs_update = true;
    }
    if (ImGui::SliderFloat("Sim Time (s)", &m_simulation_time, 1.0f, 20.0f, "%.1f"))
    {
        m_needs_update = true;
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Performance metrics
    float rise_time, settling_time, overshoot;
    CalculatePerformanceMetrics(rise_time, settling_time, overshoot);

    ImGui::Text("Performance Metrics:");
    ImGui::Text("Rise Time (10%%-90%%): %.3f s", rise_time);
    ImGui::Text("Settling Time (2%%): %.3f s", settling_time);
    ImGui::Text("Overshoot: %.1f%%", overshoot);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Control buttons
    if (ImGui::Button("Reset to Defaults"))
    {
        m_pid_params.p = 1.0f;
        m_pid_params.i = 0.1f;
        m_pid_params.d = 0.01f;
        m_plant_gain = 1.0f;
        m_plant_time_constant = 1.0f;
        m_setpoint = 1.0f;
        m_needs_update = true;
        spdlog::info("Parameters reset to defaults");
    }

    ImGui::SameLine();
    if (ImGui::Button("Refresh Plot"))
    {
        m_needs_update = true;
    }

    // Update simulation if needed
    if (m_needs_update)
    {
        SimulateStepResponse();
        m_needs_update = false;
    }

    // Draw the plot
    if (ImPlot::BeginPlot("Step Response", ImVec2(-1, 400)))
    {
        ImPlot::SetupAxes("Time (s)", "Value");
        ImPlot::SetupAxesLimits(
            0, m_simulation_time, -0.1,
            std::max(1.5f * m_setpoint, *std::max_element(m_output_data.begin(), m_output_data.end()) * 1.1f));
            
        ImPlot::SetupAxes("Time (s)", "Control Signal");
        ImPlot::SetupAxesLimits(0, m_simulation_time,
                                *std::min_element(m_control_data.begin(), m_control_data.end()) * 1.1f,
                                *std::max_element(m_control_data.begin(), m_control_data.end()) * 1.1f);

        // Plot setpoint
        ImPlot::SetNextLineStyle(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), 2.0f);
        ImPlot::PlotLine("Setpoint", m_time_data.data(), m_setpoint_data.data(), m_time_data.size());

        // Plot output
        ImPlot::SetNextLineStyle(ImVec4(0.8f, 0.2f, 0.2f, 1.0f), 2.0f);
        ImPlot::PlotLine("Output", m_time_data.data(), m_output_data.data(), m_time_data.size());

        // Plot control
        ImPlot::SetNextLineStyle(ImVec4(0.2f, 0.2f, 0.8f, 1.0f), 2.0f);
        ImPlot::PlotLine("Control", m_time_data.data(), m_control_data.data(), m_time_data.size());

        ImPlot::EndPlot();
    }
}

void PIDTuningWindow::Show()
{
    m_open = true;
}

void PIDTuningWindow::Hide()
{
    m_open = false;
}

void PIDTuningWindow::SimulateStepResponse()
{
    // Clear previous data
    m_time_data.clear();
    m_setpoint_data.clear();
    m_output_data.clear();
    m_control_data.clear();

    // Create PID controller and plant
    DigitalControl::PID pid(m_pid_params.p, m_pid_params.i, m_pid_params.d);
    DigitalControl::FirstOrderPlant plant(m_plant_gain, m_plant_time_constant);

    // Set the setpoint
    pid.set_setpoint(m_setpoint);

    // Set output limits (optional - using reasonable values)
    pid.set_output_limits(-10.0, 10.0);

    // Simulation loop
    float time = 0.0f;
    const int num_steps = static_cast<int>(m_simulation_time / m_time_step);

    for (int i = 0; i <= num_steps; ++i)
    {
        // Get current plant output
        double y = plant.get_output();

        // Calculate control signal
        double u = pid.update(y, m_time_step);

        // Apply control to plant
        plant.step(u, m_time_step);

        // Store data
        m_time_data.push_back(time);
        m_setpoint_data.push_back(m_setpoint);
        m_output_data.push_back(static_cast<float>(y));
        m_control_data.push_back(static_cast<float>(u));

        time += m_time_step;
    }
}

void PIDTuningWindow::CalculatePerformanceMetrics(float &rise_time, float &settling_time, float &overshoot)
{
    if (m_output_data.empty())
    {
        rise_time = 0.0f;
        settling_time = 0.0f;
        overshoot = 0.0f;
        return;
    }

    // Calculate rise time (10% to 90% of setpoint)
    float ten_percent = 0.1f * m_setpoint;
    float ninety_percent = 0.9f * m_setpoint;

    int rise_start = -1, rise_end = -1;
    for (size_t i = 0; i < m_output_data.size(); ++i)
    {
        if (rise_start == -1 && m_output_data[i] >= ten_percent)
        {
            rise_start = i;
        }
        if (rise_end == -1 && m_output_data[i] >= ninety_percent)
        {
            rise_end = i;
            break;
        }
    }

    if (rise_start != -1 && rise_end != -1)
    {
        rise_time = m_time_data[rise_end] - m_time_data[rise_start];
    }
    else
    {
        rise_time = -1.0f; // Not reached
    }

    // Calculate overshoot
    float max_value = *std::max_element(m_output_data.begin(), m_output_data.end());
    if (max_value > m_setpoint)
    {
        overshoot = ((max_value - m_setpoint) / m_setpoint) * 100.0f;
    }
    else
    {
        overshoot = 0.0f;
    }

    // Calculate settling time (within 2% of setpoint)
    float settling_band = 0.02f * m_setpoint;
    settling_time = -1.0f;

    // Search backwards from the end
    for (int i = m_output_data.size() - 1; i >= 0; --i)
    {
        if (std::abs(m_output_data[i] - m_setpoint) > settling_band)
        {
            if (i < m_output_data.size() - 1)
            {
                settling_time = m_time_data[i + 1];
            }
            break;
        }
    }

    // If all samples are within the settling band, settling time is 0
    if (settling_time == -1.0f)
    {
        settling_time = 0.0f;
    }
}

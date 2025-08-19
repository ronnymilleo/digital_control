#include "leadlag_tuning_window.h"
#include <algorithm>
#include <cmath>
#include <complex>
#include <implot.h>
#include <leadlag.h>
#include <memory>
#include <numbers>
#include <plant.h>
#include <spdlog/spdlog.h>

void LeadLagTuningWindow::Create()
{
    // Initialize window properties
    m_title = "Lead/Lag Compensator Tuning";
    m_open  = true;
    m_flags = ImGuiWindowFlags_None;

    // Initialize with default lead compensator parameters
    m_leadlag_params.gain = 1.0f;
    m_leadlag_params.zero = 1.0f;
    m_leadlag_params.pole = 10.0f;

    // Initial simulation
    SimulateStepResponse();

    spdlog::info("LeadLagTuningWindow created");
}

void LeadLagTuningWindow::Destroy()
{
    m_open = false;
    spdlog::info("LeadLagTuningWindow destroyed");
}

void LeadLagTuningWindow::DrawContents()
{
    ImGui::Text("Lead/Lag Compensator Design");
    ImGui::Separator();
    ImGui::Spacing();

    // Compensator type selection
    ImGui::Text("Compensator Type:");
    if (ImGui::RadioButton("Lead", m_compensator_type == CompensatorType::Lead))
    {
        m_compensator_type = CompensatorType::Lead;
        m_needs_update     = true;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Lag", m_compensator_type == CompensatorType::Lag))
    {
        m_compensator_type = CompensatorType::Lag;
        m_needs_update     = true;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Lead-Lag", m_compensator_type == CompensatorType::LeadLag))
    {
        m_compensator_type = CompensatorType::LeadLag;
        m_needs_update     = true;
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Compensator parameters
    ImGui::Text("Compensator Parameters:");
    if (ImGui::SliderFloat("Gain", &m_leadlag_params.gain, 0.1f, 20.0f, "%.2f"))
    {
        m_needs_update = true;
    }

    if (ImGui::SliderFloat("Zero Location", &m_leadlag_params.zero, 0.01f, 20.0f, "%.3f", ImGuiSliderFlags_Logarithmic))
    {
        m_needs_update = true;
    }

    if (ImGui::SliderFloat("Pole Location", &m_leadlag_params.pole, 0.01f, 100.0f, "%.3f",
                           ImGuiSliderFlags_Logarithmic))
    {
        m_needs_update = true;
    }

    // Show alpha (pole/zero ratio)
    float alpha = m_leadlag_params.pole / m_leadlag_params.zero;
    ImGui::Text("Alpha (pole/zero): %.3f", alpha);

    // Compensator type indicator
    if (alpha > 1.0f)
    {
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), "Configuration: LEAD (alpha > 1)");
    }
    else if (alpha < 1.0f)
    {
        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.2f, 1.0f), "Configuration: LAG (alpha < 1)");
    }
    else
    {
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "Configuration: Unity gain (alpha = 1)");
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Plant type selection
    ImGui::Text("Plant Type:");
    if (ImGui::RadioButton("First Order", m_plant_type == PlantType::FirstOrder))
    {
        m_plant_type   = PlantType::FirstOrder;
        m_needs_update = true;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Second Order", m_plant_type == PlantType::SecondOrder))
    {
        m_plant_type   = PlantType::SecondOrder;
        m_needs_update = true;
    }

    ImGui::Spacing();

    // Plant parameters based on type
    if (m_plant_type == PlantType::FirstOrder)
    {
        ImGui::Text("First Order Plant Parameters:");
        if (ImGui::SliderFloat("Plant Gain", &m_plant_gain, 0.1f, 5.0f, "%.2f"))
        {
            m_needs_update = true;
        }
        if (ImGui::SliderFloat("Time Constant", &m_plant_time_constant, 0.1f, 5.0f, "%.2f"))
        {
            m_needs_update = true;
        }
    }
    else
    {
        ImGui::Text("Second Order Plant Parameters:");
        if (ImGui::SliderFloat("Mass", &m_plant_mass, 0.1f, 5.0f, "%.2f"))
        {
            m_needs_update = true;
        }
        if (ImGui::SliderFloat("Damping", &m_plant_damping, 0.0f, 5.0f, "%.2f"))
        {
            m_needs_update = true;
        }
        if (ImGui::SliderFloat("Stiffness", &m_plant_stiffness, 0.1f, 20.0f, "%.2f"))
        {
            m_needs_update = true;
        }
        if (ImGui::SliderFloat("Input Gain", &m_plant_input_gain, 0.1f, 5.0f, "%.2f"))
        {
            m_needs_update = true;
        }

        // Show damping ratio
        float omega_n = std::sqrt(m_plant_stiffness / m_plant_mass);
        float zeta    = m_plant_damping / (2.0f * std::sqrt(m_plant_mass * m_plant_stiffness));
        ImGui::Text("Natural Frequency: %.2f rad/s", omega_n);
        ImGui::Text("Damping Ratio: %.3f", zeta);
        if (zeta < 1.0f)
        {
            ImGui::TextColored(ImVec4(0.8f, 0.4f, 0.2f, 1.0f), "Underdamped");
        }
        else if (zeta == 1.0f)
        {
            ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), "Critically Damped");
        }
        else
        {
            ImGui::TextColored(ImVec4(0.2f, 0.4f, 0.8f, 1.0f), "Overdamped");
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Simulation settings
    ImGui::Text("Simulation Settings:");
    if (ImGui::SliderFloat("Setpoint", &m_setpoint, 0.0f, 2.0f, "%.2f"))
    {
        m_needs_update = true;
    }
    if (ImGui::SliderFloat("Sim Time (s)", &m_simulation_time, 1.0f, 20.0f, "%.1f"))
    {
        m_needs_update = true;
    }

    // Output limits
    if (ImGui::Checkbox("Use Output Limits", &m_use_output_limits))
    {
        m_needs_update = true;
    }
    if (m_use_output_limits)
    {
        ImGui::SameLine();
        if (ImGui::SliderFloat("Min", &m_output_min, -20.0f, 0.0f, "%.1f"))
        {
            m_needs_update = true;
        }
        ImGui::SameLine();
        if (ImGui::SliderFloat("Max", &m_output_max, 0.0f, 20.0f, "%.1f"))
        {
            m_needs_update = true;
        }
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
        m_leadlag_params.gain = 1.0f;
        m_leadlag_params.zero = 1.0f;
        m_leadlag_params.pole = 10.0f;
        m_plant_gain          = 1.0f;
        m_plant_time_constant = 1.0f;
        m_setpoint            = 1.0f;
        m_needs_update        = true;
        spdlog::info("Lead/Lag parameters reset to defaults");
    }

    ImGui::SameLine();
    if (ImGui::Checkbox("Show Open-Loop", &m_show_openloop))
    {
        m_needs_update = true;
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

    // Draw the step response plot
    if (ImPlot::BeginPlot("Step Response", ImVec2(-1, 350)))
    {
        ImPlot::SetupAxes("Time (s)", "Value");
        ImPlot::SetupAxesLimits(
            0, m_simulation_time, -0.1,
            std::max(1.5f * m_setpoint, *std::max_element(m_output_data.begin(), m_output_data.end()) * 1.1f));

        // Plot setpoint
        ImPlot::SetNextLineStyle(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), 2.0f);
        ImPlot::PlotLine("Setpoint", m_time_data.data(), m_setpoint_data.data(), m_time_data.size());

        // Plot compensated output
        ImPlot::SetNextLineStyle(ImVec4(0.8f, 0.2f, 0.2f, 1.0f), 2.0f);
        ImPlot::PlotLine("Compensated Output", m_time_data.data(), m_output_data.data(), m_time_data.size());

        // Plot open-loop response if enabled
        if (m_show_openloop && !m_openloop_data.empty())
        {
            ImPlot::SetNextLineStyle(ImVec4(0.5f, 0.5f, 0.5f, 0.7f), 1.5f);
            ImPlot::PlotLine("Open-Loop", m_time_data.data(), m_openloop_data.data(), m_time_data.size());
        }

        // Plot control signal
        ImPlot::SetNextLineStyle(ImVec4(0.2f, 0.2f, 0.8f, 0.7f), 1.5f);
        ImPlot::PlotLine("Control Signal", m_time_data.data(), m_control_data.data(), m_time_data.size());

        ImPlot::EndPlot();
    }

    // Optional: Add frequency response plot
    if (ImGui::Checkbox("Show Bode Plot", &m_show_bode_plot))
    {
        m_needs_update = true;
    }

    if (m_show_bode_plot)
    {
        DrawBodePlot();
    }
}

void LeadLagTuningWindow::Show()
{
    m_open = true;
}

void LeadLagTuningWindow::Hide()
{
    m_open = false;
}

void LeadLagTuningWindow::SimulateStepResponse()
{
    // Clear previous data
    m_time_data.clear();
    m_setpoint_data.clear();
    m_output_data.clear();
    m_control_data.clear();
    m_openloop_data.clear();

    // Create compensator based on type
    std::unique_ptr<DigitalControl::LeadLag> compensator;

    try
    {
        switch (m_compensator_type)
        {
        case CompensatorType::Lead:
            // For lead: ensure zero < pole
            if (m_leadlag_params.zero >= m_leadlag_params.pole)
            {
                // Swap them if needed
                compensator = DigitalControl::make_lead_compensator(
                    m_leadlag_params.gain, std::min(m_leadlag_params.zero, m_leadlag_params.pole * 0.9f),
                    std::max(m_leadlag_params.zero * 1.1f, m_leadlag_params.pole));
            }
            else
            {
                compensator = DigitalControl::make_lead_compensator(m_leadlag_params.gain, m_leadlag_params.zero,
                                                                    m_leadlag_params.pole);
            }
            break;
        case CompensatorType::Lag:
            // For lag: ensure zero > pole
            if (m_leadlag_params.zero <= m_leadlag_params.pole)
            {
                // Swap them if needed
                compensator = DigitalControl::make_lag_compensator(
                    m_leadlag_params.gain, std::max(m_leadlag_params.zero, m_leadlag_params.pole * 1.1f),
                    std::min(m_leadlag_params.zero * 0.9f, m_leadlag_params.pole));
            }
            else
            {
                compensator = DigitalControl::make_lag_compensator(m_leadlag_params.gain, m_leadlag_params.zero,
                                                                   m_leadlag_params.pole);
            }
            break;
        case CompensatorType::LeadLag:
            compensator =
                DigitalControl::make_leadlag(m_leadlag_params.gain, m_leadlag_params.zero, m_leadlag_params.pole);
            break;
        }
    }
    catch (const std::exception& e)
    {
        spdlog::error("Failed to create compensator: {}", e.what());
        // Create a default compensator as fallback
        compensator = DigitalControl::make_leadlag(1.0, 1.0, 10.0);
    }
    // Set output limits if enabled
    if (m_use_output_limits)
    {
        compensator->set_output_limits(m_output_min, m_output_max);
    }

    // Create plant based on type
    std::unique_ptr<DigitalControl::IPlant> plant;
    std::unique_ptr<DigitalControl::IPlant> openloop_plant;

    if (m_plant_type == PlantType::FirstOrder)
    {
        plant          = DigitalControl::make_first_order_plant(m_plant_gain, m_plant_time_constant);
        openloop_plant = DigitalControl::make_first_order_plant(m_plant_gain, m_plant_time_constant);
    }
    else
    {
        plant          = DigitalControl::make_second_order_plant(m_plant_mass, m_plant_damping, m_plant_stiffness,
                                                                 m_plant_input_gain);
        openloop_plant = DigitalControl::make_second_order_plant(m_plant_mass, m_plant_damping, m_plant_stiffness,
                                                                 m_plant_input_gain);
    }

    // Set the setpoint
    compensator->set_setpoint(m_setpoint);

    // Simulation loop
    float     time      = 0.0f;
    const int num_steps = static_cast<int>(m_simulation_time / m_time_step);

    for (int i = 0; i <= num_steps; ++i)
    {
        // Compensated system
        double y = plant->get_output();
        double u = compensator->update(y, m_time_step);
        plant->step(u, m_time_step);

        // Open-loop system (direct step input)
        if (m_show_openloop)
        {
            openloop_plant->step(m_setpoint, m_time_step);
        }

        // Store data
        m_time_data.push_back(time);
        m_setpoint_data.push_back(m_setpoint);
        m_output_data.push_back(static_cast<float>(y));
        m_control_data.push_back(static_cast<float>(u));

        if (m_show_openloop)
        {
            m_openloop_data.push_back(static_cast<float>(openloop_plant->get_output()));
        }

        time += m_time_step;
    }
}

void LeadLagTuningWindow::CalculatePerformanceMetrics(float& rise_time, float& settling_time, float& overshoot)
{
    if (m_output_data.empty())
    {
        rise_time     = 0.0f;
        settling_time = 0.0f;
        overshoot     = 0.0f;
        return;
    }

    // Calculate rise time (10% to 90% of setpoint)
    float ten_percent    = 0.1f * m_setpoint;
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
    settling_time       = -1.0f;

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

void LeadLagTuningWindow::DrawBodePlot()
{
    std::vector<float> frequencies;
    std::vector<float> magnitudes_db;
    std::vector<float> phases_deg;

    CalculateFrequencyResponse(frequencies, magnitudes_db, phases_deg);

    if (frequencies.empty())
        return;

    // Magnitude plot
    if (ImPlot::BeginPlot("Bode Magnitude", ImVec2(-1, 200)))
    {
        ImPlot::SetupAxes("Frequency (rad/s)", "Magnitude (dB)");
        ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);

        float freq_min = frequencies.front();
        float freq_max = frequencies.back();
        float mag_min  = *std::min_element(magnitudes_db.begin(), magnitudes_db.end()) - 5.0f;
        float mag_max  = *std::max_element(magnitudes_db.begin(), magnitudes_db.end()) + 5.0f;

        ImPlot::SetupAxesLimits(freq_min, freq_max, mag_min, mag_max);

        ImPlot::SetNextLineStyle(ImVec4(0.8f, 0.2f, 0.2f, 1.0f), 2.0f);
        ImPlot::PlotLine("Magnitude", frequencies.data(), magnitudes_db.data(), frequencies.size());

        ImPlot::EndPlot();
    }

    // Phase plot
    if (ImPlot::BeginPlot("Bode Phase", ImVec2(-1, 200)))
    {
        ImPlot::SetupAxes("Frequency (rad/s)", "Phase (degrees)");
        ImPlot::SetupAxisScale(ImAxis_X1, ImPlotScale_Log10);

        float freq_min = frequencies.front();
        float freq_max = frequencies.back();

        ImPlot::SetupAxesLimits(freq_min, freq_max, -90.0, 90.0);

        ImPlot::SetNextLineStyle(ImVec4(0.2f, 0.2f, 0.8f, 1.0f), 2.0f);
        ImPlot::PlotLine("Phase", frequencies.data(), phases_deg.data(), frequencies.size());

        ImPlot::EndPlot();
    }
}

void LeadLagTuningWindow::CalculateFrequencyResponse(std::vector<float>& frequencies, std::vector<float>& magnitudes_db,
                                                     std::vector<float>& phases_deg)
{
    frequencies.clear();
    magnitudes_db.clear();
    phases_deg.clear();

    // Generate logarithmically spaced frequencies
    const int num_points = 100;
    float     freq_min   = 0.01f;
    float     freq_max   = 100.0f;

    for (int i = 0; i < num_points; ++i)
    {
        float t    = static_cast<float>(i) / (num_points - 1);
        float freq = freq_min * std::pow(freq_max / freq_min, t);
        frequencies.push_back(freq);

        // Calculate complex response G(jw) = K * (jw + z) / (jw + p)
        std::complex<float> jw(0.0f, freq);
        std::complex<float> numerator   = m_leadlag_params.gain * (jw + m_leadlag_params.zero);
        std::complex<float> denominator = jw + m_leadlag_params.pole;
        std::complex<float> response    = numerator / denominator;

        // Calculate magnitude in dB
        float magnitude    = std::abs(response);
        float magnitude_db = 20.0f * std::log10(magnitude);
        magnitudes_db.push_back(magnitude_db);

        // Calculate phase in degrees
        float phase_rad = std::arg(response);
        float phase_deg = phase_rad * 180.0f / std::numbers::pi_v<float>;
        phases_deg.push_back(phase_deg);
    }
}

#pragma once

#include "window_interface.h"
#include <vector>

namespace DigitalControl
{
class LeadLag;
class FirstOrderPlant;
class SecondOrderPlant;
} // namespace DigitalControl

class LeadLagTuningWindow : public IWindowInterface
{
public:
    void Create() override;
    void Destroy() override;
    void DrawContents() override;
    void Show() override;
    void Hide() override;

private:
    enum class CompensatorType
    {
        Lead,
        Lag,
        LeadLag
    };

    enum class PlantType
    {
        FirstOrder,
        SecondOrder
    };

    struct LeadLagParams
    {
        float gain = 1.0f;
        float zero = 1.0f;  // Zero location
        float pole = 10.0f; // Pole location
    } m_leadlag_params;

    // Simulation data
    std::vector<float> m_time_data;
    std::vector<float> m_setpoint_data;
    std::vector<float> m_output_data;
    std::vector<float> m_control_data;
    std::vector<float> m_openloop_data; // For comparison

    // Simulation parameters
    float m_simulation_time = 10.0f;
    float m_time_step       = 0.01f;
    float m_setpoint        = 1.0f;

    // Compensator configuration
    CompensatorType m_compensator_type = CompensatorType::Lead;
    PlantType       m_plant_type       = PlantType::FirstOrder;

    // Plant parameters
    // First order
    float m_plant_gain          = 1.0f;
    float m_plant_time_constant = 1.0f;

    // Second order
    float m_plant_mass       = 1.0f;
    float m_plant_damping    = 0.5f;
    float m_plant_stiffness  = 4.0f;
    float m_plant_input_gain = 1.0f;

    // Output limits
    bool  m_use_output_limits = true;
    float m_output_min        = -10.0f;
    float m_output_max        = 10.0f;

    // Flags
    bool m_needs_update   = true;
    bool m_show_openloop  = true;
    bool m_show_bode_plot = false;

    // Methods
    void SimulateStepResponse();
    void CalculatePerformanceMetrics(float& rise_time, float& settling_time, float& overshoot);
    void DrawBodePlot();
    void CalculateFrequencyResponse(std::vector<float>& frequencies, std::vector<float>& magnitudes_db,
                                    std::vector<float>& phases_deg);
};

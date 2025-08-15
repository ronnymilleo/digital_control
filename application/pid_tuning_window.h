#pragma once

#include "window_interface.h"
#include <vector>

namespace DigitalControl
{
class PID;
class FirstOrderPlant;
} // namespace DigitalControl

class PIDTuningWindow : public IWindowInterface
{
  public:
    void Create() override;
    void Destroy() override;
    void DrawContents() override;
    void Show() override;
    void Hide() override;

  private:
    struct PIDParams
    {
        float p = 1.0f;
        float i = 0.1f;
        float d = 0.01f;
    } m_pid_params;
    
    // Simulation data
    std::vector<float> m_time_data;
    std::vector<float> m_setpoint_data;
    std::vector<float> m_output_data;
    std::vector<float> m_control_data;
    
    // Simulation parameters
    float m_simulation_time = 10.0f;
    float m_time_step = 0.01f;
    float m_setpoint = 1.0f;
    
    // Plant parameters (first order system)
    float m_plant_gain = 1.0f;
    float m_plant_time_constant = 1.0f;
    
    // Flags
    bool m_needs_update = true;
    
    // Methods
    void SimulateStepResponse();
    void CalculatePerformanceMetrics(float& rise_time, float& settling_time, float& overshoot);
};

#include "about_window.h"

AboutWindow::AboutWindow()
{
    m_title = "About Digital Control";
    m_open  = false;
    m_flags = ImGuiWindowFlags_AlwaysAutoResize;
}

void AboutWindow::Create()
{
}

void AboutWindow::Destroy()
{
}

void AboutWindow::Show()
{
    m_open = true;
}

void AboutWindow::Hide()
{
    m_open = false;
}

void AboutWindow::DrawContents()
{
    ImGui::Text("Digital Control System");
    ImGui::Separator();
    ImGui::Text("Version: 0.1.0");
    ImGui::Text("A C++23 digital controller simulation tool.");
    ImGui::Text("Built with ImGui and ImPlot for interactive tuning.");
    
    ImGui::Spacing();
    ImGui::Text("Features:");
    ImGui::BulletText("PID Controller Tuning");
    ImGui::BulletText("Lead/Lag Compensator Design");
    ImGui::BulletText("First and Second Order Plant Models");
    ImGui::BulletText("Real-time Performance Metrics");
    
    ImGui::Spacing();
    ImGui::Separator();
    
    if (ImGui::Button("Close"))
    {
        m_open = false;
    }
}

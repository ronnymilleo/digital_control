#include "app_gui.h"
#include "imgui_layer.h"
#include <spdlog/spdlog.h>

// Main code
int main(int, char **)
{
    AppGUI app;
    if (app.Init() != 0)
    {
        spdlog::error("Failed to initialize AppGUI");
        return 1;
    }

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    bool show_demo_window = true;

    while (!app.ShouldClose())
    {
        app.PollEvents();

        ImGuiLayer::BeginFrame();
        ImGuiLayer::EnableDockSpace();

        // Render your custom windows here
        ImGui::ShowDemoWindow(&show_demo_window);

        ImGuiLayer::Render(app.GetWindow(), clear_color);
        ImGuiLayer::EndFrame();

        app.SwapBuffers();
    }

    ImGuiLayer::Shutdown();

    return 0;
}

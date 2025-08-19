# WARP.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Project Overview

Digital Control is a C++23 project for simulating digital controllers, featuring PID controllers driving first- and second-order plants. It includes both a static library (`digital_control_lib`) and a GUI application with real-time visualization using ImGui/ImPlot.

## Build Commands


```bash
cmake --preset clang-debug-tests
cmake --build --preset clang-debug-tests
```

## Running the Application

### GUI Application
```bash
# Run the main GUI application with PID tuning interface
./release/bin/digital_control
```

### Example Simulations
```bash
# Run example simulations (generates CSV files and plots)
./release/bin/sim_pid_first_order
./release/bin/sim_pid_second_order
./release/bin/sim_leadlag_first_order  # Lead/Lag compensator examples
```

## Architecture Overview

### Core Library (`digital_control_lib`)
The library is organized in the `DigitalControl` namespace with the following key components:

- **Controller Interface** (`core/controller.h`): Abstract base class defining the controller API
  - `PID` class: PID controller with anti-windup and derivative filtering
  - `LeadLag` class: Lead/Lag compensator for phase lead/lag compensation
  
- **Plant Models** (`core/plant.h`): System dynamics simulation
  - `FirstOrderPlant`: Simple first-order system (y' = -(y - K*u)/tau)
  - `SecondOrderPlant`: Mass-spring-damper system
  
- **Tuner** (`core/tuner.h`): Auto-tuning algorithms for PID parameters
  
- **Exception Hierarchy** (`core/exceptions.h`): Custom exceptions for error handling
  - `DigitalControlException` (base)
  - `InvalidParameterException`
  - `NumericalException`
  - `TuningException`

### GUI Application (`application/`)
- **AppGUI** (`app_gui.h/cpp`): Main application class managing the GUI lifecycle
- **ImGuiLayer** (`imgui_layer.h/cpp`): ImGui initialization and rendering
- **PIDTuningWindow** (`pid_tuning_window.h/cpp`): Interactive PID tuning interface with real-time plots
- **WindowInterface** (`window_interface.h`): Base interface for GUI windows

### Factory Functions
The library provides factory functions for easy object creation:
- `make_pid(kp, ki, kd)` - Create PID controller
- `make_leadlag(K, zero, pole)` - Create Lead/Lag compensator
- `make_lead_compensator(K, zero, pole)` - Create Lead compensator (zero < pole)
- `make_lag_compensator(K, zero, pole)` - Create Lag compensator (zero > pole)
- `make_first_order_plant(K, tau, y0)` - Create first-order plant
- `make_second_order_plant(m, b, k, K, x0, v0)` - Create second-order plant

### Vendored Dependencies
Located in `vendor/` (all included, no external installation needed):
- **GoogleTest**: Unit testing framework
- **nlohmann/json**: JSON parsing and serialization
- **ImGui**: Immediate mode GUI library
- **ImPlot**: Plotting extension for ImGui
- **GLFW**: Window management and OpenGL context
- **spdlog**: Fast logging library
- **stb**: Image loading utilities

## Development Workflow

### Adding New Controllers
1. Inherit from `Controller` base class in `core/controller.h`
2. Implement required virtual methods: `set_setpoint()`, `update()`, `reset()`, etc.
3. Add factory function following existing patterns
4. Create unit tests in `tests/`
5. Add example simulation in `examples/`

### Adding New Plant Models
1. Inherit from `IPlant` interface in `core/plant.h`
2. Implement `step()`, `get_output()`, and `reset()` methods
3. Validate inputs and throw appropriate exceptions
4. Add factory function
5. Test with existing controllers

### GUI Development
1. Create new window class inheriting from `WindowInterface`
2. Implement `on_update()` and `on_imgui_render()` methods
3. Register window in `AppGUI` class
4. Use ImPlot for visualization needs

## Testing

### Unit Tests
```bash
# Build and run all tests
cmake --preset clang-debug-tests
cmake --build --preset clang-debug-tests
./release/bin/digital_control_tests

# Run specific test suites
./release/bin/digital_control_tests --gtest_filter=PID*         # PID tests
./release/bin/digital_control_tests --gtest_filter=LeadLag*     # Lead/Lag tests
./release/bin/digital_control_tests --gtest_filter=*Plant*      # Plant tests
./release/bin/digital_control_tests --gtest_filter=Integration* # Integration tests
```

### Test Coverage
The test suite now includes:
- **PID Controller Tests**: Comprehensive testing of saturation, anti-windup, derivative filtering, move semantics
- **Lead/Lag Compensator Tests**: Frequency response validation, phase/magnitude verification, stability tests
- **Plant Model Tests**: First and second-order plant dynamics, numerical stability, boundary conditions
- **Integration Tests**: Complete control loop simulations, disturbance rejection, tracking performance, multi-controller comparisons

### Manual Testing
- Use the GUI application for interactive testing
- Modify example programs in `examples/` for specific scenarios
- Generated CSV files can be analyzed with `scripts/plot_sim.py`

## Common Tasks

### Updating PID Tuning Parameters
```cpp
// In code
auto pid = make_pid(1.0, 0.1, 0.05);  // Kp, Ki, Kd
pid->set_output_limits(-100, 100);     // Set actuator limits
pid->set_derivative_filter(0.8);       // Filter derivative (0-1)

// Or load from JSON
// See examples/tuned_first_order.json for format
```

### Simulating Control Loops
```cpp
// PID Control
auto controller = make_pid(2.0, 0.5, 0.1);
auto plant = make_first_order_plant(1.0, 2.0);
controller->set_setpoint(1.0);

// Lead Compensator (improves transient response)
auto lead = make_lead_compensator(10.0, 0.1, 1.0);

// Lag Compensator (reduces steady-state error)
auto lag = make_lag_compensator(10.0, 10.0, 0.1);

double dt = 0.01;  // 10ms sample time
for (int i = 0; i < 1000; ++i) {
    double y = plant->get_output();
    double u = controller->update(y, dt);
    plant->step(u, dt);
}
```

### Debugging Build Issues
```bash
# Clean build
rm -rf build release Debug
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j --verbose

# Check compile commands
cat compile_commands.json  # Auto-copied to source dir for clangd
```

## Performance Considerations

- The library uses C++23 features including constexpr constructors for compile-time optimization
- Plant models validate inputs to prevent numerical instability
- PID controller includes anti-windup to handle actuator saturation
- GUI runs at 60 FPS by default with efficient immediate-mode rendering
- Example simulations write CSV files incrementally to handle long runs

## Future Roadmap

See `UPGRADE_ROADMAP.md` for planned improvements including:
- Additional controller types (LQR, MPC, adaptive)
- Advanced numerical integrators (RK4, RK45)
- Frequency domain analysis tools
- Real-time scheduling support
- Hardware-in-the-loop capabilities

# Digital Control: PID + First Order Plant Simulator

This document tracks the steps taken to set up a minimal, working simulator for a generic digital controller (PID) running on a simple plant.

## What was added

1) Generic controller interface
- controller.h defines an abstract Controller with set_setpoint(r), update(y, dt) -> u, and reset().

2) PID controller
- pid.h/.cpp implement a discrete PID with:
  - Gains: Kp, Ki, Kd
  - Output limits (anti-windup via clamped integrator/back-calculation)
  - Derivative on measurement with optional first-order filtering (alpha)
  - API: set_gains, set_setpoint, set_output_limits, set_derivative_filter, update, reset

3) Plant interface and simple first-order plant
- plant.h defines IPlant and FirstOrderPlant: y' = (-(y - K*u))/tau, discretized via forward Euler.

4) Simulation example
- examples/sim_pid_first_order.cpp runs a step response simulation for 10s at dt=0.01, writing CSV to sim_pid_first_order.csv with columns t,r,y,u.

5) CMake cleanup
- Replaced previous CMake contents with a focused build:
  - Static library target: digital_control (sources: pid.cpp, plant.cpp)
  - Example executable: sim_pid_first_order
  - Install headers: controller.h, pid.h, plant.h
  - Configurable options: DC_BUILD_EXAMPLES, DC_ENABLE_WARNINGS, DC_WARNINGS_AS_ERRORS

6) Removed broken demo
- Removed pidtuner_demo.cpp that referenced a non-existent pidtuner.h.

## How to build and run

Using CMake presets (Debug and Release available):

- Configure (Debug):
  cmake --preset debug

- Build example:
  cmake --build build/debug --target sim_pid_first_order -j

- Run the simulator:
  ./build/debug/sim_pid_first_order

This will produce sim_pid_first_order.csv in the current working directory with the time history of setpoint r, plant output y, and control u.

## Next ideas

- Add unit tests for PID update math and plant stepping.
- Add more plants (second-order mass-spring-damper, integrator, etc.).
- Add reference profiles (ramps, sine) and disturbance injection.
- Add a simple tuner (e.g., relay auto-tune) as a separate component.
- Add a small plotting script (Python/matplotlib) to visualize results.

## Updates

- Added SecondOrderPlant (mass-spring-damper) in plant.h and new example examples/sim_pid_second_order.cpp.
- CMake updated with sim_pid_second_order target.


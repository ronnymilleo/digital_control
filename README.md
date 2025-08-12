# Digital Control

A minimal C++17 project for simulating digital controllers, featuring a PID controller driving first- and second-order plants. It includes a small static library, example programs that generate data/plots, and unit tests.

## Features
- C++17 static library: `digital_control`
- PID controller with simple plant models
- Example programs:
  - `sim_pid_first_order`
  - `sim_pid_second_order`
- Unit tests using GoogleTest (vendored)
- Vendor dependency: nlohmann/json (single-header)
- Optional clang-format integration

## Requirements
- CMake 3.28+
- A C++17 compiler (Clang, GCC, or MSVC)
- Optional: `clang-format` for automatic formatting

## Quick start
```bash
# Configure and build (Release by default per CMakeLists)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```
Artifacts are placed under:
- Binaries: `Release/bin/`
- Libraries: `Release/lib/`

## Build options
These CMake options can be toggled at configure time:
- `-DDC_BUILD_TESTS=ON|OFF` (default ON)
- `-DDC_BUILD_EXAMPLES=ON|OFF` (default ON)
- `-DDC_ENABLE_WARNINGS=ON|OFF` (default ON)
- `-DDC_WARNINGS_AS_ERRORS=ON|OFF` (default OFF)

Example:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug \
      -DDC_BUILD_TESTS=ON -DDC_BUILD_EXAMPLES=ON \
      -DDC_ENABLE_WARNINGS=ON -DDC_WARNINGS_AS_ERRORS=OFF
cmake --build build -j
```

## Running the examples
After a successful build, example executables are available at `Release/bin` (or `Debug/bin` depending on the build type):

```bash
./Release/bin/sim_pid_first_order
./Release/bin/sim_pid_second_order
```

These examples may generate output CSV files and plots in the project root, such as:
- `sim_pid_first_order.csv`
- `sim_pid_second_order.csv`
- `first_order.png`
- `second_order.png`

## Running tests
GoogleTest is vendored under `vendor/googletest`, so no external install is required.

```bash
# If you configured with -DDC_BUILD_TESTS=ON
cmake --build build --target digital_control_tests
./Release/bin/digital_control_tests

# Or via CTest from the build directory
ctest --test-dir build --output-on-failure
```

## Formatting
If `clang-format` is available in PATH, the build runs a `format` target automatically to format key source files. You can also invoke it explicitly:
```bash
cmake --build build --target format
```

## Project structure (selected)
- `pid.cpp`, `pid.h` – PID controller
- `plant.cpp`, `plant.h` – Plant models (first/second order)
- `tuner.cpp`, `tuner.h` – Simple tuning utilities
- `examples/` – Example programs
- `tests/` – Unit tests (GoogleTest)
- `vendor/` – Vendored dependencies (googletest, nlohmann_json)

## License
This project is licensed under the GNU General Public License v3.0 (GPL-3.0). See `LICENSE` for details.

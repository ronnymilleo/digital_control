#include "pidtuner.h"

PIDTuner::PIDTuner() {
    // Initialize PID gains
    kp_ = 0.0;
    ki_ = 0.0;
    kd_ = 0.0;
}

PIDTuner::~PIDTuner() {
    // Destructor
}

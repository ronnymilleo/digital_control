#include "pidtuner.h"
#include <iostream>

int main() {
    PIDTuner pidtuner;
    pidtuner.set_gains(1.0, 0.0, 0.0);
    pidtuner.set_setpoint(100.0);
    pidtuner.set_process_value(0.0);
    std::cout << "PID output: " << pidtuner.get_output() << std::endl;
    return 0;
}

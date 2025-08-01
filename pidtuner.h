#pragma once

class PIDTuner {
    public:
        PIDTuner();
        ~PIDTuner();

        void set_gains(double kp, double ki, double kd);
        void set_setpoint(double setpoint);
        void set_process_value(double process_value);
        double get_output();
};
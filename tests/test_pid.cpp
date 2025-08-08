#include "gtest/gtest.h"

#include "pid.h"
#include "plant.h"

TEST(PID, ZeroGainsProducesZeroOutput)
{
    PID pid;
    pid.set_gains(0.0, 0.0, 0.0);
    pid.set_setpoint(1.0);
    pid.set_output_limits(-100.0, 100.0);
    double u = pid.update(0.0, 0.01);
    EXPECT_DOUBLE_EQ(u, 0.0);
}

TEST(PID, ProportionalOnlyRespondsToError)
{
    PID pid;
    pid.set_gains(2.0, 0.0, 0.0);
    pid.set_setpoint(1.0);
    pid.set_output_limits(-100.0, 100.0);

    // y=0 -> error=1 -> u = Kp*e = 2
    double u1 = pid.update(0.0, 0.01);
    EXPECT_NEAR(u1, 2.0, 1e-12);

    // y=0.5 -> error=0.5 -> u = 1
    double u2 = pid.update(0.5, 0.01);
    EXPECT_NEAR(u2, 1.0, 1e-12);
}

TEST(PID, IntegratorEliminatesSteadyStateError)
{
    PID pid;
    pid.set_gains(0.0, 1.0, 0.0); // pure I
    pid.set_setpoint(1.0);
    pid.set_output_limits(-100.0, 100.0);

    // With constant error e=1, output should grow linearly: u = ki * integral(e dt)
    double u = 0.0;
    double y = 0.0; // irrelevant for pure I, but e = 1 - 0 = 1
    for (int k = 0; k < 100; ++k)
    {
        u = pid.update(y, 0.01);
    }
    // After 1s total time, u ~= ki * 1s * 1 = 1
    EXPECT_NEAR(u, 1.0, 1e-2);
}

TEST(PID, DerivativeOnMeasurementZeroOnFirstStep)
{
    PID pid;
    pid.set_gains(0.0, 0.0, 1.0);
    pid.set_setpoint(0.0);

    // First update uses first_ flag; derivative term should be zero
    double u1 = pid.update(0.0, 0.01);
    EXPECT_DOUBLE_EQ(u1, 0.0);

    // Now change measurement; derivative should react
    double u2 = pid.update(1.0, 0.01);
    // d_meas = (y_prev - y)/dt = (0 - 1)/0.01 = -100 -> D = -100
    EXPECT_NEAR(u2, -100.0, 1e-9);
}

TEST(Plant, FirstOrderStepsTowardKTimesU)
{
    FirstOrderPlant plant(2.0, 1.0, 0.0);
    const double dt = 0.01;
    const double u = 1.0;
    // Over time, y should approach K*u = 2.0
    double y = 0.0;
    for (int k = 0; k < 1000; ++k)
    {
        y = plant.step(u, dt);
    }
    EXPECT_NEAR(y, 2.0, 1e-2);
}

TEST(Plant, SecondOrderConvergesToKOverKspringTimesU)
{
    // For second-order: steady-state x_ss = (K / k) * u
    const double m = 1.0;
    const double b = 0.8;
    const double kspring = 5.0;
    const double K = 2.0;
    SecondOrderPlant plant(m, b, kspring, K, 0.0, 0.0);

    const double dt = 0.001; // smaller step for better integration
    const double u = 1.0;
    const double expected = (K / kspring) * u; // = 0.4

    double y = 0.0;
    for (int k = 0; k < 20000; ++k)
    { // simulate 20 seconds
        y = plant.step(u, dt);
    }

    // Expect close to steady-state value
    EXPECT_NEAR(y, expected, 5e-3);
}

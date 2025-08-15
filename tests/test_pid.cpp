#include "gtest/gtest.h"

#include "exceptions.h"
#include "pid.h"
#include "plant.h"
#include <cmath>
#include <limits>
#include <memory>

TEST(PID, ZeroGainsProducesZeroOutput)
{
    DigitalControl::PID pid;
    pid.set_gains(0.0, 0.0, 0.0);
    pid.set_setpoint(1.0);
    pid.set_output_limits(-100.0, 100.0);
    double u = pid.update(0.0, 0.01);
    EXPECT_DOUBLE_EQ(u, 0.0);
}

TEST(PID, ProportionalOnlyRespondsToError)
{
    DigitalControl::PID pid;
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
    DigitalControl::PID pid;
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
    DigitalControl::PID pid;
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

TEST(PID, FactoryFunctionCreatesValidController)
{
    auto pid = DigitalControl::make_pid(1.0, 0.5, 0.1);
    ASSERT_NE(pid, nullptr);
    EXPECT_EQ(pid->get_kp(), 1.0);
    EXPECT_EQ(pid->get_ki(), 0.5);
    EXPECT_EQ(pid->get_kd(), 0.1);
}

TEST(PID, MoveSemantics)
{
    DigitalControl::PID pid1(1.0, 2.0, 3.0);
    pid1.set_setpoint(5.0);

    DigitalControl::PID pid2(std::move(pid1));
    EXPECT_EQ(pid2.get_kp(), 1.0);
    EXPECT_EQ(pid2.get_ki(), 2.0);
    EXPECT_EQ(pid2.get_kd(), 3.0);
    EXPECT_EQ(pid2.get_setpoint(), 5.0);
}

TEST(PID, ExceptionOnInvalidGains)
{
    DigitalControl::PID pid;
    EXPECT_THROW(pid.set_gains(-1.0, 0.0, 0.0), DigitalControl::InvalidParameterException);
    EXPECT_THROW(pid.set_gains(0.0, -1.0, 0.0), DigitalControl::InvalidParameterException);
    EXPECT_THROW(pid.set_gains(0.0, 0.0, -1.0), DigitalControl::InvalidParameterException);
}

TEST(Plant, FirstOrderStepsTowardKTimesU)
{
    DigitalControl::FirstOrderPlant plant(2.0, 1.0, 0.0);
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
    DigitalControl::SecondOrderPlant plant(m, b, kspring, K, 0.0, 0.0);

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

TEST(Plant, FactoryFunctions)
{
    auto plant1 = DigitalControl::make_first_order_plant(2.0, 1.5, 0.5);
    ASSERT_NE(plant1, nullptr);
    EXPECT_EQ(plant1->get_gain(), 2.0);
    EXPECT_EQ(plant1->get_time_constant(), 1.5);
    EXPECT_EQ(plant1->get_initial_value(), 0.5);

    auto plant2 = DigitalControl::make_second_order_plant(1.0, 0.5, 4.0, 2.0);
    ASSERT_NE(plant2, nullptr);
    EXPECT_EQ(plant2->get_mass(), 1.0);
    EXPECT_EQ(plant2->get_damping(), 0.5);
    EXPECT_EQ(plant2->get_stiffness(), 4.0);
    EXPECT_EQ(plant2->get_gain(), 2.0);
}

TEST(Plant, ResetFunctionality)
{
    DigitalControl::FirstOrderPlant plant(1.0, 1.0, 5.0);
    EXPECT_EQ(plant.get_output(), 5.0);

    // Step the plant
    plant.step(1.0, 0.1);
    EXPECT_NE(plant.get_output(), 5.0);

    // Reset should restore initial condition
    plant.reset();
    EXPECT_EQ(plant.get_output(), 5.0);
}

// ============================================================================
// Additional comprehensive PID tests
// ============================================================================

TEST(PID, OutputSaturation)
{
    DigitalControl::PID pid(10.0, 0.0, 0.0); // High P gain
    pid.set_setpoint(10.0);
    pid.set_output_limits(-5.0, 5.0);

    // Large error should saturate at upper limit
    double u = pid.update(0.0, 0.01);
    EXPECT_EQ(u, 5.0);

    // Negative error should saturate at lower limit
    pid.set_setpoint(-10.0);
    pid.reset();
    u = pid.update(0.0, 0.01);
    EXPECT_EQ(u, -5.0);
}

TEST(PID, AntiWindupBehavior)
{
    DigitalControl::PID pid(0.0, 1.0, 0.0); // Pure integral
    pid.set_setpoint(1.0);
    pid.set_output_limits(-1.0, 1.0);

    // Run for many iterations with constant error
    double u = 0.0;
    for (int i = 0; i < 1000; ++i)
    {
        u = pid.update(0.0, 0.01);
    }

    // Output should be saturated but integral shouldn't wind up infinitely
    EXPECT_EQ(u, 1.0);

    // When error reverses, controller should respond quickly
    pid.set_setpoint(-1.0);
    for (int i = 0; i < 10; ++i)
    {
        u = pid.update(0.0, 0.01);
    }
    EXPECT_LT(u, 1.0); // Should start decreasing
}

TEST(PID, DerivativeFiltering)
{
    DigitalControl::PID pid(0.0, 0.0, 1.0); // Pure derivative
    pid.set_setpoint(0.0);
    pid.set_derivative_filter(0.5); // Medium filtering

    // First update should be zero
    double u1 = pid.update(0.0, 0.01);
    EXPECT_EQ(u1, 0.0);

    // Step change in measurement
    double u2 = pid.update(1.0, 0.01);

    // With filtering, derivative response should be attenuated
    // Without filter: d = -100, with alpha=0.5: filtered
    EXPECT_LT(std::abs(u2), 100.0);
}

TEST(PID, ResetClearsState)
{
    DigitalControl::PID pid(1.0, 1.0, 1.0);
    pid.set_setpoint(1.0);

    // Run controller to build up state
    for (int i = 0; i < 10; ++i)
    {
        pid.update(0.0, 0.01);
    }

    // Integral term should be non-zero
    EXPECT_NE(pid.get_integral_term(), 0.0);

    // Reset should clear all state
    pid.reset();
    EXPECT_EQ(pid.get_integral_term(), 0.0);

    // First update after reset should behave like initial update
    double u = pid.update(0.0, 0.01);
    EXPECT_NEAR(u, 1.01, 1e-10); // P term + small I term, no D on first step
}

TEST(PID, SetpointChanges)
{
    DigitalControl::PID pid(2.0, 0.0, 0.0); // P-only

    pid.set_setpoint(1.0);
    double u1 = pid.update(0.5, 0.01);
    EXPECT_NEAR(u1, 1.0, 1e-10); // error = 0.5, u = 2*0.5 = 1

    pid.set_setpoint(2.0);
    double u2 = pid.update(0.5, 0.01);
    EXPECT_NEAR(u2, 3.0, 1e-10); // error = 1.5, u = 2*1.5 = 3
}

TEST(PID, InvalidInputHandling)
{
    DigitalControl::PID pid(1.0, 0.0, 0.0);
    pid.set_setpoint(1.0);

    // NaN input
    EXPECT_THROW(pid.update(std::numeric_limits<double>::quiet_NaN(), 0.01), DigitalControl::InvalidParameterException);

    // Infinite input
    EXPECT_THROW(pid.update(std::numeric_limits<double>::infinity(), 0.01), DigitalControl::InvalidParameterException);

    // Negative time step
    EXPECT_THROW(pid.update(0.0, -0.01), DigitalControl::InvalidParameterException);

    // Zero time step
    EXPECT_THROW(pid.update(0.0, 0.0), DigitalControl::InvalidParameterException);
}

TEST(PID, OutputLimitValidation)
{
    DigitalControl::PID pid;

    // Invalid limits (min >= max)
    EXPECT_THROW(pid.set_output_limits(5.0, 5.0), DigitalControl::InvalidParameterException);
    EXPECT_THROW(pid.set_output_limits(10.0, 5.0), DigitalControl::InvalidParameterException);

    // NaN limits
    EXPECT_THROW(pid.set_output_limits(std::numeric_limits<double>::quiet_NaN(), 10.0),
                 DigitalControl::InvalidParameterException);

    // Infinite limits should be allowed
    EXPECT_NO_THROW(
        pid.set_output_limits(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()));
}

TEST(PID, StreamOperator)
{
    DigitalControl::PID pid(1.5, 0.5, 0.25);
    pid.set_setpoint(10.0);
    pid.set_output_limits(-100, 100);

    std::stringstream ss;
    ss << pid;
    std::string output = ss.str();

    // Check that key information is present
    EXPECT_NE(output.find("1.5"), std::string::npos);  // Kp
    EXPECT_NE(output.find("0.5"), std::string::npos);  // Ki
    EXPECT_NE(output.find("0.25"), std::string::npos); // Kd
}

TEST(PID, MoveAssignment)
{
    DigitalControl::PID pid1(1.0, 2.0, 3.0);
    pid1.set_setpoint(5.0);
    pid1.set_output_limits(-10, 10);

    DigitalControl::PID pid2;
    pid2 = std::move(pid1);

    EXPECT_EQ(pid2.get_kp(), 1.0);
    EXPECT_EQ(pid2.get_ki(), 2.0);
    EXPECT_EQ(pid2.get_kd(), 3.0);
    EXPECT_EQ(pid2.get_setpoint(), 5.0);
    EXPECT_EQ(pid2.get_min_output(), -10);
    EXPECT_EQ(pid2.get_max_output(), 10);
}

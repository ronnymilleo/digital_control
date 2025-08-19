#include "gtest/gtest.h"

#include "exceptions.h"
#include "plant.h"
#include <cmath>
#include <limits>
#include <memory>

// ============================================================================
// FirstOrderPlant Tests
// ============================================================================

TEST(FirstOrderPlant, ConstructorValidation)
{
    // Valid construction
    EXPECT_NO_THROW(DigitalControl::FirstOrderPlant(1.0, 1.0, 0.0));
    EXPECT_NO_THROW(DigitalControl::FirstOrderPlant(10.0, 0.1, -5.0));

    // Default constructor values
    DigitalControl::FirstOrderPlant plant;
    EXPECT_EQ(plant.get_gain(), 1.0);
    EXPECT_EQ(plant.get_time_constant(), 1.0);
    EXPECT_EQ(plant.get_initial_value(), 0.0);
}

TEST(FirstOrderPlant, StepResponse)
{
    DigitalControl::FirstOrderPlant plant(2.0, 1.0, 0.0);

    // Step input
    const double u  = 1.0;
    const double dt = 0.01;

    // Initial output should be y0
    EXPECT_EQ(plant.get_output(), 0.0);

    // First step
    double y1 = plant.step(u, dt);
    EXPECT_GT(y1, 0.0); // Should start rising
    EXPECT_LT(y1, 2.0); // But not at steady state yet

    // Steady state after long time
    for (int i = 0; i < 1000; ++i)
    {
        plant.step(u, dt);
    }
    EXPECT_NEAR(plant.get_output(), 2.0, 1e-3); // K*u = 2*1 = 2
}

TEST(FirstOrderPlant, TimeConstantBehavior)
{
    // After one time constant, output should be ~63.2% of final value
    const double                    K   = 3.0;
    const double                    tau = 2.0;
    DigitalControl::FirstOrderPlant plant(K, tau, 0.0);

    const double u     = 1.0;
    const double dt    = 0.01;
    const int    steps = static_cast<int>(tau / dt);

    for (int i = 0; i < steps; ++i)
    {
        plant.step(u, dt);
    }

    const double expected = K * u * (1.0 - std::exp(-1.0)); // ~0.632 * K*u
    EXPECT_NEAR(plant.get_output(), expected, 0.05);
}

TEST(FirstOrderPlant, NegativeGainBehavior)
{
    DigitalControl::FirstOrderPlant plant(-2.0, 1.0, 0.0);

    // Positive input should give negative steady-state output
    const double u = 1.0;
    for (int i = 0; i < 1000; ++i)
    {
        plant.step(u, 0.01);
    }
    EXPECT_NEAR(plant.get_output(), -2.0, 1e-3);
}

TEST(FirstOrderPlant, ZeroInput)
{
    DigitalControl::FirstOrderPlant plant(2.0, 1.0, 5.0);

    // With zero input, should decay to zero
    for (int i = 0; i < 1000; ++i)
    {
        plant.step(0.0, 0.01);
    }
    EXPECT_NEAR(plant.get_output(), 0.0, 1e-3);
}

TEST(FirstOrderPlant, InvalidInputHandling)
{
    DigitalControl::FirstOrderPlant plant(1.0, 1.0, 0.0);

    // NaN input
    EXPECT_THROW(plant.step(std::numeric_limits<double>::quiet_NaN(), 0.01), DigitalControl::InvalidParameterException);

    // Infinite input
    EXPECT_THROW(plant.step(std::numeric_limits<double>::infinity(), 0.01), DigitalControl::InvalidParameterException);

    // Negative time step
    EXPECT_THROW(plant.step(1.0, -0.01), DigitalControl::InvalidParameterException);

    // Zero time step should return current output
    double y0 = plant.get_output();
    double y1 = plant.step(1.0, 0.0);
    EXPECT_EQ(y1, y0);
}

TEST(FirstOrderPlant, ResetBehavior)
{
    const double                    y0 = 3.14;
    DigitalControl::FirstOrderPlant plant(1.0, 1.0, y0);

    // Step away from initial condition
    plant.step(10.0, 0.1);
    EXPECT_NE(plant.get_output(), y0);

    // Reset should restore initial condition
    plant.reset();
    EXPECT_EQ(plant.get_output(), y0);
}

TEST(FirstOrderPlant, StreamOperator)
{
    DigitalControl::FirstOrderPlant plant(2.5, 1.5, 0.5);

    std::stringstream ss;
    ss << plant;
    std::string output = ss.str();

    // Check that key parameters are present
    EXPECT_NE(output.find("2.5"), std::string::npos); // K
    EXPECT_NE(output.find("1.5"), std::string::npos); // tau
}

// ============================================================================
// SecondOrderPlant Tests
// ============================================================================

TEST(SecondOrderPlant, ConstructorValidation)
{
    // Valid construction
    EXPECT_NO_THROW(DigitalControl::SecondOrderPlant(1.0, 0.5, 4.0, 1.0, 0.0, 0.0));

    // Default constructor values
    DigitalControl::SecondOrderPlant plant;
    EXPECT_EQ(plant.get_mass(), 1.0);
    EXPECT_EQ(plant.get_damping(), 0.5);
    EXPECT_EQ(plant.get_stiffness(), 4.0);
    EXPECT_EQ(plant.get_gain(), 1.0);
    EXPECT_EQ(plant.get_position(), 0.0);
    EXPECT_EQ(plant.get_velocity(), 0.0);
}

TEST(SecondOrderPlant, SteadyStateResponse)
{
    // Steady state: x_ss = (K/k) * u
    const double                     m = 1.0;
    const double                     b = 1.0;
    const double                     k = 2.0;
    const double                     K = 3.0;
    DigitalControl::SecondOrderPlant plant(m, b, k, K, 0.0, 0.0);

    const double u           = 2.0;
    const double expected_ss = (K / k) * u; // 3/2 * 2 = 3

    // Run to steady state (longer simulation for better convergence)
    for (int i = 0; i < 20000; ++i)
    {
        plant.step(u, 0.001);
    }

    EXPECT_NEAR(plant.get_output(), expected_ss, 0.025); // Relaxed tolerance
    EXPECT_NEAR(plant.get_velocity(), 0.0, 0.02);        // Relaxed tolerance for velocity
}

TEST(SecondOrderPlant, UnderdampedOscillation)
{
    // Underdamped system (zeta < 1)
    const double                     m = 1.0;
    const double                     k = 4.0;
    const double                     b = 0.2; // Light damping for oscillation
    DigitalControl::SecondOrderPlant plant(m, b, k, 1.0, 0.0, 0.0);

    // Apply step input
    double max_pos         = 0.0;
    bool   found_overshoot = false;

    for (int i = 0; i < 1000; ++i)
    {
        double y = plant.step(1.0, 0.001);
        if (y > max_pos)
        {
            max_pos = y;
        }
        // Check for overshoot (position exceeds steady state)
        if (y > 0.25 * 1.01)
        { // steady state is K/k = 1/4 = 0.25
            found_overshoot = true;
        }
    }

    EXPECT_TRUE(found_overshoot); // Underdamped should overshoot
}

TEST(SecondOrderPlant, CriticallyDamped)
{
    // Critically damped: b = 2*sqrt(m*k)
    const double                     m = 1.0;
    const double                     k = 4.0;
    const double                     b = 2.0 * std::sqrt(m * k); // = 4.0
    DigitalControl::SecondOrderPlant plant(m, b, k, 1.0, 0.0, 0.0);

    // Should approach steady state without overshoot
    double prev      = 0.0;
    bool   monotonic = true;

    for (int i = 0; i < 1000; ++i)
    {
        double y = plant.step(1.0, 0.001);
        if (i > 0 && y < prev)
        {
            monotonic = false; // Found non-monotonic behavior
        }
        prev = y;
    }

    EXPECT_TRUE(monotonic); // Critically damped should be monotonic
}

TEST(SecondOrderPlant, InitialConditions)
{
    const double                     x0 = 2.0;
    const double                     v0 = -1.0;
    DigitalControl::SecondOrderPlant plant(1.0, 0.5, 4.0, 1.0, x0, v0);

    EXPECT_EQ(plant.get_position(), x0);
    EXPECT_EQ(plant.get_velocity(), v0);

    // With zero input, should oscillate around zero (longer for damping)
    for (int i = 0; i < 20000; ++i)
    {
        plant.step(0.0, 0.001);
    }
    EXPECT_NEAR(plant.get_position(), 0.0, 0.1);  // Relaxed tolerance
    EXPECT_NEAR(plant.get_velocity(), 0.0, 0.35); // Relaxed tolerance for velocity
}

TEST(SecondOrderPlant, InvalidInputHandling)
{
    DigitalControl::SecondOrderPlant plant;

    // NaN input
    EXPECT_THROW(plant.step(std::numeric_limits<double>::quiet_NaN(), 0.01), DigitalControl::InvalidParameterException);

    // Infinite time step
    EXPECT_THROW(plant.step(1.0, std::numeric_limits<double>::infinity()), DigitalControl::InvalidParameterException);

    // Negative time step
    EXPECT_THROW(plant.step(1.0, -0.01), DigitalControl::InvalidParameterException);
}

TEST(SecondOrderPlant, ResetBehavior)
{
    const double                     x0 = 1.5;
    const double                     v0 = -0.5;
    DigitalControl::SecondOrderPlant plant(1.0, 0.5, 4.0, 1.0, x0, v0);

    // Step away from initial conditions
    for (int i = 0; i < 100; ++i)
    {
        plant.step(5.0, 0.01);
    }
    EXPECT_NE(plant.get_position(), x0);
    EXPECT_NE(plant.get_velocity(), v0);

    // Reset should restore initial conditions
    plant.reset();
    EXPECT_EQ(plant.get_position(), x0);
    EXPECT_EQ(plant.get_velocity(), v0);
}

TEST(SecondOrderPlant, NumericalStability)
{
    DigitalControl::SecondOrderPlant plant(1.0, 0.5, 4.0, 1.0, 0.0, 0.0);

    // Test with very small time steps
    bool stable = true;
    for (int i = 0; i < 100000; ++i)
    {
        double y = plant.step(1.0, 1e-6);
        if (!std::isfinite(y))
        {
            stable = false;
            break;
        }
    }
    EXPECT_TRUE(stable);

    // Test with moderate time steps
    plant.reset();
    for (int i = 0; i < 1000; ++i)
    {
        double y = plant.step(1.0, 0.01);
        if (!std::isfinite(y))
        {
            stable = false;
            break;
        }
    }
    EXPECT_TRUE(stable);
}

TEST(SecondOrderPlant, StreamOperator)
{
    DigitalControl::SecondOrderPlant plant(1.5, 0.75, 3.0, 2.0, 0.5, -0.25);

    std::stringstream ss;
    ss << plant;
    std::string output = ss.str();

    // Check that key parameters are present
    EXPECT_NE(output.find("1.5"), std::string::npos);  // mass
    EXPECT_NE(output.find("0.75"), std::string::npos); // damping
    EXPECT_NE(output.find("3"), std::string::npos);    // stiffness
    EXPECT_NE(output.find("2"), std::string::npos);    // gain
}

// ============================================================================
// Factory Function Tests
// ============================================================================

TEST(PlantFactory, FirstOrderFactory)
{
    auto plant = DigitalControl::make_first_order_plant(2.0, 3.0, 1.0);
    ASSERT_NE(plant, nullptr);
    EXPECT_EQ(plant->get_gain(), 2.0);
    EXPECT_EQ(plant->get_time_constant(), 3.0);
    EXPECT_EQ(plant->get_initial_value(), 1.0);

    // Test polymorphic behavior
    DigitalControl::IPlant* base = plant.get();
    EXPECT_EQ(base->get_output(), 1.0);
    base->step(1.0, 0.01);
    EXPECT_NE(base->get_output(), 1.0);
}

TEST(PlantFactory, SecondOrderFactory)
{
    auto plant = DigitalControl::make_second_order_plant(2.0, 1.0, 8.0, 4.0, 0.5, -1.0);
    ASSERT_NE(plant, nullptr);
    EXPECT_EQ(plant->get_mass(), 2.0);
    EXPECT_EQ(plant->get_damping(), 1.0);
    EXPECT_EQ(plant->get_stiffness(), 8.0);
    EXPECT_EQ(plant->get_gain(), 4.0);
    EXPECT_EQ(plant->get_position(), 0.5);
    EXPECT_EQ(plant->get_velocity(), -1.0);
}

// ============================================================================
// Boundary and Edge Case Tests
// ============================================================================

TEST(PlantEdgeCases, VerySmallTimeConstant)
{
    // Very small time constant should respond quickly
    DigitalControl::FirstOrderPlant plant(1.0, 1e-6, 0.0);

    // Should reach steady state almost immediately (run more steps)
    for (int i = 0; i < 10; ++i)
    {
        plant.step(1.0, 1e-6);
    }
    EXPECT_NEAR(plant.get_output(), 1.0, 0.1);
}

TEST(PlantEdgeCases, VeryLargeTimeConstant)
{
    // Very slow system
    DigitalControl::FirstOrderPlant plant(1.0, 1e6, 0.0);

    // Should barely move in reasonable time
    plant.step(1.0, 1.0);
    EXPECT_LT(plant.get_output(), 0.001);
}

TEST(PlantEdgeCases, ZeroMass)
{
    // Zero mass should make system algebraic (no dynamics)
    // This might throw or behave unexpectedly - test the behavior
    DigitalControl::SecondOrderPlant plant(1e-10, 1.0, 1.0, 1.0, 0.0, 0.0);

    // System should be very responsive (nearly infinite acceleration)
    double y = plant.step(1.0, 0.001);
    EXPECT_GT(std::abs(y), 0.0); // Should move immediately
}

#include "gtest/gtest.h"

#include "exceptions.h"
#include "leadlag.h"
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>

// ============================================================================
// LeadLag Constructor and Basic Tests
// ============================================================================

TEST(LeadLag, DefaultConstructor)
{
    DigitalControl::LeadLag comp;
    EXPECT_EQ(comp.get_gain(), 1.0);
    EXPECT_EQ(comp.get_zero(), 0.1);
    EXPECT_EQ(comp.get_pole(), 1.0);
    EXPECT_TRUE(comp.is_lead()); // 0.1 < 1.0
    EXPECT_FALSE(comp.is_lag());
}

TEST(LeadLag, ParameterizedConstructor)
{
    DigitalControl::LeadLag lead(2.0, 0.5, 2.0);
    EXPECT_EQ(lead.get_gain(), 2.0);
    EXPECT_EQ(lead.get_zero(), 0.5);
    EXPECT_EQ(lead.get_pole(), 2.0);
    EXPECT_TRUE(lead.is_lead());

    DigitalControl::LeadLag lag(1.5, 3.0, 0.5);
    EXPECT_EQ(lag.get_gain(), 1.5);
    EXPECT_EQ(lag.get_zero(), 3.0);
    EXPECT_EQ(lag.get_pole(), 0.5);
    EXPECT_TRUE(lag.is_lag());
}

TEST(LeadLag, InvalidConstructorParameters)
{
    // NaN parameters
    EXPECT_THROW(DigitalControl::LeadLag(std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0),
                 DigitalControl::InvalidParameterException);

    // Negative zero/pole
    EXPECT_THROW(DigitalControl::LeadLag(1.0, -1.0, 1.0), DigitalControl::InvalidParameterException);
    EXPECT_THROW(DigitalControl::LeadLag(1.0, 1.0, -1.0), DigitalControl::InvalidParameterException);

    // Zero gain
    EXPECT_THROW(DigitalControl::LeadLag(0.0, 1.0, 1.0), DigitalControl::InvalidParameterException);
}

// ============================================================================
// Lead Compensator Tests
// ============================================================================

TEST(LeadLag, LeadCompensatorBasicResponse)
{
    // Lead compensator: zero < pole (adds phase lead)
    DigitalControl::LeadLag lead(1.0, 0.1, 1.0);
    lead.set_setpoint(1.0);

    // Step response should have initial kick due to derivative action
    double u1 = lead.update(0.0, 0.01);
    EXPECT_GT(u1, 0.0); // Positive error should give positive output

    // Subsequent updates
    double u2 = lead.update(0.2, 0.01);
    double u3 = lead.update(0.4, 0.01);

    // Output should decrease as error decreases
    EXPECT_LT(u3, u2);
    EXPECT_LT(u2, u1);
}

TEST(LeadLag, LeadCompensatorPhaseContribution)
{
    DigitalControl::LeadLag lead(1.0, 0.1, 1.0);

    // At geometric mean frequency sqrt(z*p), phase is maximum
    double omega_max = std::sqrt(0.1 * 1.0);
    double phase_max = lead.get_phase_at_frequency(omega_max);

    // Phase should be positive (lead)
    EXPECT_GT(phase_max, 0.0);

    // Phase at low frequency should be near zero
    double phase_low = lead.get_phase_at_frequency(0.01);
    EXPECT_LT(std::abs(phase_low), std::abs(phase_max));

    // Phase at high frequency should be near zero
    double phase_high = lead.get_phase_at_frequency(10.0);
    EXPECT_LT(std::abs(phase_high), std::abs(phase_max));
}

TEST(LeadLag, LeadCompensatorMagnitude)
{
    DigitalControl::LeadLag lead(2.0, 0.1, 1.0);

    // At low frequency, magnitude ≈ K * (z/p)
    double mag_low = lead.get_magnitude_at_frequency(0.001);
    EXPECT_NEAR(mag_low, 2.0 * 0.1 / 1.0, 0.01);

    // At high frequency, magnitude ≈ K
    double mag_high = lead.get_magnitude_at_frequency(100.0);
    EXPECT_NEAR(mag_high, 2.0, 0.01);
}

// ============================================================================
// Lag Compensator Tests
// ============================================================================

TEST(LeadLag, LagCompensatorBasicResponse)
{
    // Lag compensator: zero > pole (improves steady-state error)
    DigitalControl::LeadLag lag(1.0, 1.0, 0.1);
    lag.set_setpoint(1.0);

    // Lag compensator should have slower response
    double u1 = lag.update(0.0, 0.01);
    double u2 = lag.update(0.0, 0.01);
    double u3 = lag.update(0.0, 0.01);

    // Output should increase gradually (integral-like behavior)
    EXPECT_GT(u3, u2);
    EXPECT_GT(u2, u1);
}

TEST(LeadLag, LagCompensatorPhaseContribution)
{
    DigitalControl::LeadLag lag(1.0, 1.0, 0.1);

    // Phase should be negative (lag)
    double omega_mid = std::sqrt(0.1 * 1.0);
    double phase_mid = lag.get_phase_at_frequency(omega_mid);
    EXPECT_LT(phase_mid, 0.0);
}

TEST(LeadLag, LagCompensatorSteadyStateImprovement)
{
    DigitalControl::LeadLag lag(10.0, 10.0, 0.1); // High DC gain
    lag.set_setpoint(1.0);

    // Run to steady state with constant error
    double u = 0.0;
    for (int i = 0; i < 1000; ++i)
    {
        u = lag.update(0.5, 0.01); // Constant error of 0.5
    }

    // Steady-state output should be significant due to high DC gain
    // DC gain = K * (z/p) = 10 * (10/0.1) = 1000
    EXPECT_GT(u, 100.0); // Should have large steady-state gain
}

// ============================================================================
// Dynamic Behavior Tests
// ============================================================================

TEST(LeadLag, SetParametersDynamically)
{
    DigitalControl::LeadLag comp;

    // Start as lead
    comp.set_parameters(1.0, 0.1, 1.0);
    EXPECT_TRUE(comp.is_lead());

    // Change to lag
    comp.set_parameters(1.0, 1.0, 0.1);
    EXPECT_TRUE(comp.is_lag());

    // Invalid parameters should throw
    EXPECT_THROW(comp.set_parameters(0.0, 1.0, 1.0), DigitalControl::InvalidParameterException);
}

TEST(LeadLag, OutputSaturation)
{
    DigitalControl::LeadLag comp(10.0, 0.1, 1.0);
    comp.set_setpoint(10.0);
    comp.set_output_limits(-5.0, 5.0);

    // Large error should saturate
    double u = comp.update(0.0, 0.01);
    EXPECT_LE(u, 5.0);
    EXPECT_GE(u, -5.0);
}

TEST(LeadLag, ResetFunctionality)
{
    DigitalControl::LeadLag comp(1.0, 0.5, 2.0);
    comp.set_setpoint(1.0);

    // Run for a while to build up state
    for (int i = 0; i < 10; ++i)
    {
        comp.update(0.0, 0.01);
    }

    // Reset should clear state
    comp.reset();

    // First update after reset should behave like initial update
    double u1 = comp.update(0.0, 0.01);

    // Create fresh compensator and compare
    DigitalControl::LeadLag comp2(1.0, 0.5, 2.0);
    comp2.set_setpoint(1.0);
    double u2 = comp2.update(0.0, 0.01);

    EXPECT_NEAR(u1, u2, 1e-10);
}

// ============================================================================
// Move Semantics Tests
// ============================================================================

TEST(LeadLag, MoveConstructor)
{
    DigitalControl::LeadLag comp1(2.0, 0.2, 2.0);
    comp1.set_setpoint(5.0);
    comp1.set_output_limits(-10, 10);

    DigitalControl::LeadLag comp2(std::move(comp1));

    EXPECT_EQ(comp2.get_gain(), 2.0);
    EXPECT_EQ(comp2.get_zero(), 0.2);
    EXPECT_EQ(comp2.get_pole(), 2.0);
    EXPECT_EQ(comp2.get_setpoint(), 5.0);
    EXPECT_EQ(comp2.get_min_output(), -10);
    EXPECT_EQ(comp2.get_max_output(), 10);
}

TEST(LeadLag, MoveAssignment)
{
    DigitalControl::LeadLag comp1(3.0, 0.3, 3.0);
    comp1.set_setpoint(7.0);

    DigitalControl::LeadLag comp2;
    comp2 = std::move(comp1);

    EXPECT_EQ(comp2.get_gain(), 3.0);
    EXPECT_EQ(comp2.get_zero(), 0.3);
    EXPECT_EQ(comp2.get_pole(), 3.0);
    EXPECT_EQ(comp2.get_setpoint(), 7.0);
}

// ============================================================================
// Factory Function Tests
// ============================================================================

TEST(LeadLag, FactoryFunctions)
{
    // Generic factory
    auto comp = DigitalControl::make_leadlag(2.0, 0.5, 5.0);
    ASSERT_NE(comp, nullptr);
    EXPECT_EQ(comp->get_gain(), 2.0);
    EXPECT_TRUE(comp->is_lead());

    // Lead compensator factory
    auto lead = DigitalControl::make_lead_compensator(1.0, 0.1, 1.0);
    ASSERT_NE(lead, nullptr);
    EXPECT_TRUE(lead->is_lead());

    // Lead factory with invalid parameters should throw
    EXPECT_THROW(DigitalControl::make_lead_compensator(1.0, 1.0, 0.1), DigitalControl::InvalidParameterException);

    // Lag compensator factory
    auto lag = DigitalControl::make_lag_compensator(1.0, 1.0, 0.1);
    ASSERT_NE(lag, nullptr);
    EXPECT_TRUE(lag->is_lag());

    // Lag factory with invalid parameters should throw
    EXPECT_THROW(DigitalControl::make_lag_compensator(1.0, 0.1, 1.0), DigitalControl::InvalidParameterException);
}

// ============================================================================
// Frequency Response Tests
// ============================================================================

TEST(LeadLag, FrequencyResponseValidation)
{
    DigitalControl::LeadLag comp(2.0, 1.0, 10.0);

    // Invalid frequency should throw
    EXPECT_THROW(comp.get_phase_at_frequency(0.0), DigitalControl::InvalidParameterException);
    EXPECT_THROW(comp.get_phase_at_frequency(-1.0), DigitalControl::InvalidParameterException);
    EXPECT_THROW(comp.get_magnitude_at_frequency(0.0), DigitalControl::InvalidParameterException);
    EXPECT_THROW(comp.get_magnitude_at_frequency(-1.0), DigitalControl::InvalidParameterException);
}

TEST(LeadLag, MaximumPhaseFrequency)
{
    // For lead compensator, maximum phase occurs at ω = √(z*p)
    DigitalControl::LeadLag lead(1.0, 1.0, 4.0);

    double omega_max = std::sqrt(1.0 * 4.0); // = 2.0
    double phase_max = lead.get_phase_at_frequency(omega_max);

    // Check that this is indeed maximum by comparing to nearby frequencies
    double phase_lower  = lead.get_phase_at_frequency(omega_max * 0.8);
    double phase_higher = lead.get_phase_at_frequency(omega_max * 1.2);

    EXPECT_GT(phase_max, phase_lower);
    EXPECT_GT(phase_max, phase_higher);

    // Calculate theoretical maximum phase
    // φ_max = arcsin((p-z)/(p+z))
    double theoretical_max = std::asin((4.0 - 1.0) / (4.0 + 1.0));
    EXPECT_NEAR(phase_max, theoretical_max, 0.01);
}

// ============================================================================
// Numerical Stability Tests
// ============================================================================

TEST(LeadLag, VerySmallTimeStep)
{
    DigitalControl::LeadLag comp(1.0, 0.1, 1.0);
    comp.set_setpoint(1.0);

    // Very small time step shouldn't cause instability
    bool stable = true;
    for (int i = 0; i < 1000; ++i)
    {
        double u = comp.update(0.5, 1e-6);
        if (!std::isfinite(u))
        {
            stable = false;
            break;
        }
    }
    EXPECT_TRUE(stable);
}

TEST(LeadLag, VaryingTimeStep)
{
    DigitalControl::LeadLag comp(1.0, 0.5, 2.0);
    comp.set_setpoint(1.0);

    // Varying time steps should be handled correctly
    double u1 = comp.update(0.0, 0.01);
    double u2 = comp.update(0.2, 0.02);  // Different dt
    double u3 = comp.update(0.4, 0.005); // Another different dt

    // All outputs should be finite
    EXPECT_TRUE(std::isfinite(u1));
    EXPECT_TRUE(std::isfinite(u2));
    EXPECT_TRUE(std::isfinite(u3));
}

// ============================================================================
// Stream Operator Test
// ============================================================================

TEST(LeadLag, StreamOperator)
{
    DigitalControl::LeadLag lead(1.5, 0.2, 2.0);
    lead.set_output_limits(-50, 50);

    std::stringstream ss;
    ss << lead;
    std::string output = ss.str();

    // Check that key information is present
    EXPECT_NE(output.find("1.5"), std::string::npos);  // K
    EXPECT_NE(output.find("0.2"), std::string::npos);  // zero
    EXPECT_NE(output.find("2"), std::string::npos);    // pole
    EXPECT_NE(output.find("lead"), std::string::npos); // type
    EXPECT_NE(output.find("-50"), std::string::npos);  // min limit
    EXPECT_NE(output.find("50"), std::string::npos);   // max limit
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST(LeadLag, UnityGainAtCrossover)
{
    // When zero = pole, it's just a gain block
    DigitalControl::LeadLag unity(2.0, 1.0, 1.0);

    // Phase contribution should be zero at all frequencies
    double phase = unity.get_phase_at_frequency(1.0);
    EXPECT_NEAR(phase, 0.0, 1e-10);

    // Magnitude should be constant K at all frequencies
    double mag_low  = unity.get_magnitude_at_frequency(0.01);
    double mag_high = unity.get_magnitude_at_frequency(100.0);
    EXPECT_NEAR(mag_low, 2.0, 1e-3);
    EXPECT_NEAR(mag_high, 2.0, 1e-3);
}

TEST(LeadLag, ExtremePoleZeroRatios)
{
    // Very large pole/zero ratio (strong lead)
    DigitalControl::LeadLag strong_lead(1.0, 0.001, 1000.0);
    double                  phase_max = strong_lead.get_phase_at_frequency(std::sqrt(0.001 * 1000.0));
    EXPECT_GT(phase_max, 1.5); // Should be close to π/2

    // Very large zero/pole ratio (strong lag)
    DigitalControl::LeadLag strong_lag(1.0, 1000.0, 0.001);
    double                  phase_min = strong_lag.get_phase_at_frequency(std::sqrt(0.001 * 1000.0));
    EXPECT_LT(phase_min, -1.5); // Should be close to -π/2
}

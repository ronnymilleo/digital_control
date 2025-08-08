#pragma once

// Generic plant interface
struct IPlant
{
    virtual ~IPlant() = default;
    // Advance the plant one step given input u and timestep dt; returns new output y
    virtual double step(double u, double dt) = 0;
};

// First-order plant: y' = (-(y - K*u))/tau
class FirstOrderPlant : public IPlant
{
  public:
    explicit FirstOrderPlant(double K = 1.0, double tau = 1.0, double y0 = 0.0) : K_(K), tau_(tau), y_(y0)
    {
    }

    double step(double u, double dt) override
    {
        if (dt <= 0.0)
            return y_;
        const double dydt = (-(y_ - K_ * u)) / tau_;
        y_ += dt * dydt;
        return y_;
    }

  private:
    double K_;
    double tau_;
    double y_;
};

// Second-order mass-spring-damper plant:
// m * x_dd + b * x_d + k * x = K * u
// Output y = x
class SecondOrderPlant : public IPlant
{
  public:
    explicit SecondOrderPlant(double m = 1.0, double b = 0.5, double k = 4.0, double K = 1.0, double x0 = 0.0,
                              double v0 = 0.0)
        : m_(m), b_(b), k_(k), K_(K), x_(x0), v_(v0)
    {
    }

    double step(double u, double dt) override
    {
        if (dt <= 0.0)
            return x_;
        // x_dd = (K*u - b*v - k*x) / m
        const double a = (K_ * u - b_ * v_ - k_ * x_) / m_;
        // Integrate (semi-implicit Euler for a bit more stability):
        v_ += dt * a;
        x_ += dt * v_;
        return x_;
    }

  private:
    double m_;
    double b_;
    double k_;
    double K_;
    double x_; // position (output)
    double v_; // velocity
};

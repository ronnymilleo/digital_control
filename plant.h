#pragma once

struct IPlant {
    virtual ~IPlant() = default;
    // Advance the plant one step given input u and timestep dt; returns new output y
    virtual double step(double u, double dt) = 0;
};

class FirstOrderPlant : public IPlant {
public:
    // y' = (-(y - K*u))/tau
    explicit FirstOrderPlant(double K = 1.0, double tau = 1.0, double y0 = 0.0)
        : K_(K), tau_(tau), y_(y0) {}

    double step(double u, double dt) override {
        if (dt <= 0.0) return y_;
        const double dydt = (-(y_ - K_ * u)) / tau_;
        y_ += dt * dydt;
        return y_;
    }

private:
    double K_;
    double tau_;
    double y_;
};


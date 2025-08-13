#include "plant.h"
#include <ostream>

namespace DigitalControl
{

std::ostream &operator<<(std::ostream &os, const FirstOrderPlant &plant)
{
    os << "FirstOrderPlant{K=" << plant.K_ << ", tau=" << plant.tau_ << ", y0=" << plant.y0_ << ", y=" << plant.y_
       << "}";
    return os;
}

std::ostream &operator<<(std::ostream &os, const SecondOrderPlant &plant)
{
    os << "SecondOrderPlant{m=" << plant.m_ << ", b=" << plant.b_ << ", k=" << plant.k_ << ", K=" << plant.K_
       << ", x=" << plant.x_ << ", v=" << plant.v_ << "}";
    return os;
}

} // namespace DigitalControl

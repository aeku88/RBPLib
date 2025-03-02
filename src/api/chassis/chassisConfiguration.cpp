#include "api/chassis/chassisConfiguration.hpp"

namespace rbplib 
{
    ChassisConfiguration::ChassisConfiguration(const inch_t iwheelDiameter,
                                               const inch_t itrackWidth,
                                               const pros::MotorGear &igearset,
                                               const double igearRatio)
    : wheelDiameter(iwheelDiameter), trackWidth(itrackWidth), gearset(igearset), gearRatio(igearRatio)
    {
    }
}
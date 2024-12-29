#pragma once

#include "api.h"
#include "api/units.h"
#include "Eigen/Core"

using namespace units::literals;

using namespace units;
using namespace units::length;
using namespace units::time;
using namespace units::velocity;
using namespace units::angular_velocity;


namespace rbplib
{
    class ChassisKinematics
    {
    public:
        ChassisKinematics(const pros::MotorGear &igearset,
                          const inch_t iwheelDiameter,
                          const inch_t itrackWidth);

        Eigen::Vector2<revolutions_per_minute_t> inverseKinematics(const std::pair<feet_per_second_t, radians_per_second_t> ichassisVelocities);

        std::pair<feet_per_second_t, radians_per_second_t> forwardKinematics(const Eigen::Vector2<revolutions_per_minute_t> imotorVelocities);
    private:
        pros::MotorGear gearset;

        radians_per_second_t maxMotorVel;
        inch_t wheelDiameter, trackWidth;
    };
} // namespace rbplib

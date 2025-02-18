#pragma once

#include "api/units.h"
#include "api.h"

#include <memory>

using namespace units::literals;
using namespace units::angular_velocity;

namespace rbplib
{
    class ChassisModel
    {
    public:
        ChassisModel(pros::MotorGroup &ileft,
                     pros::MotorGroup &iright,
                     pros::Rotation &imiddle);

        void drive(const revolutions_per_minute_t ileftVel, 
                   const revolutions_per_minute_t irightVel);

        std::valarray<std::int32_t> getSensorVals() const;

    private:
        pros::AbstractMotor &left;
        pros::AbstractMotor &right;
        pros::Rotation &middle;
    };
} // namespace rbplib

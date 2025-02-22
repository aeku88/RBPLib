#pragma once

#include "api/chassis/chassisConfiguration.hpp"
#include "api/units.h"
#include "api.h"

#include <memory>

using namespace units::literals;

using namespace units::velocity;
using namespace units::angular_velocity;

namespace tekuaek
{
    class ChassisModel
    {
    public:
        ChassisModel(pros::MotorGroup &ileft,
                     pros::MotorGroup &iright,
                     pros::Rotation &imiddle,
                     std::shared_ptr<ChassisConfiguration> &iconfig);

        void drive(const revolutions_per_minute_t ileftVel, 
                   const revolutions_per_minute_t irightVel);
                   
        std::pair<feet_per_second_t, radians_per_second_t> getMaxVelocity() const;

        std::valarray<std::int32_t> getSensorVals() const;

    private:
        pros::AbstractMotor &left;
        pros::AbstractMotor &right;
        pros::Rotation &middle;
        
        radians_per_second_t maxMotorVel;

        std::shared_ptr<ChassisConfiguration> config;
    };
} // namespace rbplib

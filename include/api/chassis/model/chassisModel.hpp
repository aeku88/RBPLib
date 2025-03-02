#pragma once

#include "api/chassis/chassisConfiguration.hpp"
#include "api/units.h"
#include "api.h"

#include <memory>
#include <api/chassis/kinematics/chassisKinematics.hpp>

using namespace units::literals;

using namespace units::velocity;
using namespace units::angular_velocity;

namespace rbplib
{
    class ChassisModel
    {
    public:
        ChassisModel(pros::MotorGroup &ileft,
                     pros::MotorGroup &iright,
                     pros::Rotation &imiddle,
                     std::shared_ptr<ChassisKinematics> &ikinematics,
                     std::shared_ptr<ChassisConfiguration> &iconfig);

        void drive(const Eigen::Vector2<revolutions_per_minute_t> &imotorVelocities);
        void drive(const std::pair<feet_per_second_t, radians_per_second_t> &ichassisVelocities);

        std::pair<feet_per_second_t, radians_per_second_t> getMaxVelocity() const;

        radians_per_second_t getMaxMotorVel() const { return maxMotorVel; }

        std::valarray<std::int32_t> getSensorVals() const;

    private:
        pros::AbstractMotor &left;
        pros::AbstractMotor &right;
        pros::Rotation &middle;
        
        radians_per_second_t maxMotorVel;

        std::shared_ptr<ChassisKinematics> kinematics;
        std::shared_ptr<ChassisConfiguration> config;
    };
} // namespace rbplib

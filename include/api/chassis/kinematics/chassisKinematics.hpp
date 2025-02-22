#pragma once

#include "api/chassis/chassisConfiguration.hpp"
#include "api.h"
#include "api/units.h" 
#include "Eigen/Core"

using namespace units::literals;

using namespace units;
using namespace units::length;
using namespace units::time;
using namespace units::velocity;
using namespace units::angular_velocity;


namespace tekuaek
{
    class ChassisKinematics
    {
    public:
        ChassisKinematics(std::shared_ptr<ChassisConfiguration> &iconfig);

        Eigen::Vector2<revolutions_per_minute_t> inverseKinematics(const std::pair<feet_per_second_t, radians_per_second_t> ichassisVelocities);

        std::pair<feet_per_second_t, radians_per_second_t> forwardKinematics(const Eigen::Vector2<revolutions_per_minute_t> imotorVelocities);
    private:
        std::shared_ptr<ChassisConfiguration> config;
    };
} // namespace rbplib

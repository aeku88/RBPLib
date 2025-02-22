#include "api/chassis/kinematics/chassisKinematics.hpp"
#include "api/units.h"

namespace tekuaek
{
    ChassisKinematics::ChassisKinematics(std::shared_ptr<ChassisConfiguration> &iconfig)
    : config(iconfig)
    {
        
    }

    template <typename T> int sgn(T val) 
    {
        return (T(0) < val) - (val < T(0));
    }

    Eigen::Vector2<revolutions_per_minute_t> ChassisKinematics::inverseKinematics(const std::pair<feet_per_second_t, radians_per_second_t> ichassisVelocities)
    {
        feet_per_second_t linVel = ichassisVelocities.first;
        radians_per_second_t angVel = ichassisVelocities.second;

        radians_per_second_t leftVel = (linVel * 1_rad + (config->getTrackWidth() / 2) * angVel) / (config->getWheelDiameter() / 2); // Multiplied linear velocity by 1 radian to match units match; in units library, angle has dimension.
        radians_per_second_t rightVel = (linVel * 1_rad - (config->getTrackWidth() / 2) * angVel) / (config->getWheelDiameter() / 2);

        return {leftVel, rightVel};
    }

    std::pair<feet_per_second_t, radians_per_second_t> ChassisKinematics::forwardKinematics(const Eigen::Vector2<revolutions_per_minute_t> imotorVelocities)
    {
        radians_per_second_t leftVel = imotorVelocities[0];
        radians_per_second_t rightVel = imotorVelocities[1];

        feet_per_second_t linVel = (config->getWheelDiameter() * (leftVel + rightVel)) / 4_rad;
        radians_per_second_t angVel = (config->getWheelDiameter() * (rightVel - leftVel)) / (2 * config->getTrackWidth());
    
        return {linVel, angVel};
    }

} // namespace rbplib

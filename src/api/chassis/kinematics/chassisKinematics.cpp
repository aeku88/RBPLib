#include "api/chassis/kinematics/chassisKinematics.hpp"
#include "api/units.h"

namespace rbplib
{
    ChassisKinematics::ChassisKinematics(const pros::MotorGear &igearset,
                                         const inch_t iwheelDiameter,
                                         const inch_t itrackWidth)
    : gearset(igearset), wheelDiameter(iwheelDiameter), trackWidth(itrackWidth)
    {
        switch (gearset)
        {
        case pros::MotorGear::red:
            maxMotorVel = 100_rpm;
            break;
        
        case pros::MotorGear::green:
            maxMotorVel = 200_rpm;
            break;

        case pros::MotorGear::blue:
            maxMotorVel = 600_rpm;
            break;

        default:
            break;
        }
    }

    template <typename T> int sgn(T val) 
    {
        return (T(0) < val) - (val < T(0));
    }

    Eigen::Vector2<revolutions_per_minute_t> ChassisKinematics::inverseKinematics(const std::pair<feet_per_second_t, radians_per_second_t> ichassisVelocities)
    {
        feet_per_second_t linVel = ichassisVelocities.first;
        radians_per_second_t angVel = ichassisVelocities.second;

        radians_per_second_t leftVel = (linVel * 1_rad + (trackWidth / 2) * angVel) / (wheelDiameter / 2); // Multiplied linear velocity by 1 radian to match units match; in units library, angle has dimension.
        radians_per_second_t rightVel = (linVel * 1_rad - (trackWidth / 2) * angVel) / (wheelDiameter / 2);

        radians_per_second_t fasterSide = math::max(math::fabs(leftVel), math::fabs(rightVel));

        if (fasterSide > maxMotorVel) 
        {
            leftVel *= (maxMotorVel / fasterSide);
            rightVel *= (maxMotorVel / fasterSide);
        }

        return {leftVel, rightVel};
    }

    std::pair<feet_per_second_t, radians_per_second_t> ChassisKinematics::forwardKinematics(const Eigen::Vector2<revolutions_per_minute_t> imotorVelocities)
    {
        radians_per_second_t leftVel = imotorVelocities[0];
        radians_per_second_t rightVel = imotorVelocities[1];

        feet_per_second_t linVel = (wheelDiameter * (leftVel + rightVel)) / 4_rad;
        radians_per_second_t angVel = (wheelDiameter * (rightVel - leftVel)) / (2 * trackWidth);
    
        return {linVel, angVel};
    }

} // namespace rbplib

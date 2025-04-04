#include "api/chassis/model/chassisModel.hpp"
#include <valarray>

namespace rbplib
{
    ChassisModel::ChassisModel(pros::MotorGroup &ileft,
                               pros::MotorGroup &iright,
                               pros::Rotation &imiddle,
                               std::shared_ptr<ChassisKinematics> &ikinematics,
                               std::shared_ptr<ChassisConfiguration> &iconfig)
    : left(ileft), right(iright), middle(imiddle), kinematics(ikinematics), config(iconfig)
    {
        left.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
        right.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);

        left.set_zero_position_all(0);
        right.set_zero_position_all(0);

        middle.reset_position();

        switch (config->getGearset())
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

    void ChassisModel::drive(const Eigen::Vector2<revolutions_per_minute_t> &imotorVelocities)
    {
        left.move(127.0 * imotorVelocities[0] / maxMotorVel);
        right.move(127.0 * imotorVelocities[1] / maxMotorVel);
    }

    void ChassisModel::drive(const std::pair<feet_per_second_t, radians_per_second_t> &ichassisVelocities)
    {
        auto motorVelocities = kinematics->inverseKinematics(ichassisVelocities);

        drive(motorVelocities);
    }

    std::pair<feet_per_second_t, radians_per_second_t> ChassisModel::getMaxVelocity() const
    {
        return {maxMotorVel * config->getWheelDiameter() * M_PI / 60_rad,
                  config->getWheelDiameter() * maxMotorVel / config->getTrackWidth() };
    }

    std::valarray<std::int32_t> ChassisModel::getSensorVals() const
    {
        return std::valarray{
                    static_cast<std::int32_t>(left.get_position()),
                    static_cast<std::int32_t>(right.get_position()),
                    static_cast<std::int32_t>(middle.get_position())};
    }
} // namespace tekuaek

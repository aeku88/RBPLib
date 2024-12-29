#include "api/chassis/model/chassisModel.hpp"
#include <valarray>

namespace rbplib
{
    ChassisModel::ChassisModel(pros::MotorGroup &ileft,
                               pros::MotorGroup &iright,
                               pros::Rotation &imiddle)
    : left(ileft), right(iright), middle(imiddle)
    {
        left.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
        right.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
        left.set_zero_position_all(0);
        right.set_zero_position_all(0);
        middle.reset_position();
    }

    void ChassisModel::driveMotors(const revolutions_per_minute_t ileftVel, const revolutions_per_minute_t irightVel)
    {
        left.move_velocity(ileftVel.value());
        right.move_velocity(irightVel.value());
    }

    std::valarray<std::int32_t> ChassisModel::getSensorVals() const
    {
        return std::valarray<std::int32_t>{static_cast<std::int32_t>(left.get_position()),
                                           static_cast<std::int32_t>(right.get_position()),
                                           static_cast<std::int32_t>(middle.get_position())};
    }
} // namespace rbplib

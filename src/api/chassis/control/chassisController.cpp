#include "api/chassis/control/chassisController.hpp"

namespace rbplib
{
    ChassisController::ChassisController(pros::MotorGroup &ileft,
                                         pros::MotorGroup &iright,
                                         pros::Rotation &imiddle,
                                         const inch_t &iwheelDiameter,
                                         const inch_t &iwheelTrack)
    {
        model = std::make_shared<ChassisModel>(ileft, iright, imiddle);
        kinematics = std::make_shared<ChassisKinematics>(ileft.get_gearing(), iwheelDiameter, iwheelTrack);
        odometry = std::make_shared<ThreeWheelOdometry>(model);
    }


}
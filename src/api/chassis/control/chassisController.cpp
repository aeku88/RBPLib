#include "api/chassis/control/chassisController.hpp"

namespace tekuaek
{
    ChassisController::ChassisController(pros::MotorGroup &ileft,
                                         pros::MotorGroup &iright,
                                         pros::Rotation &imiddle,
                                         std::shared_ptr<ChassisConfiguration> &iconfig)
    {
        model = std::make_shared<ChassisModel>(ileft, iright, imiddle, iconfig);
        kinematics = std::make_shared<ChassisKinematics>(iconfig);
        odometry = std::make_shared<ThreeWheelOdometry>(model);
    }

    void ChassisController::arcade(const double throttle, const double steer)
    {
    }
}
#include "api/chassis/control/chassisController.hpp"

namespace rbplib
{
    ChassisController::ChassisController(pros::MotorGroup &ileft,
                                         pros::MotorGroup &iright,
                                         pros::Rotation &imiddle,
                                         std::shared_ptr<ChassisConfiguration> &iconfig)
    {
        kinematics = std::make_shared<ChassisKinematics>(iconfig);
        model = std::make_shared<ChassisModel>(ileft, iright, imiddle, kinematics, iconfig);
    }

    void ChassisController::moveToPoint(const Eigen::Vector2<inch_t> &ipoint, const bool ibackwards)
    {

    }

    void ChassisController::moveToPose(const Eigen::Vector2<inch_t> &ipoint, const degree_t &iangle, const bool ibackwards)
    {

    }

    void ChassisController::tank(const double leftThrottle, const double rightThrottle) const
    {
        auto leftVel = model->getMaxMotorVel() * leftThrottle / 127.0;
        auto rightVel = model->getMaxMotorVel() * rightThrottle / 127.0;

        model->drive(Eigen::Vector2<radians_per_second_t>{leftVel, rightVel});
    }

    void ChassisController::arcade(const double throttle, const double steer) const
    {
        auto linVel = model->getMaxVelocity().first * throttle / 127.0;
        auto angVel = model->getMaxVelocity().second * steer / 127.0;

        model->drive(make_pair(linVel, angVel));
    }
}
#pragma once

#include "api/chassis/kinematics/chassisKinematics.hpp"
#include "api/chassis/model/chassisModel.hpp"
#include "api/odometry/threeWheelOdometry.hpp"
#include "api/units.h"
#include "api.h"

using namespace units::literals;

using namespace units;
using namespace units::length;
using namespace units::time;
using namespace units::velocity;
using namespace units::angle;
using namespace units::angular_velocity;

namespace rbplib
{
    class ChassisController
    {
    public:
        ChassisController(pros::MotorGroup &ileft,
                          pros::MotorGroup &iright,
                          pros::Rotation &imiddle,
                          const inch_t &iwheelDiameter,
                          const inch_t &iwheelTrack);

        void moveToPoint(const Eigen::Vector2<inch_t> &ipoint, const bool ibackwards);

        void moveToPose(const Eigen::Vector2<inch_t> &ipoint, const degree_t &iangle, const bool ibackwards);

        std::shared_ptr<ChassisKinematics> kinematics;
        std::shared_ptr<ChassisModel> model;
        std::shared_ptr<ThreeWheelOdometry> odometry;
    private:
    };
}
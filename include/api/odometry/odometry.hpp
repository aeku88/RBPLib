#pragma once

#include "api/units.h"
#include "api.h"
#include "Eigen/Core"
#include <valarray>

#include <memory>

using namespace units::literals;
using namespace units::length;
using namespace units::angle;

namespace rbplib
{
    class Odometry
    {
    public:
        virtual Eigen::Vector3d getPose() const = 0;

        virtual void step() = 0;

        virtual Eigen::Vector3d odomMathStep(std::valarray<std::int32_t> itickDiffs) = 0;

    protected:
        Eigen::Vector3d pose = {0, 0, 0}; // x in inch, y in inch, theta in radians
    
        std::valarray<std::int32_t> newTicks = {0, 0, 0}, lastTicks = {0, 0, 0}, tickDiff = {0, 0, 0};
    };
}

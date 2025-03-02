#include "api/odometry/threeWheelOdometry.hpp"
#include <math.h>

namespace rbplib
{
    ThreeWheelOdometry::ThreeWheelOdometry(const std::shared_ptr<ChassisModel> &imodel)
    : model(std::move(imodel))
    {
        pose = {0, 0, 0};
    }

    void ThreeWheelOdometry::step()
    {
        newTicks = model->getSensorVals();
        
        tickDiff = newTicks - lastTicks;
        lastTicks = newTicks;

        const auto poseDiff = odomMathStep(tickDiff);
        pose += poseDiff;
        
        std::cout << "x: " << pose[0] << '\n';
        std::cout << "y: " << pose[1] << '\n';
        std::cout << "theta: " << pose[2] * 180/M_PI << '\n';
    }

    Eigen::Vector3d ThreeWheelOdometry::odomMathStep(std::valarray<std::int32_t> itickDiffs)
    {
        const double deltaL = (itickDiffs[0] / 360.0) * (60.0 / 36.0) * 3.25 * M_PI;
        const double deltaR = (itickDiffs[1] / 360.0) * (60.0 / 36.0) * 3.25 * M_PI;
        double deltaTheta = (deltaL - deltaR) / 8.75;
        double localOffX, localOffY;

        const auto deltaM = static_cast<const double>(
            itickDiffs[2] / 36000.0 * 2 * M_PI - 
            (deltaTheta * 2.6875
        ));

        if (deltaL == deltaR) 
        {
            localOffX = deltaM;
            localOffY = deltaR;
        } 
        
        else 
        {
            localOffX = 2 * std::sin(deltaTheta / 2) *
                        (deltaM / deltaTheta + 4.125);
            localOffY = 2 * std::sin(deltaTheta / 2) *
                        (deltaR / deltaTheta + 8.75);
        }

        double avgA = pose[2] + (deltaTheta / 2);

        double polarR = std::sqrt((localOffX * localOffX) + (localOffY * localOffY));
        double polarA = std::atan2(localOffY, localOffX) - avgA;

        double dX = std::sin(polarA) * polarR;
        double dY = std::cos(polarA) * polarR;

        if (isnan(dX))
            dX = 0;

        if (isnan(dY))
            dY = 0;

        if (isnan(deltaTheta))
            deltaTheta = 0;

        return {dX, dY, deltaTheta};
    }
}
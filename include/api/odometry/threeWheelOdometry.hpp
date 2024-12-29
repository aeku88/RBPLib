#pragma once

#include "api/odometry/odometry.hpp"
#include "api/chassis/model/chassisModel.hpp"
#include "api.h"

namespace rbplib
{
    class ThreeWheelOdometry : public Odometry
    {
    public:
        ThreeWheelOdometry(const std::shared_ptr<ChassisModel> &imodel);

        Eigen::Vector3d getPose() const override;

        void step() override;

        Eigen::Vector3d odomMathStep(std::valarray<std::int32_t> itickDiffs) override;
    private:
        std::shared_ptr<ChassisModel> model;
    };
}
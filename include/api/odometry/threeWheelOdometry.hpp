#pragma once

#include "api/odometry/odometry.hpp"
#include "api/chassis/model/chassisModel.hpp"
#include "api.h"

namespace tekuaek
{
    class ThreeWheelOdometry : public Odometry
    {
    public:
        ThreeWheelOdometry(const std::shared_ptr<ChassisModel> &imodel);

        inline Eigen::Vector3d getPose() const override { return pose; }

        inline Eigen::Vector3d setPose(const Eigen::Vector3d &ipose) override { pose = ipose; }

        void step() override;

        Eigen::Vector3d odomMathStep(std::valarray<std::int32_t> itickDiffs) override;
        
    private:
        std::shared_ptr<ChassisModel> model;
    };
}
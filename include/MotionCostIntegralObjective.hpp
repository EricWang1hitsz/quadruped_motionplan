#pragma once

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/GetGridMap.h"
#include <ros/ros.h>

namespace ompl
{
    namespace base
    {

        class motionCostIntegralObjective : public OptimizationObjective
        {
        public:

            motionCostIntegralObjective(const SpaceInformationPtr &si, bool enableMotionCostInterpolation = false);

            Cost stateCost(const State *s) const override;

            Cost motionCost(const State *s1, const State *s2) const override;

            bool isMotionCostInterpolationEnabled() const;

            void traversabilityMapCallback(const grid_map_msgs::GridMapPtr &traversability_map);

        protected:

            bool interpolateMotionCost_;

            Cost trapezoid(Cost c1, Cost c2, double dist) const
            {
                return Cost(0.5 * dist * (c1.value() + c2.value()));
            }

        private:
            ros::NodeHandle nh_;

            ros::Subscriber traversability_map_sub_;

            grid_map::GridMap traversability_map_;

        };


        OptimizationObjectivePtr getMotionCostIntegralObjective(const SpaceInformationPtr& si, bool enableMotionCostInterpolation)
        {
            return std::make_shared<motionCostIntegralObjective>(si, enableMotionCostInterpolation);
        }

}
}





#pragma once

#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/config.h"
#include "ompl/base/StateSampler.h"

#include "grid_map_msgs/GridMap.h"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

#include <ros/ros.h>
#include <thread>
#include <memory>


namespace ob = ompl::base;
namespace og = ompl::geometric;

class MyValidStateSampler : public ob::ValidStateSampler
{
public:
    MyValidStateSampler(const ob::SpaceInformation *si) : ValidStateSampler(si)
    {
        name_ = "my sampler";
    }

    /**
     * @brief sample Sample the state
     * @param state
     * @return True
     */
    bool sample(ob::State *state) override;
    /**
     * @brief sampleNear
     * @return
     */
    bool sampleNear(ob::State*, const ob::State*, const double) override;


protected:
    ompl::RNG rng_;

private:
    ob::StateSamplerPtr sampler_;

};

ob::ValidStateSamplerPtr allocMyValidStateSampler(const ob::SpaceInformation *si)
{
    return std::make_shared<MyValidStateSampler>(si);
}


class validStateCheck
{
public:

    validStateCheck();

    void traverabilityMapCallback(const grid_map_msgs::GridMapPtr& traversability_map);

    static bool isStateValid(const ob::State *state);

private:

    grid_map::GridMap traversability_map_;

    ros::Subscriber traversability_map_sub_;

    ros::NodeHandle nodehandle_;

    typedef std::shared_ptr<validStateCheck> validStateCheckPtr;


};



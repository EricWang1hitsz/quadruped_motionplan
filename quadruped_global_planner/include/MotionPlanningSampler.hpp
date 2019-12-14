#pragma once

#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/config.h"
#include "ompl/base/State.h"
#include "ompl/base/StateSampler.h"

#include "grid_map_msgs/GridMap.h"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <thread>
#include <memory>


namespace ob = ompl::base;
namespace og = ompl::geometric;

// TODO where is class MyValidStateSampler is constructed????

class MyValidStateSampler : public ob::ValidStateSampler
{
public:

    MyValidStateSampler(const ob::SpaceInformation *si) : ValidStateSampler(si)
    {
        ROS_INFO("MyValidStateSampler is constructing...");

        name_ = "my sampler";

        state_marker_pub_ = nh.advertise<visualization_msgs::Marker>("State_Marker", 1000);

        ROS_INFO("MyValidStateSample construted");
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

    /**
     * @brief stateMarkerPub Publish Sampled Marker
     * @param state
     */
    void stateMarkerPub(ob::State *state);

protected:

    ompl::RNG rng_;

private:

    ob::StateSamplerPtr sampler_;

    ob::ValidStateSamplerPtr valid_state_sampler_;

    ros::NodeHandle nh;

    ros::Publisher state_marker_pub_;

};

ob::ValidStateSamplerPtr allocMyValidStateSampler(const ob::SpaceInformation *si)
{
    return std::make_shared<MyValidStateSampler>(si);
}

//class validStateCheck
//{
//public:

//    validStateCheck();

//    void traverabilityMapCallback(const grid_map_msgs::GridMapPtr& traversability_map);

////    static bool isStateValid(const ob::State *state);

//    static grid_map::GridMap traversability_map_;

//private:

////    static grid_map::GridMap traversability_map_;

//    ros::Subscriber traversability_map_sub_;

//    ros::NodeHandle nodehandle_;

//    typedef std::shared_ptr<validStateCheck> validStateCheckPtr;


//};



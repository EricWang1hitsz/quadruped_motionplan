#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/SimpleSetup.h"
#include "MotionPlanningSampler.hpp"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class motion_planning
{
public:

    motion_planning();

    void setStart(double x, double y, double yaw);

    void setGoal(double x, double y, double yaw);

    void plan();

    void traj_pub(og::PathGeometric* pth);

    void elevationMapCallback(const grid_map_msgs::GridMapPtr& elevation_map);

    float getElevationInformation(const grid_map::Position& postion);

    /**
     * @brief set_start Flag for starting planning;
     */
    bool set_start = false;

private:

    ob::StateSpacePtr space;

    ob::ProblemDefinitionPtr pdef_;

//    ob::SpaceInformationPtr si_;

    grid_map::GridMap elevation_map_;

    ros::Publisher traj_pub_;

    ros::Subscriber elevation_map_sub_;

    ros::NodeHandle nh_;


};


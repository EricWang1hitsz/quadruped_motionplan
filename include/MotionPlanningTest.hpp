#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/SimpleSetup.h"
#include "StateValidityChecker.hpp"
#include "MotionValidityChecker.hpp" // check motion is valid or not.
#include "MotionPlanningSampler.hpp"
#include "MotionPlannerRRTstar.hpp"
#include "MotionPlannerInfRRTstar.hpp"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class motion_planning
{
public:

    motion_planning(void);

    void init_start(void);

    void setStart(double x, double y, double yaw);

    void setGoal(double x, double y, double yaw);

    void plan();

    void basePoseCb(const geometry_msgs::PoseWithCovarianceStampedPtr& base_pose);

    void goalPoseCb(const geometry_msgs::PoseStampedPtr& goal_pose);

    void traj_pub(og::PathGeometric* pth);

    void traj3d_pub(og::PathGeometric* pth);

    void elevationMapCallback(const grid_map_msgs::GridMapPtr& elevation_map);

    float getElevationInformation(const grid_map::Position& postion);

    /**
     * @brief set_start Flag for starting planning;
     */
    bool set_start = false;

private:

    ob::StateSpacePtr space;

    ob::ProblemDefinitionPtr pdef_;

    ob::SpaceInformationPtr si_;

    grid_map::GridMap elevation_map_;

    ros::Publisher traj_pub_;

    ros::Publisher traj3d_pub_;

    ros::Subscriber elevation_map_sub_;

    ros::Subscriber base_pose_sub_;

    ros::Subscriber goal_pose_sub_;

    ros::NodeHandle nh_;


};


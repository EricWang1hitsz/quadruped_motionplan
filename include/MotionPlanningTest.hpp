#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "MotionCostIntegralObjective.hpp"
#include "StateValidityChecker.hpp"
#include "MotionValidityChecker.hpp" // check motion is valid or not.
#include "MotionPlanningSampler.hpp"
#include "MotionPlannerRRTstar.hpp"
#include "MotionPlannerInfRRTstar.hpp"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
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

    void replan();

    void basePoseCb(const geometry_msgs::PoseWithCovarianceStampedPtr& base_pose);

    void goalPoseCb(const geometry_msgs::PoseStampedPtr& goal_pose);

    void traj_pub(og::PathGeometric* pth);

    void traj3d_pub(og::PathGeometric* pth);

    void waypoint_tracked(og::PathGeometric* pth);

    void elevationMapCallback(const grid_map_msgs::GridMapPtr& elevation_map);

    void localElevationMapCallback(const grid_map_msgs::GridMapPtr& local_elevation_map);

    float getElevationInformation(const grid_map::Position& postion);
    /*!
     * \brief setReplanInitialInfo Set start and goal info when replan.
     */
    void setReplanInitialInfo();
    /*!
     * \brief setSegmentIsUntraversable Set segments from pathValidityChecker.
     * \param segmentsIsUntraversable
     */
    void setSegmentIsUntraversable(geometry_msgs::PoseArray segmentsIsUntraversable);
    /**
     * @brief checkNextSegment Use the most up-to-date elevation map to verify the next segment of the
     * global path starting from the current robot position
     * @return
     */
    bool checkNextSegment();

    /**
     * @brief set_start Flag for starting planning;
     */
    bool set_start;

private:

    ob::StateSpacePtr space;

    ob::ProblemDefinitionPtr pdef_;

    ob::SpaceInformationPtr si_;
    /**
     * @brief robot_location_ real-time robot location
     */
    //ob::ScopedState<ob::SE2StateSpace> robot_location_;
    /**
     * @brief robot_des_ real-time robot destination when replanning
     */
    //ob::ScopedState<ob::SE2StateSpace> robot_des_;

    grid_map::GridMap elevation_map_;

    grid_map::GridMap local_elevation_map_;

    ros::Publisher traj_pub_;

    ros::Publisher traj3d_pub_;
    /**
     * @brief waypoint_pub_ Publish interpolation point
     */
    ros::Publisher waypoint_pub_;
    /**
     * @brief elevation_map_sub_ Subcribe non-realtime global elevation map
     */
    ros::Subscriber elevation_map_sub_;
    /**
     * @brief real_elevation_map_sub_ Subscribe real-time local elevation map
     */
    ros::Subscriber local_elevation_map_sub_;

    ros::Subscriber base_pose_sub_;

    ros::Subscriber goal_pose_sub_;

    ros::NodeHandle nh_;

    double last_yaw;
    /*!
     * \brief footPrintPath Path uesd for checking path valid.
     */
    geometry_msgs::PoseArray footPrintPath;
    /*!
     * \brief segmentIsUntraversable Untraversable segments return from pathValidityChecker.
     */
    geometry_msgs::PoseArray segmentIsUntraversable_;
    /*!
     * \brief footPrintPathMsg_ Saved palnned path for checking and replanning.
     */
    traversability_msgs::FootprintPath footPrintPathMsg_;

    tf::TransformBroadcaster br;

    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
    {
        ob::OptimizationObjectivePtr obj (new ob::PathLengthOptimizationObjective(si));
        obj->setCostThreshold(ob::Cost(0.60)); // Specify an optimality threshold.
        return obj;
    }

};


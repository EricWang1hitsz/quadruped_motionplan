#pragma once

#include <vector>
#include <Eigen/Core>
#include <string.h>

#include "trajectory.h"
#include "local_planner_util.h"
#include "local_planner_limits.h"
#include "simple_scored_sampling_planner.h"
#include "simple_trajectory_generator.h"

namespace dwa_planner
{
class DWAplanner
{
public:

    DWAplanner();

    ~DWAplanner();

    /**
     * @brief checkTrajectory Check if a trajectory is legal for a position/velocity pair.
     * @param pos
     * @param vel
     * @param vel_samples
     * @return
     */
    bool checkTrajectory(const Eigen::Vector3f pos, const Eigen::Vector3f vel, const Eigen::Vector3f vel_samples);

    /**
     * @brief findBestPath Given the current postion and velocity of the robot, find the best trajectory to execute.
     * @param global_pose
     * @param global_vel
     * @param drive_velocities
     * @return
     */
    quadruped_local_planner::Trajectory findBestPath(
            tf::Stamped<tf::Pose> global_pose,
            tf::Stamped<tf::Pose> global_vel,
            tf::Stamped<tf::Pose>& drive_velocities);
    /**
     * @brief updatePlanAndLocalCost Update the cost functions before planning.
     * @param global_pose
     * @param new_plan
     * @param footprint_spec
     */
    void updatePlanAndLocalCost(tf::Stamped<tf::Pose> global_pose,
                                const std::vector<geometry_msgs::PoseStamped>& new_plan,
                                const std::vector<geometry_msgs::Point>& footprint_spec);
    /**
     * @brief getSimPeriod Get the period at which the local planner is expected to run.
     * @return
     */
    double getSimPeriod() {return sim_period_;};

    /**
     * @brief setPlan Set a new plan and resets state.
     * @param origin_global_plan
     * @return
     */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& origin_global_plan);


private:
    /**
     * @brief sim_period_ The number of seconds to use to compute max/min vels for dwa.
     */
    double sim_period_;
    /**
     * @brief result_traj_ Get the best trajectory.
     */
    quadruped_local_planner::Trajectory result_traj_;
};
}


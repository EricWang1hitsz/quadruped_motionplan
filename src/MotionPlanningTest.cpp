#include "MotionPlanner.hpp"
#include "MotionPlanningSampler.hpp"
#include "MotionPlanningTest.hpp"
#include "ompl/geometric/planners/prm/PRM.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

motion_planning::motion_planning()
{
    ROS_INFO("Class motion_planning constrcting...");
    traj_pub_ = nh_.advertise<nav_msgs::Path>("waypoints", 10);
    elevation_map_sub_ = nh_.subscribe("/elevation_mapping/elevation_map", 1, &motion_planning::elevationMapCallback, this);
}

void motion_planning::setStart(double x, double y, double yaw)
{
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(x);
    start->setY(y);
    start->setYaw(yaw);
    pdef_->clearStartStates();
    pdef_->addStartState(start);
    ROS_INFO("Add Start State");
}

void motion_planning::setGoal(double x, double y, double yaw)
{
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(x);
    goal->setY(y);
    goal->setYaw(yaw);
    pdef_->clearGoal();
    pdef_->setGoalState(goal);
    ROS_INFO("Add Goal State");
}

void motion_planning::plan()
{
    auto space(std::make_shared<ob::SE2StateSpace>());

        // set the bounds, according to the space dimension;
        ob::RealVectorBounds bounds(2);
        bounds.setLow(-1);
        bounds.setHigh(1);
        space->setBounds(bounds);

        // define a simple setup class
        og::SimpleSetup ss(space);

        // set state validity checking for this space
        ss.setStateValidityChecker(validStateCheck::isStateValid);

        // create a start state
        ob::ScopedState<ob::SE2StateSpace> start(space);
        start[0] = start[1] = start[2] = 0;

        // create a goal state
        ob::ScopedState<ob::SE2StateSpace> goal(space);
        goal[0] = goal[1] = 1;
        goal[2] = 1;

        // set the start and goal states
        ss.setStartAndGoalStates(start, goal);

        // create a planner for the defined space
        auto planner(std::make_shared<og::RRT_IM>(ss.getSpaceInformation()));
        ss.setPlanner(planner);
        planner->setRange(0.1);

        // attempt to solve the problem within ten seconds of planning time
        ob::PlannerStatus solved = ss.solve(30.0);
        if (solved)
        {
            std::cout << "Found solution:" << std::endl;
            // print the path to screen
            ss.getSolutionPath().print(std::cout);
        }
        else
            std::cout << "No solution found" << std::endl;

        pdef_ = ss.getProblemDefinition();

        ob::PathPtr path = pdef_->getSolutionPath();

        og::PathGeometric* pth_ = pdef_->getSolutionPath()->as<og::PathGeometric>();
        ROS_INFO("TEST1");
        traj_pub(pth_);

}

void motion_planning::elevationMapCallback(const grid_map_msgs::GridMapPtr &elevation_map)
{
    grid_map::GridMapRosConverter::fromMessage(*elevation_map, elevation_map_);
    set_start = true;
    ROS_INFO("Receive elevation map successfully");
}


float motion_planning::getElevationInformation(const grid_map::Position& position)
{
    //Get cell data at requested position.
    double elevation = elevation_map_.atPosition("elevation",position);
    return elevation;
}


void motion_planning::traj_pub(og::PathGeometric* pth)
{
    ROS_INFO("Receive path successfully, start publishing");
    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";

    geometry_msgs::PoseStamped pose;

    for(std::size_t path_idx = 0; path_idx < pth->getStateCount(); path_idx++)
    {
        const ob::SE2StateSpace::StateType *se2state = pth->getState(path_idx)->as<ob::SE2StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
        pose.pose.position.x = pos->values[0];
        pose.pose.position.y = pos->values[1];
        //TODO Add height from elevation map;
        grid_map::Position postion_;
        postion_.x() = pos->values[0];
        postion_.y() = pos->values[1];
        std::cout<<postion_.x()<<std::endl;
        float z;
        z = getElevationInformation(postion_);
        pose.pose.position.z = z + 0.5;

        double yaw = pos->values[2];
        // Creat quaternion from yaw;
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);
        pose.pose.orientation.x = quat.x;
        pose.pose.orientation.y = quat.y;
        pose.pose.orientation.z = quat.z;
        pose.pose.orientation.w = quat.w;

        msg.poses.push_back(pose);
        ROS_INFO("Publish planned trajctory");
        traj_pub_.publish(msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MotionPlanningTest");
    validStateCheck vsc;
    motion_planning mopl;
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        if(mopl.set_start)
        {
            mopl.plan();
            return 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
//    mopl.plan();
    ROS_INFO("PLANNING FINISHED");

}

#include "MotionPlanner.hpp"
#include "MotionPlanningSampler.hpp"
#include <MotionPlanningTest.hpp>
#include <ros/ros.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

motion_planning::motion_planning(void)
{
    ROS_INFO("Class motion_planning constrcting...");
    traj_pub_ = nh_.advertise<nav_msgs::Path>("waypoints", 1000);
    traj3d_pub_ = nh_.advertise<nav_msgs::Path>("waypoints_3d", 1000);
    waypoint_pub_ = nh_.advertise<geometry_msgs::PoseArray>("waypoint_tracked", 100);
    elevation_map_sub_ = nh_.subscribe("/traversability_estimation/traversability_map", 1, &motion_planning::elevationMapCallback, this);
    //elevation_map_sub_ = nh_.subscribe("/grid_map_filter_demo/filtered_map", 1, &motion_planning::elevationMapCallback, this);
    //local_elevation_map_sub_ = nh_.subscribe("/grid_map_filter_demo/filtered_map", 1, &motion_planning::localElevationMapCallback, this);
    base_pose_sub_ = nh_.subscribe("/pose_pub_node/base_pose", 1, &motion_planning::basePoseCb, this);
    goal_pose_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &motion_planning::goalPoseCb, this);

//    auto space(std::make_shared<ob::SE2StateSpace>());// what's difference??
    space = ob::StateSpacePtr(new ob::SE2StateSpace());
    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    // create a goal state
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    // set the bounds, according to the space dimension;
    ob::RealVectorBounds bounds(2);
    // Set x y direction bounds.
    bounds.low[0] = -2.5;
    bounds.low[1] = -2.5;
    bounds.high[0] = 2.5;
    bounds.high[1] = 2.5;
    bounds.low[2] = -2.5;
    bounds.high[2] = 2.5;
//    bounds.setLow(-2.5);
//    bounds.setHigh(2.5);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    si_ = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

    start->setX(0);
    start->setY(0);
    start->setYaw(0);

    goal->setX(1);
    goal->setY(1);
    goal->setYaw(1);

    // set state validity checking for this space
    si_->setStateValidityChecker(std::make_shared<ob::myStateValidityChecker>(si_));

    // Set the instance of the motion validity checker to use.
    si_->setMotionValidator(std::make_shared<ob::myMotionValidator>(si_));

    // creat a problem instance
    pdef_ = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si_));

    // set the start and goal states
    pdef_->setStartAndGoalStates(start, goal);

    pdef_->setOptimizationObjective(getPathLengthObjective(si_));
    pdef_->setOptimizationObjective(getMotionCostIntegralObjective(si_, false)); // Defined optimal obejective.

//    ss.getSpaceInformation()->setValidStateSamplerAllocator(allocMyValidStateSampler);

    si_->setValidStateSamplerAllocator(allocMyValidStateSampler);

    last_yaw = 0.0;

    set_start = false;

    ROS_INFO("MotionPlanning Construction:: Initilized");
}


void motion_planning::init_start(void)
{
    if(!set_start)
        std::cout << "Initialized" << std::endl;
    set_start = true;
}

// start state must satisfy the bound and state valid checker.
void motion_planning::setStart(double x, double y, double yaw)
{
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(x + 0.5);
    start->setY(y);
    start->setYaw(yaw);
    pdef_->clearStartStates();
    pdef_->addStartState(start);
    //std::cout << "start pose is:" << x << y << yaw << std::endl;
    ROS_INFO_ONCE("Please add goal pose in Rviz");
    init_start();
}

void motion_planning::setGoal(double x, double y, double yaw)
{
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(x);
    goal->setY(y);
    goal->setYaw(yaw);
    pdef_->clearGoal();
    pdef_->setGoalState(goal);
    std::cout << "goal pose is:" << x << y << yaw << std::endl;
    ROS_INFO("Add Goal State");
    if(set_start)
        plan();
}

void motion_planning::basePoseCb(const geometry_msgs::PoseWithCovarianceStampedPtr &base_pose)
{
    double x, y, z;
    x = base_pose->pose.pose.position.x;
    y = base_pose->pose.pose.position.y;
    z = base_pose->pose.pose.position.z;

    tf::Quaternion quaternion_;
    tf::quaternionMsgToTF(base_pose->pose.pose.orientation, quaternion_);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion_).getRPY(roll, pitch, yaw);
    //robot_location_->setX(x);
    //robot_location_->setY(y);
    //robot_location_->setYaw(yaw);
    setStart(x, y, yaw);
}

void motion_planning::goalPoseCb(const geometry_msgs::PoseStampedPtr &goal_pose)
{
    double x, y, z;
    x = goal_pose->pose.position.x;
    y = goal_pose->pose.position.y;
    z = goal_pose->pose.position.z;

    tf::Quaternion quaternion_;
    tf::quaternionMsgToTF(goal_pose->pose.orientation, quaternion_);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion_).getRPY(roll, pitch, yaw);
    setGoal(x, y, yaw);
}

void motion_planning::plan()
{
    // create a planner for the defined space

//    ob::PlannerPtr plan(new og::RRT_IM(si_));

    ROS_INFO("Start planning");

//    og::RRT_IM* rrt = new og::RRT_IM(si_); // RRT.

//    og::RRTstar *rrt = new og::RRTstar(si_); // RRTstar.

    og::InformedRRTstar_IM* rrt = new og::InformedRRTstar_IM(si_); // InformedRRTstar.

    //represents the maximum length of a motion to be added in the tree of motion.
    rrt->setRange(0.3);
    rrt->setGoalBias(0.3);

    ob::PlannerPtr plan(rrt);

    plan->setProblemDefinition(pdef_);

    plan->setup();

    // attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = plan->solve(5.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen

        ob::PathPtr path = pdef_->getSolutionPath();

        path->print(std::cout);

        og::PathGeometric* pth_ = pdef_->getSolutionPath()->as<og::PathGeometric>();
        ROS_WARN_STREAM("Path state count :" << pth_->getStateCount() << std::endl);
        ROS_WARN_STREAM("Path length size :" << pth_->length() << std::endl);
        //pth_->interpolate(10);
        ROS_WARN_STREAM("Path state count after interpolatation: " << pth_->getStateCount() << std::endl);
        traj_pub(pth_);
        waypoint_tracked(pth_);
        og::PathSimplifier* pathBSpline = new og::PathSimplifier(si_);

        pathBSpline->smoothBSpline(*pth_, 5);// State count change more after smooth.
        //pathBSpline->reduceVertices(*pth_, 10, 10, 0.33);
        traj3d_pub(pth_); // Bspline trajectory.
        //waypoint_tracked(pth_);

    }
    else
        std::cout << "No solution found" << std::endl;


}

void motion_planning::setReplanInitialInfo()
{
    geometry_msgs::PoseArray segments = segmentIsUntraversable_;
    const auto arraySize = segments.poses.size();
    if(arraySize < 2)
        ROS_WARN("Has no start and goal pose when replan");
    geometry_msgs::Pose start_ = segmentIsUntraversable_.poses[0];
    geometry_msgs::Pose goal_ = segments.poses[arraySize - 1];
    ob::ScopedState<ob::SE2StateSpace> restart(space);
    ob::ScopedState<ob::SE2StateSpace> regoal(space);
    restart->setX(start_.position.x);
    restart->setY(start_.position.y);
    tf::Quaternion quaternion_;
    tf::quaternionMsgToTF(start_.orientation, quaternion_);
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion_).getRPY(roll, pitch, yaw);
    restart->setYaw(yaw);
    ROS_INFO_STREAM("restart info " << start_.position.x << " " << start_.position.y << " " << yaw << std::endl);
    //goal info
    regoal->setX(goal_.position.x);
    regoal->setY(goal_.position.y);
    tf::Quaternion quaternion2_;
    tf::quaternionMsgToTF(goal_.orientation, quaternion2_);
    double roll2, pitch2, yaw2;
    tf::Matrix3x3(quaternion2_).getRPY(roll2, pitch2, yaw2);
    regoal->setYaw(yaw2);
    ROS_INFO_STREAM("regoal info " << goal_.position.x <<" " << goal_.position.y << " " << yaw2 << std::endl);
    //set start and goal.
    pdef_->clearStartStates();
    pdef_->addStartState(restart);
    pdef_->clearGoal();
    pdef_->setGoalState(regoal);
    ROS_INFO("Replan start and goal info set!");

}

void motion_planning::setSegmentIsUntraversable(geometry_msgs::PoseArray segmentsIsUntraversable)
{
    segmentIsUntraversable_ = segmentsIsUntraversable;
}

// TODO!
void motion_planning::replan()
{
    pdef_->clearStartStates();
    pdef_->clearGoal();
    //Set start state where robot is now.
    //pdef_->addStartState(robot_location_);
    //Next valid segment starting from the current robot position
    //pdef_->setGoalState(robot_des_);
}

void motion_planning::elevationMapCallback(const grid_map_msgs::GridMapPtr &elevation_map)
{
    grid_map::GridMapRosConverter::fromMessage(*elevation_map, elevation_map_);
    set_start = true;
    ROS_INFO_ONCE("Motion planning test::Receive elevation map successfully");
}

void motion_planning::localElevationMapCallback(const grid_map_msgs::GridMapPtr &local_elevation_map)
{
    grid_map::GridMapRosConverter::fromMessage(*local_elevation_map, local_elevation_map_);
    ROS_WARN_ONCE("Receive local real-time elevation map successfully");
}


float motion_planning::getElevationInformation(const grid_map::Position& position)
{
    //Get cell data at requested position.
    double elevation = elevation_map_.atPosition("elevation",position);
    return elevation;
}


void motion_planning::traj_pub(og::PathGeometric* pth)
{
    ROS_INFO_ONCE("Receive path successfully, start publishing");
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
        pose.pose.position.z = 0.2;

        double yaw = pos->values[2];
        // Creat quaternion from yaw;
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);
        pose.pose.orientation.x = quat.x;
        pose.pose.orientation.y = quat.y;
        pose.pose.orientation.z = quat.z;
        pose.pose.orientation.w = quat.w;

        msg.poses.push_back(pose);
        //ROS_INFO_ONCE("Publish planned trajctory");
        traj_pub_.publish(msg);
    }
}

void motion_planning::traj3d_pub(og::PathGeometric *pth)
{
    ROS_INFO_ONCE("Receive path successfully, start publishing");
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
        //ROS_INFO("Publish 3d planned trajctory");
        traj3d_pub_.publish(msg);
    }
}

void motion_planning::waypoint_tracked(og::PathGeometric* pth)
{
    geometry_msgs::PoseArray msgs_;
    geometry_msgs::Pose msg;
    msgs_.header.stamp = ros::Time::now();
    msgs_.header.frame_id = "odom";

    for(std::size_t path_idx = 0; path_idx < pth->getStateCount() - 1; path_idx++)
    {
        ROS_WARN_STREAM_ONCE("Publish waypoints : " << pth->getStateCount() << std::endl);
        const ob::SE2StateSpace::StateType *se2state = pth->getState(path_idx)->as<ob::SE2StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
        const ob::SE2StateSpace::StateType *se2stateNext = pth->getState(path_idx + 1)->as<ob::SE2StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *posNext = se2stateNext->as<ob::RealVectorStateSpace::StateType>(0);
        double x1 = pos->values[0];
        double y1 = pos->values[1];
        double x2 = posNext->values[0];
        double y2 = posNext->values[1];
        double yaw = std::atan2((y2 - y1), (x2 - x1));
        ROS_WARN_STREAM("Yaw value: " << yaw << std::endl);
        double current_yaw = yaw;
        msg.position.x = x1;
        msg.position.y = y1;
        //TODO Add height from elevation map;
        grid_map::Position postion_;
        postion_.x() = pos->values[0];
        postion_.y() = pos->values[1];
        float z;
        z = getElevationInformation(postion_);
        z = z + 0.5;
        msg.position.z = z;
        //double yaw = pos->values[2];
        // Creat quaternion from yaw;
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);
        msg.orientation.x = quat.x;
        msg.orientation.y = quat.y;
        msg.orientation.z = quat.z;
        msg.orientation.w = quat.w;
        //msgs_.poses.push_back(msg);
        //Set a interpolation waypoint if change over 20 degree.
        if(std::abs(current_yaw - last_yaw) > 0.35) // 20/180*3.14
            msgs_.poses.push_back(msg);
            last_yaw = current_yaw;
        // Check path validity.
        footPrintPath = msgs_;
    }

    waypoint_pub_.publish(msgs_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MotionPlanningTest");
    motion_planning mopl;
//    mopl.setStart(-2, -2, 1.35); // default start pose.
//    mopl.setGoal(1, 1, 1);
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("PLANNING FINISHED");

}

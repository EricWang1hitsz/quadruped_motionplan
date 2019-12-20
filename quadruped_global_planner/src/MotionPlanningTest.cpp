#include "MotionPlanner.hpp"
#include "MotionPlanningSampler.hpp"
#include "MotionPlanningTest.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

motion_planning::motion_planning(void)
{
    ROS_INFO("Class motion_planning constrcting...");
    traj_pub_ = nh_.advertise<nav_msgs::Path>("waypoints", 1000);
    traj3d_pub_ = nh_.advertise<nav_msgs::Path>("waypoints_3d", 1000);
    trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("trajectory", 1000);
    elevation_map_sub_ = nh_.subscribe("/traversability_estimation/traversability_map", 1, &motion_planning::elevationMapCallback, this);
//    elevation_map_sub_ = nh_.subscribe("/traversability_estimation/traversability_map", &motion_planning::elevationMapCallback, this);
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
    bounds.setLow(-2.5);
    bounds.setHigh(2.5);
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

//    pdef_->setOptimizationObjective(getPathLengthObjective(si_));
//    pdef_->setOptimizationObjective(getMotionCostIntegralObjective(si_, false)); // Defined optimal obejective.

//    ss.getSpaceInformation()->setValidStateSamplerAllocator(allocMyValidStateSampler);

    si_->setValidStateSamplerAllocator(allocMyValidStateSampler);

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
    start->setX(x);
    start->setY(y);
    start->setYaw(yaw);
    pdef_->clearStartStates();
    pdef_->addStartState(start);
    std::cout << "start pose is:" << x << y << yaw << std::endl;
    ROS_INFO("Please add goal pose in Rviz");
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

    rrt->setRange(0.02);
    rrt->setGoalBias(0.5);

    ob::PlannerPtr plan(rrt);

    plan->setProblemDefinition(pdef_);

    plan->setup();

    // attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = plan->solve(20.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen

        ob::PathPtr path = pdef_->getSolutionPath();

        path->print(std::cout);

        og::PathGeometric* pth_ = pdef_->getSolutionPath()->as<og::PathGeometric>();
        traj_pub(pth_);
        og::PathSimplifier* pathBSpline = new og::PathSimplifier(si_);
        pathBSpline->smoothBSpline(*pth_, 5);
//        traj_pub(pth_);
        traj3d_pub(pth_); // Bspline trajectory.
        trajectory_pub(pth_);
    }
    else
        std::cout << "No solution found" << std::endl;


}

void motion_planning::elevationMapCallback(const grid_map_msgs::GridMapPtr &elevation_map)
{
    grid_map::GridMapRosConverter::fromMessage(*elevation_map, elevation_map_);
    set_start = true;
    ROS_INFO("Motion planning test::Receive elevation map successfully");
}


float motion_planning::getElevationInformation(const grid_map::Position& position)
{
    //Get cell data at requested position.
    double elevation = elevation_map_.atPosition("elevation",position);
    return elevation;
}

void motion_planning::trajectory_pub(og::PathGeometric *pth)
{
    ROS_INFO("Receive path successfully, start publishing trajectory");
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;

    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.header.frame_id = "odom";
    trajectory_msg.joint_names.clear();
    trajectory_msg.points.clear();
    trajectory_msg.joint_names.push_back("base_link");

    for(std::size_t path_idx = 0; path_idx < pth->getStateCount(); path_idx++)
    {
        const ob::SE2StateSpace::StateType *se2state = pth->getState(path_idx)->as<ob::SE2StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
        const ob::SO2StateSpace::StateType *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

        trajectory_point_msg.time_from_start.fromNSec(ros::Time::now().toSec());
        trajectory_point_msg.transforms.resize(1);

        trajectory_point_msg.transforms[0].translation.x = pos->values[0];
        trajectory_point_msg.transforms[0].translation.y = pos->values[1];
        trajectory_point_msg.transforms[0].translation.z = 0.5;

        double yaw = rot->value;
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);
        trajectory_point_msg.transforms[0].rotation.x = quat.x;
        trajectory_point_msg.transforms[0].rotation.y = quat.y;
        trajectory_point_msg.transforms[0].rotation.z = quat.z;
        trajectory_point_msg.transforms[0].rotation.w = quat.w;
//        trajectory_point_msg.velocities.


        trajectory_msg.points.push_back(trajectory_point_msg);

    }

    trajectory_pub_.publish(trajectory_msg);
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
        pose.pose.position.z = 0.2;

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

void motion_planning::traj3d_pub(og::PathGeometric *pth)
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
        ROS_INFO("Publish 3d planned trajctory");
        traj3d_pub_.publish(msg);
    }
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

#include "MotionPlanningSampler.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;


bool MyValidStateSampler::sample(ob::State *state)
{
    // TODO Add heuristic sampling.


//    double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
    double x = rng_.uniformReal(0, 20);
    double y = rng_.uniformReal(-2,2);
    double yaw = rng_.uniformReal(-1,1);

    static_cast<ob::SE2StateSpace::StateType*>(state)->setX(x);
    static_cast<ob::SE2StateSpace::StateType*>(state)->setY(y);
    static_cast<ob::SE2StateSpace::StateType*>(state)->setYaw(yaw);

    assert(si_->isValid(state));
    // Publish sampled state marker
    stateMarkerPub(state);
    return true;
}

bool MyValidStateSampler::sampleNear(ob::State *, const ob::State *, const double)
{
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
    return false;
}

void MyValidStateSampler::stateMarkerPub(ob::State *state)
{
    const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    double x = pos->values[0];
    double y = pos->values[1];
    double yaw = pos->values[2];

    geometry_msgs::Pose base_pose;
    base_pose.position.x = x;
    base_pose.position.y = y;
    base_pose.position.z = 0.5;
    geometry_msgs::Quaternion quaternion_ = tf::createQuaternionMsgFromYaw(yaw);
    base_pose.orientation.x = quaternion_.x;
    base_pose.orientation.y = quaternion_.y;
    base_pose.orientation.z = quaternion_.z;
    base_pose.orientation.w = quaternion_.w;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = base_pose.position.x;
    marker.pose.position.y = base_pose.position.y;
    marker.pose.position.z = base_pose.position.z;
    marker.pose.orientation.x = base_pose.orientation.x;
    marker.pose.orientation.y = base_pose.orientation.y;
    marker.pose.orientation.z = base_pose.orientation.z;
    marker.pose.orientation.w = base_pose.orientation.w;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
//    marker.mesh_resource = "/home/hit/Documents/kinect.dae";
    state_marker_pub_.publish(marker);
    ROS_INFO("Publish sampled state maker once");
}

validStateCheck::validStateCheck()
{
    ROS_INFO("Class validStateCheck is constructing...");
    traversability_map_sub_ = nodehandle_.subscribe("grid_map_filter_demo/filtered_map", 1, &validStateCheck::traverabilityMapCallback, this);
}

void validStateCheck::traverabilityMapCallback(const grid_map_msgs::GridMapPtr &traversability_map)// Pointer
{
    grid_map::GridMapRosConverter::fromMessage(*traversability_map, traversability_map_);// Dereference
    ROS_INFO("Receive traversability map successfully");
}

bool validStateCheck::isStateValid(const ob::State *state)
{
    OMPL_INFORM("Test State is Valid");

    const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    std::this_thread::sleep_for(ompl::time::seconds(1));

    double x = pos->values[0];
    double y = pos->values[1];
    double yaw = pos->values[2];

    ROS_INFO("isStateValid");
    double radius = 1.0;
    Eigen::Vector2d center(x, y);

//    return true;

    ROS_INFO_STREAM("center(x, y)");

    validStateCheckPtr vsck;

    ROS_INFO("Created validStateChecker smart pointer");
    for(grid_map::CircleIterator iterator(vsck->traversability_map_, center, radius); !iterator.isPastEnd(); ++iterator)
    {
        ROS_INFO("Meet traversability requirement");
        if(vsck->traversability_map_.at("traversability", *iterator) > 0.9)
            return true;
    }
}






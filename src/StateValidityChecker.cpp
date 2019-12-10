#include "StateValidityChecker.hpp"

ompl::base::myStateValidityChecker::myStateValidityChecker(const SpaceInformationPtr &si) : StateValidityChecker (si)
{
    traversability_map_sub_ = nh_.subscribe("/traversability_estimation/traversability_map", 1, &myStateValidityChecker::traversabilityMapCallback, this);
}

bool ompl::base::myStateValidityChecker::isValid(const State *state) const
{
    ROS_INFO("Test State is Valid");

    const SE2StateSpace::StateType *se2state = state->as<SE2StateSpace::StateType>();
    const RealVectorStateSpace::StateType *pos = se2state->as<RealVectorStateSpace::StateType>(0);
    //eric_wang: TODO what it do?
//    std::this_thread::sleep_for(ompl::time::seconds(0.0005));

    double x = pos->values[0];
    double y = pos->values[1];
    double yaw = pos->values[2];

    double radius = 0.5;
    Eigen::Vector2d center(x, y);

//    return true;

    for(grid_map::CircleIterator iterator(traversability_map_, center, radius); !iterator.isPastEnd(); ++iterator)
    {

        if(traversability_map_.at("traversability", *iterator) > 0.90)
        {
            ROS_INFO("Meet traversability requirement");
            return true;
        }
        else
        {
            ROS_INFO("Not meet traversability requirement");
            return false;
        }
    }
}


void ompl::base::myStateValidityChecker::traversabilityMapCallback(const grid_map_msgs::GridMapPtr &traversability_map)
{
    grid_map::GridMapRosConverter::fromMessage(*traversability_map, traversability_map_);// Dereference
    ROS_INFO("Receive traversability map successfully");
}

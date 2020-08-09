#include "StateValidityChecker.hpp"

//double traversability;
ompl::base::myStateValidityChecker::myStateValidityChecker(const SpaceInformationPtr &si)
    : StateValidityChecker (si),
      layer_("traversability")
{

    traversability_map_sub_ = nh_.subscribe("/traversability_estimation/traversability_map", 1, &myStateValidityChecker::traversabilityMapCallback, this);
    ROS_INFO("Constrcuting StateValidityChecker Class");
}

bool ompl::base::myStateValidityChecker::isValid(const State *state) const
{
//    ROS_INFO("Checking State Validity");

    const SE2StateSpace::StateType *se2state = state->as<SE2StateSpace::StateType>();
    const RealVectorStateSpace::StateType *pos = se2state->as<RealVectorStateSpace::StateType>(0);
    //eric_wang: TODO what it do?
//    std::this_thread::sleep_for(ompl::time::seconds(0.0005));

    double x = pos->values[0];
    double y = pos->values[1];
    double yaw = pos->values[2];

//    double radius = 0.35;
    double radius = 0.20;
    Eigen::Vector2d center(x, y);

    grid_map::Position position_(x, y);

    double traversability_footprint;

    double traversability = 0;
    //traversability_footprint = traversability_map_.atPosition("traversability_footprint", position_);
//    std::cout << "footprint traversability in traversability_footprint layer:" << traversability_footprint << std::endl;

    int nCells = 0;
    //todo check data is nan??
    for(grid_map::SpiralIterator iterator(traversability_map_, center, radius); !iterator.isPastEnd(); ++iterator)
    {
        float currentPositionIsTraversale = traversability_map_.at("traversability", *iterator);
        if(!traversability_map_.isValid(*iterator, "traversability"))
            ROS_WARN("Invalid data in traversability map");
//        std::cout << "current position traversability" << std::endl << currentPositionIsTraversale << std::endl;
        if(currentPositionIsTraversale > 0.50)
        {
            nCells++;
//            std::cout << nCells << std::endl;
//            std::cout << currentPositionIsTraversale << std::endl;
            traversability += traversability_map_.at("traversability", *iterator);
//            std::cout << "total traversability" << traversability << std::endl;
        }
        else
        {   //std::cout << "current position traversability is < 0.60" << std::endl;
            traversability = 0;
            break;
        }
    }
//    std::cout << traversability << std::endl;
    traversability /= nCells;
//    std::cout << "footprint traversability computed in traversability layer:" << traversability << std::endl;

    if(traversability > 0.7)
        return true;
    else
        return false;
}


void ompl::base::myStateValidityChecker::traversabilityMapCallback(const grid_map_msgs::GridMapPtr &traversability_map)
{
    grid_map::GridMapRosConverter::fromMessage(*traversability_map, traversability_map_);// Dereference
    ROS_INFO("State validity checker::Receive traversability map successfully");
    if(!traversability_map_.exists(layer_))
        ROS_WARN("State checker error because there exists no layer %s in traversability map.", layer_.c_str());
}

void ompl::base::myStateValidityChecker::localTraversabilityMapCallback(const grid_map_msgs::GridMapPtr &local_traversability_map)
{
    grid_map::GridMapRosConverter::fromMessage(*local_traversability_map, local_traversability_map_);
}

#include "MotionCostIntegralObjective.hpp"

double traversability;

ompl::base::motionCostIntegralObjective::motionCostIntegralObjective(const SpaceInformationPtr &si,
                                                                     bool enableMotionCostInterpolation)
    : OptimizationObjective(si), interpolateMotionCost_(enableMotionCostInterpolation)
{
    description_ = "Motion Cost Objective Writeen by Eirc Wang";

    traversability_map_sub_ = nh_.subscribe("/traversability_estimation/traversability_map", 1, &motionCostIntegralObjective::traversabilityMapCallback, this);
}

ompl::base::Cost ompl::base::motionCostIntegralObjective::stateCost(const State *s) const
{
    const SE2StateSpace::StateType *se2state = s->as<SE2StateSpace::StateType>();
    const RealVectorStateSpace::StateType *pos = se2state->as<RealVectorStateSpace::StateType>(0);

    double x = pos->values[0];
    double y = pos->values[1];
    Eigen::Vector2d center(x, y);
    grid_map::Position position_(x, y);

    int nCells = 0;
    double radius = 0.35;

    for(grid_map::SpiralIterator iterator(traversability_map_, center, radius); !iterator.isPastEnd(); ++iterator)
    {
        traversability += traversability_map_.at("traversability", *iterator);
    }

    double traversability_footprint;
    traversability_footprint = traversability / nCells;

    return Cost(1 / traversability_footprint); // Maximum (state) footprint traversability.
}

ompl::base::Cost ompl::base::motionCostIntegralObjective::motionCost(const State *s1, const State *s2) const
{
//    ROS_INFO("Motion cost integral objective::motion Cost");
    double distance_;

    distance_ = si_->distance(s1, s2); // distance between s1 and s2.

    if(interpolateMotionCost_)
    {
        Cost totalCost = this->identityCost();

        int nd = si_->getStateSpace()->validSegmentCount(s1, s2);

        State *test1 = si_->cloneState(s1);
        Cost prevStateCost = this->stateCost(test1);
        if(nd > 1)
        {
            State *test2 = si_->allocState();
            for(int j = 1; j < nd; ++j)
            {
                si_->getStateSpace()->interpolate(s1, s2, (double)j / (double)nd, test2);
                Cost nextStateCost = this->stateCost(test2);
                totalCost = Cost(totalCost.value() + this->trapezoid(prevStateCost, nextStateCost, si_->distance(test1, test2)).value());

                std::swap(test1, test2);
                prevStateCost = nextStateCost;
            }

            si_->freeState(test2);
        }

        totalCost = Cost(totalCost.value() + this->trapezoid(prevStateCost, this->stateCost(s2), si_->distance(test1, s2)).value());

        si_->freeState(test1);

        Cost costFunction(distance_ * (1 + totalCost.value()));
        return costFunction;
    }
}

void ompl::base::motionCostIntegralObjective::traversabilityMapCallback(const grid_map_msgs::GridMapPtr &traversability_map)
{
    grid_map::GridMapRosConverter::fromMessage(*traversability_map, traversability_map_);
    ROS_INFO("motionCostIntegralObjective::receive traversability map successfully");
}

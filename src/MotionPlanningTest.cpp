#include "MotionPlanningSampler.hpp"
#include "ompl/geometric/planners/prm/PRM.h"
#include "MotionPlanner.hpp"


namespace ob = ompl::base;
namespace og = ompl::geometric;


void plan()
{
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);

    ss.setStateValidityChecker(isStateValid);\

    ob::ScopedState<> start(space);
    start[0] = start[1] = start[2] = 0;

    ob::ScopedState<> goal(space);
    goal[0] = goal[1] = 0;
    goal[2] = 1;

    ss.setStartAndGoalStates(start, goal);

    ss.getSpaceInformation()->setValidStateSamplerAllocator(allocMyValidStateSampler);

    auto planner(std::make_shared<og::RRT_IM>(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    ob::PlannerStatus solved = ss.solve(10.0);
    if(solved)
    {
        std::cout << "Found Solution:" << std::endl;
        ss.getSolutionPath().print(std::cout);
    }
    else {
        std::cout << "No Solution Found" << std::endl;
    }
}

int main(int, char **)
{
    plan();
    return 0;
}

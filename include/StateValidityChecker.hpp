#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/config.h"
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/GetGridMap.h"
#include <ros/ros.h>
#include <thread>
#include <iostream>
#include <c++/5/ctime>



namespace ompl
{
namespace base
{
class myStateValidityChecker : public StateValidityChecker
{
public:
    myStateValidityChecker(const SpaceInformationPtr &si);

    bool isValid(const State *state) const override;

    void traversabilityMapCallback(const grid_map_msgs::GridMapPtr &traversability_map);

private:

    ros::NodeHandle nh_;

    ros::Subscriber traversability_map_sub_;

    grid_map::GridMap traversability_map_;
};
}
}

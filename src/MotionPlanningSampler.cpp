#include "MotionPlanningSampler.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;


bool MyValidStateSampler::sample(ob::State *state)
{
//    double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
//    double z = rng_.uniformReal(-1, 1);

//    if (z>.25 && z<.5)
//          {
//              double x = rng_.uniformReal(0,1.8), y = rng_.uniformReal(0,.2);
//              switch(rng_.uniformInt(0,3))
//              {
//                  case 0: val[0]=x-1;  val[1]=y-1;  break;
//                  case 1: val[0]=x-.8; val[1]=y+.8; break;
//                  case 2: val[0]=y-1;  val[1]=x-1;  break;
//                  case 3: val[0]=y+.8; val[1]=x-.8; break;
//              }
//          }
//          else
//          {
//              val[0] = rng_.uniformReal(-1,1);
//              val[1] = rng_.uniformReal(-1,1);
//          }
//          val[2] = z;
//    OMPL_INFORM("MyValidStateSampler");
//    assert(si_->isValid(state));
//    return true;
    bool valid = false;
    sampler_->sampleUniform(state);
    valid = si_->isValid(state);
    return valid;
}

bool MyValidStateSampler::sampleNear(ob::State *, const ob::State *, const double)
{
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
    return false;
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
    double radius = 1.0;
    Eigen::Vector2d center(x, y);

//    ROS_INFO_STREAM("center(x, y)");

//    validStateCheckPtr vsck;

//    for(grid_map::CircleIterator iterator(vsck->traversability_map_, center, radius); !iterator.isPastEnd(); ++iterator)
//    {
//        ROS_INFO("Meet traversability requirement");
//        if(vsck->traversability_map_.at("traversability", *iterator) > 0.9)
            return true;
//        else
//            return false;
//    }
}





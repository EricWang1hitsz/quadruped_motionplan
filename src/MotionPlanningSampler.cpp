#include "MotionPlanningSampler.hpp"


#include <memory.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


bool MyValidStateSampler::sample(ob::State *state)
{
    double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
    double z = rng_.uniformReal(-1, 1);

    if (z>.25 && z<.5)
          {
              double x = rng_.uniformReal(0,1.8), y = rng_.uniformReal(0,.2);
              switch(rng_.uniformInt(0,3))
              {
                  case 0: val[0]=x-1;  val[1]=y-1;  break;
                  case 1: val[0]=x-.8; val[1]=y+.8; break;
                  case 2: val[0]=y-1;  val[1]=x-1;  break;
                  case 3: val[0]=y+.8; val[1]=x-.8; break;
              }
          }
          else
          {
              val[0] = rng_.uniformReal(-1,1);
              val[1] = rng_.uniformReal(-1,1);
          }
          val[2] = z;
    OMPL_INFORM("MyValidStateSampler");
    assert(si_->isValid(state));
    return true;
}

bool MyValidStateSampler::sampleNear(ob::State *, const ob::State *, const double)
{
    throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
    return false;
}

bool isStateValid(const ob::State *state)
{
    OMPL_INFORM("Test State is Valid");

    const ob::RealVectorStateSpace::StateType& pos = *state->as<ob::RealVectorStateSpace::StateType>();

    std::this_thread::sleep_for(ompl::time::seconds(0.05));

    return !(fabs(pos[0]) < 0.8 && fabs(pos[1])< 0.8 && pos[2] > 0.25 && pos[2] < 0.5);
}





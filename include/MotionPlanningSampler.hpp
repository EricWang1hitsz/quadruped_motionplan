#pragma once

#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/config.h"

#include <thread>


namespace ob = ompl::base;
namespace og = ompl::geometric;

class MyValidStateSampler : public ob::ValidStateSampler
{
public:
    MyValidStateSampler(const ob::SpaceInformation *si) : ValidStateSampler(si)
    {
        name_ = "my sampler";
    }

    /**
     * @brief sample Sample the state
     * @param state
     * @return True
     */
    bool sample(ob::State *state) override;
    /**
     * @brief sampleNear
     * @return
     */
    bool sampleNear(ob::State*, const ob::State*, const double) override;

protected:
    ompl::RNG rng_;


};

ob::ValidStateSamplerPtr allocMyValidStateSampler(const ob::SpaceInformation *si)
{
    return std::make_shared<MyValidStateSampler>(si);
}

bool isStateValid(const ob::State *state);

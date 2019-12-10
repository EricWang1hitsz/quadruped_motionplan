#pragma once

#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateSpace.h"
#include <queue>
#include <ros/ros.h>


namespace ompl
{
namespace base
{
    class myMotionValidator : public MotionValidator
    {
    public:
        // Constructor.
        myMotionValidator(SpaceInformation *si) : MotionValidator(si)
        {
            defaultSettings();
        }

        myMotionValidator(const SpaceInformationPtr &si) : MotionValidator(si)
        {
            defaultSettings();
        }

        ~myMotionValidator() override = default;

        bool checkMotion(const State *si, const State *s2) const override;

        bool checkMotion(const State *si, const State *s2, std::pair<State *, double> &lastValid) const override;

    private:

        StateSpace *stateSpace_;

        void defaultSettings();

    };

}
}

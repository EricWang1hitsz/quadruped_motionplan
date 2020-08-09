#include "MotionValidityChecker.hpp"


void ompl::base::myMotionValidator::defaultSettings()
{
    // Return the instance of the used state space.
    stateSpace_ = si_->getStateSpace().get();
    if(stateSpace_ == nullptr)
        throw Exception("No state space for motion validator");
}

bool ompl::base::myMotionValidator::checkMotion(const State *s1, const State *s2) const
{
//    ROS_INFO("Checking Motion Validity");
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
    {
        invalid_++;
        return false;
    }
    bool result = true;
    unsigned int nd = stateSpace_->validSegmentCount(s1, s2);
    std::queue<std::pair<int, int>> pos;
    if(nd >= 2)
    {
//        ROS_INFO("Motion validity checker::checkMotion::nd >= 2");
        pos.emplace(1, nd - 1); //eric_wang: Add 1 and n-1 into pos queue.

        /* temporary storage for the checked state */
        State *test = si_->allocState(); // Allocate memory for a state.

        /* repeatedly subdivide the path segment in the middle (and check the middle state's validity) */
        while(!pos.empty())
        {
            std::pair<int, int> x = pos.front();
            int mid = (x.first + x.second) / 2;
            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);
            if(!si_->isValid(test))
            {
                result = false;
                break;
            }

            pos.pop(); //eric_wang: remove the first element.

            if(x.first < mid)
                pos.emplace(x.first, mid - 1);
            if(x.first > mid)
                pos.emplace(mid + 1, x.second);
        }

        si_->freeState(test);
    }

    if(result)
        valid_++;
    else
        invalid_++;

    return result;

}


bool ompl::base::myMotionValidator::checkMotion(const State *s1, const State *s2,
                                                      std::pair<State *, double> &lastValid) const
{
    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_INFORMED_RRTSTAR_
#define OMPL_GEOMETRIC_PLANNERS_RRT_INFORMED_RRTSTAR_

#include "MotionPlannerRRTstar.hpp"

namespace ompl
{
    namespace geometric
    {
        /**
            @anchor gInformedRRTstar

            Run \ref gRRTstar "RRT*" with an informed search strategy that uses heuristics to only consider subproblem
           that could provide a better solution.
            The search is limited to this subproblem by pruning the graph, generating samples only in this subproblem
           (directly if available, e.g., \ref gPathLengthDirectInfSampler "path length")
            and, when available, using the measure of this subproblem to calculate the connection terms (e.g., path
           length)

            @par Associated publication:

            J. D. Gammell, S. S. Srinivasa, T. D. Barfoot, "Informed RRT*: Optimal Sampling-based
            Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic." In Proceedings
            of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). Chicago, IL, USA,
            14-18 Sept. 2014.
            DOI: <a href="http://dx.doi.org/10.1109/IROS.2014.6942976">10.1109/IROS.2014.6942976</a>.
            <a href="https://www.youtube.com/watch?v=d7dX5MvDYTc">Illustration video</a>.
            <a href="https://www.youtube.com/watch?v=nsl-5MZfwu4">Short description video</a>.
        */

        /** \brief Informed RRT* */
        class InformedRRTstar_IM : public RRTstar
        {
        public:
            /** \brief Constructor */
            InformedRRTstar_IM(const base::SpaceInformationPtr &si);
        };
    }
}

#endif

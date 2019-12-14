#include "MotionPlannerInfRRTstar.hpp"

ompl::geometric::InformedRRTstar_IM::InformedRRTstar_IM(const base::SpaceInformationPtr &si) : RRTstar(si)
{
    OMPL_INFORM("Using Improved InformedRRTstar");
    // Set my name:
    setName("InformedRRTstar_IM");

    // Configure RRTstar to be InformedRRT*:
    setAdmissibleCostToCome(true);
    setInformedSampling(true);
    setTreePruning(true);
    setPrunedMeasure(true);

    // Disable conflicting options
    setSampleRejection(false);
    setNewStateRejection(false);

    // Remove those parameters:
    params_.remove("use_admissible_heuristic");
    params_.remove("informed_sampling");
    params_.remove("pruned_measure");
    params_.remove("tree_pruning");

    // Remove conflicting parameters:
    params_.remove("sample_rejection");
    params_.remove("new_state_rejection");
    params_.remove("focus_search");
}

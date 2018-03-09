#ifndef IIT_ROBOGEN__SLM_TRAITS_H_
#define IIT_ROBOGEN__SLM_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace iit {
namespace SLM {

struct Traits {
    typedef typename SLM::JointState JointState;

    typedef typename SLM::JointIdentifiers JointID;
    typedef typename SLM::LinkIdentifiers  LinkID;

    typedef typename SLM::HomogeneousTransforms HomogeneousTransforms;
    typedef typename SLM::MotionTransforms MotionTransforms;
    typedef typename SLM::ForceTransforms ForceTransforms;

    typedef typename SLM::dyn::InertiaProperties InertiaProperties;
    typedef typename SLM::dyn::ForwardDynamics FwdDynEngine;
    typedef typename SLM::dyn::InverseDynamics InvDynEngine;
    typedef typename SLM::dyn::JSIM JSIM;

    static const int joints_count = SLM::jointsCount;
    static const int links_count  = SLM::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return SLM::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return SLM::orderedLinkIDs;
}

}
}

#endif

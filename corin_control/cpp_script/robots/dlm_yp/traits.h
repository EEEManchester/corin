#ifndef IIT_ROBOGEN__DLM_YP_TRAITS_H_
#define IIT_ROBOGEN__DLM_YP_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace iit {
namespace DLM_YP {

struct Traits {
    typedef typename DLM_YP::JointState JointState;

    typedef typename DLM_YP::JointIdentifiers JointID;
    typedef typename DLM_YP::LinkIdentifiers  LinkID;

    typedef typename DLM_YP::HomogeneousTransforms HomogeneousTransforms;
    typedef typename DLM_YP::MotionTransforms MotionTransforms;
    typedef typename DLM_YP::ForceTransforms ForceTransforms;

    typedef typename DLM_YP::dyn::InertiaProperties InertiaProperties;
    typedef typename DLM_YP::dyn::ForwardDynamics FwdDynEngine;
    typedef typename DLM_YP::dyn::InverseDynamics InvDynEngine;
    typedef typename DLM_YP::dyn::JSIM JSIM;

    static const int joints_count = DLM_YP::jointsCount;
    static const int links_count  = DLM_YP::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return DLM_YP::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return DLM_YP::orderedLinkIDs;
}

}
}

#endif

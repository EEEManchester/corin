#ifndef IIT_ROBOGEN__MCORIN_LEG_TRAITS_H_
#define IIT_ROBOGEN__MCORIN_LEG_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace iit {
namespace MCORIN_LEG {

struct Traits {
    typedef typename MCORIN_LEG::JointState JointState;

    typedef typename MCORIN_LEG::JointIdentifiers JointID;
    typedef typename MCORIN_LEG::LinkIdentifiers  LinkID;

    typedef typename MCORIN_LEG::HomogeneousTransforms HomogeneousTransforms;
    typedef typename MCORIN_LEG::MotionTransforms MotionTransforms;
    typedef typename MCORIN_LEG::ForceTransforms ForceTransforms;

    typedef typename MCORIN_LEG::dyn::InertiaProperties InertiaProperties;
    typedef typename MCORIN_LEG::dyn::ForwardDynamics FwdDynEngine;
    typedef typename MCORIN_LEG::dyn::InverseDynamics InvDynEngine;
    typedef typename MCORIN_LEG::dyn::JSIM JSIM;

    static const int joints_count = MCORIN_LEG::jointsCount;
    static const int links_count  = MCORIN_LEG::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return MCORIN_LEG::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return MCORIN_LEG::orderedLinkIDs;
}

}
}

#endif

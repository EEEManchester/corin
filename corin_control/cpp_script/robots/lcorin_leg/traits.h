#ifndef IIT_ROBOGEN__LCORIN_LEG_TRAITS_H_
#define IIT_ROBOGEN__LCORIN_LEG_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace iit {
namespace LCORIN_LEG {

struct Traits {
    typedef typename LCORIN_LEG::JointState JointState;

    typedef typename LCORIN_LEG::JointIdentifiers JointID;
    typedef typename LCORIN_LEG::LinkIdentifiers  LinkID;

    typedef typename LCORIN_LEG::HomogeneousTransforms HomogeneousTransforms;
    typedef typename LCORIN_LEG::MotionTransforms MotionTransforms;
    typedef typename LCORIN_LEG::ForceTransforms ForceTransforms;

    typedef typename LCORIN_LEG::dyn::InertiaProperties InertiaProperties;
    typedef typename LCORIN_LEG::dyn::ForwardDynamics FwdDynEngine;
    typedef typename LCORIN_LEG::dyn::InverseDynamics InvDynEngine;
    typedef typename LCORIN_LEG::dyn::JSIM JSIM;

    static const int joints_count = LCORIN_LEG::jointsCount;
    static const int links_count  = LCORIN_LEG::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return LCORIN_LEG::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return LCORIN_LEG::orderedLinkIDs;
}

}
}

#endif

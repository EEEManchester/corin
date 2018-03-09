#ifndef IIT_ROBOGEN__MCORIN_TRAITS_H_
#define IIT_ROBOGEN__MCORIN_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace iit {
namespace mcorin {

struct Traits {
    typedef typename mcorin::JointState JointState;

    typedef typename mcorin::JointIdentifiers JointID;
    typedef typename mcorin::LinkIdentifiers  LinkID;

    typedef typename mcorin::HomogeneousTransforms HomogeneousTransforms;
    typedef typename mcorin::MotionTransforms MotionTransforms;
    typedef typename mcorin::ForceTransforms ForceTransforms;

    typedef typename mcorin::dyn::InertiaProperties InertiaProperties;
    typedef typename mcorin::dyn::ForwardDynamics FwdDynEngine;
    typedef typename mcorin::dyn::InverseDynamics InvDynEngine;
    typedef typename mcorin::dyn::JSIM JSIM;

    static const int joints_count = mcorin::jointsCount;
    static const int links_count  = mcorin::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return mcorin::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return mcorin::orderedLinkIDs;
}

}
}

#endif

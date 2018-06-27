#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::DLM_YP;
using namespace iit::DLM_YP::dyn;

iit::rbd::Vector3d iit::DLM_YP::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());


    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    tmpX = tmpX * ht.fr_base_X_fr_l1;
    tmpSum += inertiaProps.getMass_l1() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_l1()));
    
    tmpX = tmpX * ht.fr_l1_X_fr_l2;
    tmpSum += inertiaProps.getMass_l2() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_l2()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::DLM_YP::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base_X_fr_l1(q);
    ht.fr_l1_X_fr_l2(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}

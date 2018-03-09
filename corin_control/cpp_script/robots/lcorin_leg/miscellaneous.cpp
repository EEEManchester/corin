#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::LCORIN_LEG;
using namespace iit::LCORIN_LEG::dyn;

iit::rbd::Vector3d iit::LCORIN_LEG::getWholeBodyCOM(
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
    
    tmpX = tmpX * ht.fr_l2_X_fr_l3;
    tmpSum += inertiaProps.getMass_l3() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_l3()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::LCORIN_LEG::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base_X_fr_l1(q);
    ht.fr_l1_X_fr_l2(q);
    ht.fr_l2_X_fr_l3(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}

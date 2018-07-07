#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::mcorin;
using namespace iit::mcorin::dyn;

iit::rbd::Vector3d iit::mcorin::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());

    tmpSum += inertiaProps.getCOM_trunk() * inertiaProps.getMass_trunk();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    HomogeneousTransforms::MatrixType base_X_LF_q1_joint_chain;
    HomogeneousTransforms::MatrixType base_X_LM_q1_joint_chain;
    HomogeneousTransforms::MatrixType base_X_LR_q1_joint_chain;
    HomogeneousTransforms::MatrixType base_X_RF_q1_joint_chain;
    HomogeneousTransforms::MatrixType base_X_RM_q1_joint_chain;
    HomogeneousTransforms::MatrixType base_X_RR_q1_joint_chain;
    
    
    base_X_LF_q1_joint_chain = tmpX * ht.fr_trunk_X_fr_LF_hipassembly;
    tmpSum += inertiaProps.getMass_LF_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_LF_q1_joint_chain, inertiaProps.getCOM_LF_hipassembly()));
    // std::cout << "=========================================" << std::endl;
    // std::cout << base_X_LF_q1_joint_chain << std::endl;
    // std::cout << ht.fr_trunk_X_fr_LF_hipassembly << std::endl;
    base_X_LF_q1_joint_chain = base_X_LF_q1_joint_chain * ht.fr_LF_hipassembly_X_fr_LF_upperleg;
    tmpSum += inertiaProps.getMass_LF_upperleg() *
            ( iit::rbd::Utils::transform(base_X_LF_q1_joint_chain, inertiaProps.getCOM_LF_upperleg()));
    
    base_X_LF_q1_joint_chain = base_X_LF_q1_joint_chain * ht.fr_LF_upperleg_X_fr_LF_lowerleg;
    tmpSum += inertiaProps.getMass_LF_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_LF_q1_joint_chain, inertiaProps.getCOM_LF_lowerleg()));
    
    base_X_LM_q1_joint_chain = tmpX * ht.fr_trunk_X_fr_LM_hipassembly;
    tmpSum += inertiaProps.getMass_LM_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_LM_q1_joint_chain, inertiaProps.getCOM_LM_hipassembly()));
    
    base_X_LM_q1_joint_chain = base_X_LM_q1_joint_chain * ht.fr_LM_hipassembly_X_fr_LM_upperleg;
    tmpSum += inertiaProps.getMass_LM_upperleg() *
            ( iit::rbd::Utils::transform(base_X_LM_q1_joint_chain, inertiaProps.getCOM_LM_upperleg()));
    
    base_X_LM_q1_joint_chain = base_X_LM_q1_joint_chain * ht.fr_LM_upperleg_X_fr_LM_lowerleg;
    tmpSum += inertiaProps.getMass_LM_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_LM_q1_joint_chain, inertiaProps.getCOM_LM_lowerleg()));
    
    base_X_LR_q1_joint_chain = tmpX * ht.fr_trunk_X_fr_LR_hipassembly;
    tmpSum += inertiaProps.getMass_LR_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_LR_q1_joint_chain, inertiaProps.getCOM_LR_hipassembly()));
    
    base_X_LR_q1_joint_chain = base_X_LR_q1_joint_chain * ht.fr_LR_hipassembly_X_fr_LR_upperleg;
    tmpSum += inertiaProps.getMass_LR_upperleg() *
            ( iit::rbd::Utils::transform(base_X_LR_q1_joint_chain, inertiaProps.getCOM_LR_upperleg()));
    
    base_X_LR_q1_joint_chain = base_X_LR_q1_joint_chain * ht.fr_LR_upperleg_X_fr_LR_lowerleg;
    tmpSum += inertiaProps.getMass_LR_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_LR_q1_joint_chain, inertiaProps.getCOM_LR_lowerleg()));
    
    base_X_RF_q1_joint_chain = tmpX * ht.fr_trunk_X_fr_RF_hipassembly;
    tmpSum += inertiaProps.getMass_RF_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_RF_q1_joint_chain, inertiaProps.getCOM_RF_hipassembly()));
    
    base_X_RF_q1_joint_chain = base_X_RF_q1_joint_chain * ht.fr_RF_hipassembly_X_fr_RF_upperleg;
    tmpSum += inertiaProps.getMass_RF_upperleg() *
            ( iit::rbd::Utils::transform(base_X_RF_q1_joint_chain, inertiaProps.getCOM_RF_upperleg()));
    
    base_X_RF_q1_joint_chain = base_X_RF_q1_joint_chain * ht.fr_RF_upperleg_X_fr_RF_lowerleg;
    tmpSum += inertiaProps.getMass_RF_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_RF_q1_joint_chain, inertiaProps.getCOM_RF_lowerleg()));
    
    base_X_RM_q1_joint_chain = tmpX * ht.fr_trunk_X_fr_RM_hipassembly;
    tmpSum += inertiaProps.getMass_RM_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_RM_q1_joint_chain, inertiaProps.getCOM_RM_hipassembly()));
    
    base_X_RM_q1_joint_chain = base_X_RM_q1_joint_chain * ht.fr_RM_hipassembly_X_fr_RM_upperleg;
    tmpSum += inertiaProps.getMass_RM_upperleg() *
            ( iit::rbd::Utils::transform(base_X_RM_q1_joint_chain, inertiaProps.getCOM_RM_upperleg()));
    
    base_X_RM_q1_joint_chain = base_X_RM_q1_joint_chain * ht.fr_RM_upperleg_X_fr_RM_lowerleg;
    tmpSum += inertiaProps.getMass_RM_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_RM_q1_joint_chain, inertiaProps.getCOM_RM_lowerleg()));
    
    base_X_RR_q1_joint_chain = tmpX * ht.fr_trunk_X_fr_RR_hipassembly;
    tmpSum += inertiaProps.getMass_RR_hipassembly() *
            ( iit::rbd::Utils::transform(base_X_RR_q1_joint_chain, inertiaProps.getCOM_RR_hipassembly()));
    
    base_X_RR_q1_joint_chain = base_X_RR_q1_joint_chain * ht.fr_RR_hipassembly_X_fr_RR_upperleg;
    tmpSum += inertiaProps.getMass_RR_upperleg() *
            ( iit::rbd::Utils::transform(base_X_RR_q1_joint_chain, inertiaProps.getCOM_RR_upperleg()));
    
    base_X_RR_q1_joint_chain = base_X_RR_q1_joint_chain * ht.fr_RR_upperleg_X_fr_RR_lowerleg;
    tmpSum += inertiaProps.getMass_RR_lowerleg() *
            ( iit::rbd::Utils::transform(base_X_RR_q1_joint_chain, inertiaProps.getCOM_RR_lowerleg()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::mcorin::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_trunk_X_fr_LF_hipassembly(q);
    ht.fr_trunk_X_fr_LM_hipassembly(q);
    ht.fr_trunk_X_fr_LR_hipassembly(q);
    ht.fr_trunk_X_fr_RF_hipassembly(q);
    ht.fr_trunk_X_fr_RM_hipassembly(q);
    ht.fr_trunk_X_fr_RR_hipassembly(q);
    ht.fr_LF_hipassembly_X_fr_LF_upperleg(q);
    ht.fr_LF_upperleg_X_fr_LF_lowerleg(q);
    ht.fr_LM_hipassembly_X_fr_LM_upperleg(q);
    ht.fr_LM_upperleg_X_fr_LM_lowerleg(q);
    ht.fr_LR_hipassembly_X_fr_LR_upperleg(q);
    ht.fr_LR_upperleg_X_fr_LR_lowerleg(q);
    ht.fr_RF_hipassembly_X_fr_RF_upperleg(q);
    ht.fr_RF_upperleg_X_fr_RF_lowerleg(q);
    ht.fr_RM_hipassembly_X_fr_RM_upperleg(q);
    ht.fr_RM_upperleg_X_fr_RM_lowerleg(q);
    ht.fr_RR_hipassembly_X_fr_RR_upperleg(q);
    ht.fr_RR_upperleg_X_fr_RR_lowerleg(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}

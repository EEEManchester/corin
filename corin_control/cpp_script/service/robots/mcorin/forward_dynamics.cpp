#include "forward_dynamics.h"

#include <Eigen/Cholesky>
#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

// Initialization of static-const data
const iit::mcorin::dyn::ForwardDynamics::ExtForces
    iit::mcorin::dyn::ForwardDynamics::zeroExtForces(Force::Zero());

iit::mcorin::dyn::ForwardDynamics::ForwardDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    LF_hipassembly_v.setZero();
    LF_hipassembly_c.setZero();
    LF_upperleg_v.setZero();
    LF_upperleg_c.setZero();
    LF_lowerleg_v.setZero();
    LF_lowerleg_c.setZero();
    LM_hipassembly_v.setZero();
    LM_hipassembly_c.setZero();
    LM_upperleg_v.setZero();
    LM_upperleg_c.setZero();
    LM_lowerleg_v.setZero();
    LM_lowerleg_c.setZero();
    LR_hipassembly_v.setZero();
    LR_hipassembly_c.setZero();
    LR_upperleg_v.setZero();
    LR_upperleg_c.setZero();
    LR_lowerleg_v.setZero();
    LR_lowerleg_c.setZero();
    RF_hipassembly_v.setZero();
    RF_hipassembly_c.setZero();
    RF_upperleg_v.setZero();
    RF_upperleg_c.setZero();
    RF_lowerleg_v.setZero();
    RF_lowerleg_c.setZero();
    RM_hipassembly_v.setZero();
    RM_hipassembly_c.setZero();
    RM_upperleg_v.setZero();
    RM_upperleg_c.setZero();
    RM_lowerleg_v.setZero();
    RM_lowerleg_c.setZero();
    RR_hipassembly_v.setZero();
    RR_hipassembly_c.setZero();
    RR_upperleg_v.setZero();
    RR_upperleg_c.setZero();
    RR_lowerleg_v.setZero();
    RR_lowerleg_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

void iit::mcorin::dyn::ForwardDynamics::fd(
    JointState& qdd,
    Acceleration& trunk_a,
    const Velocity& trunk_v,
    const Acceleration& g,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    trunk_AI = inertiaProps->getTensor_trunk();
    trunk_p = - fext[TRUNK];
    LF_hipassembly_AI = inertiaProps->getTensor_LF_hipassembly();
    LF_hipassembly_p = - fext[LF_HIPASSEMBLY];
    RF_hipassembly_AI = inertiaProps->getTensor_RF_hipassembly();
    RF_hipassembly_p = - fext[RF_HIPASSEMBLY];
    LF_upperleg_AI = inertiaProps->getTensor_LF_upperleg();
    LF_upperleg_p = - fext[LF_UPPERLEG];
    RF_upperleg_AI = inertiaProps->getTensor_RF_upperleg();
    RF_upperleg_p = - fext[RF_UPPERLEG];
    LF_lowerleg_AI = inertiaProps->getTensor_LF_lowerleg();
    LF_lowerleg_p = - fext[LF_LOWERLEG];
    RF_lowerleg_AI = inertiaProps->getTensor_RF_lowerleg();
    RF_lowerleg_p = - fext[RF_LOWERLEG];
    LM_hipassembly_AI = inertiaProps->getTensor_LM_hipassembly();
    LM_hipassembly_p = - fext[LM_HIPASSEMBLY];
    LR_hipassembly_AI = inertiaProps->getTensor_LR_hipassembly();
    LR_hipassembly_p = - fext[LR_HIPASSEMBLY];
    RM_hipassembly_AI = inertiaProps->getTensor_RM_hipassembly();
    RM_hipassembly_p = - fext[RM_HIPASSEMBLY];
    RR_hipassembly_AI = inertiaProps->getTensor_RR_hipassembly();
    RR_hipassembly_p = - fext[RR_HIPASSEMBLY];
    LM_upperleg_AI = inertiaProps->getTensor_LM_upperleg();
    LM_upperleg_p = - fext[LM_UPPERLEG];
    LR_upperleg_AI = inertiaProps->getTensor_LR_upperleg();
    LR_upperleg_p = - fext[LR_UPPERLEG];
    RM_upperleg_AI = inertiaProps->getTensor_RM_upperleg();
    RM_upperleg_p = - fext[RM_UPPERLEG];
    RR_upperleg_AI = inertiaProps->getTensor_RR_upperleg();
    RR_upperleg_p = - fext[RR_UPPERLEG];
    LM_lowerleg_AI = inertiaProps->getTensor_LM_lowerleg();
    LM_lowerleg_p = - fext[LM_LOWERLEG];
    LR_lowerleg_AI = inertiaProps->getTensor_LR_lowerleg();
    LR_lowerleg_p = - fext[LR_LOWERLEG];
    RM_lowerleg_AI = inertiaProps->getTensor_RM_lowerleg();
    RM_lowerleg_p = - fext[RM_LOWERLEG];
    RR_lowerleg_AI = inertiaProps->getTensor_RR_lowerleg();
    RR_lowerleg_p = - fext[RR_LOWERLEG];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link LF_hipassembly
    //  - The spatial velocity:
    LF_hipassembly_v = (motionTransforms-> fr_LF_hipassembly_X_fr_trunk) * trunk_v;
    LF_hipassembly_v(AZ) += qd(LF_Q1_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(LF_hipassembly_v, vcross);
    LF_hipassembly_c = vcross.col(AZ) * qd(LF_Q1_JOINT);
    
    //  - The bias force term:
    LF_hipassembly_p += vxIv(LF_hipassembly_v, LF_hipassembly_AI);
    
    // + Link RF_hipassembly
    //  - The spatial velocity:
    RF_hipassembly_v = (motionTransforms-> fr_RF_hipassembly_X_fr_trunk) * trunk_v;
    RF_hipassembly_v(AZ) += qd(RF_Q1_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(RF_hipassembly_v, vcross);
    RF_hipassembly_c = vcross.col(AZ) * qd(RF_Q1_JOINT);
    
    //  - The bias force term:
    RF_hipassembly_p += vxIv(RF_hipassembly_v, RF_hipassembly_AI);
    
    // + Link LF_upperleg
    //  - The spatial velocity:
    LF_upperleg_v = (motionTransforms-> fr_LF_upperleg_X_fr_LF_hipassembly) * LF_hipassembly_v;
    LF_upperleg_v(AZ) += qd(LF_Q2_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(LF_upperleg_v, vcross);
    LF_upperleg_c = vcross.col(AZ) * qd(LF_Q2_JOINT);
    
    //  - The bias force term:
    LF_upperleg_p += vxIv(LF_upperleg_v, LF_upperleg_AI);
    
    // + Link RF_upperleg
    //  - The spatial velocity:
    RF_upperleg_v = (motionTransforms-> fr_RF_upperleg_X_fr_RF_hipassembly) * RF_hipassembly_v;
    RF_upperleg_v(AZ) += qd(RF_Q2_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(RF_upperleg_v, vcross);
    RF_upperleg_c = vcross.col(AZ) * qd(RF_Q2_JOINT);
    
    //  - The bias force term:
    RF_upperleg_p += vxIv(RF_upperleg_v, RF_upperleg_AI);
    
    // + Link LF_lowerleg
    //  - The spatial velocity:
    LF_lowerleg_v = (motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_v;
    LF_lowerleg_v(AZ) += qd(LF_Q3_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(LF_lowerleg_v, vcross);
    LF_lowerleg_c = vcross.col(AZ) * qd(LF_Q3_JOINT);
    
    //  - The bias force term:
    LF_lowerleg_p += vxIv(LF_lowerleg_v, LF_lowerleg_AI);
    
    // + Link RF_lowerleg
    //  - The spatial velocity:
    RF_lowerleg_v = (motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_v;
    RF_lowerleg_v(AZ) += qd(RF_Q3_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(RF_lowerleg_v, vcross);
    RF_lowerleg_c = vcross.col(AZ) * qd(RF_Q3_JOINT);
    
    //  - The bias force term:
    RF_lowerleg_p += vxIv(RF_lowerleg_v, RF_lowerleg_AI);
    
    // + Link LM_hipassembly
    //  - The spatial velocity:
    LM_hipassembly_v = (motionTransforms-> fr_LM_hipassembly_X_fr_trunk) * trunk_v;
    LM_hipassembly_v(AZ) += qd(LM_Q1_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(LM_hipassembly_v, vcross);
    LM_hipassembly_c = vcross.col(AZ) * qd(LM_Q1_JOINT);
    
    //  - The bias force term:
    LM_hipassembly_p += vxIv(LM_hipassembly_v, LM_hipassembly_AI);
    
    // + Link LR_hipassembly
    //  - The spatial velocity:
    LR_hipassembly_v = (motionTransforms-> fr_LR_hipassembly_X_fr_trunk) * trunk_v;
    LR_hipassembly_v(AZ) += qd(LR_Q1_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(LR_hipassembly_v, vcross);
    LR_hipassembly_c = vcross.col(AZ) * qd(LR_Q1_JOINT);
    
    //  - The bias force term:
    LR_hipassembly_p += vxIv(LR_hipassembly_v, LR_hipassembly_AI);
    
    // + Link RM_hipassembly
    //  - The spatial velocity:
    RM_hipassembly_v = (motionTransforms-> fr_RM_hipassembly_X_fr_trunk) * trunk_v;
    RM_hipassembly_v(AZ) += qd(RM_Q1_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(RM_hipassembly_v, vcross);
    RM_hipassembly_c = vcross.col(AZ) * qd(RM_Q1_JOINT);
    
    //  - The bias force term:
    RM_hipassembly_p += vxIv(RM_hipassembly_v, RM_hipassembly_AI);
    
    // + Link RR_hipassembly
    //  - The spatial velocity:
    RR_hipassembly_v = (motionTransforms-> fr_RR_hipassembly_X_fr_trunk) * trunk_v;
    RR_hipassembly_v(AZ) += qd(RR_Q1_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(RR_hipassembly_v, vcross);
    RR_hipassembly_c = vcross.col(AZ) * qd(RR_Q1_JOINT);
    
    //  - The bias force term:
    RR_hipassembly_p += vxIv(RR_hipassembly_v, RR_hipassembly_AI);
    
    // + Link LM_upperleg
    //  - The spatial velocity:
    LM_upperleg_v = (motionTransforms-> fr_LM_upperleg_X_fr_LM_hipassembly) * LM_hipassembly_v;
    LM_upperleg_v(AZ) += qd(LM_Q2_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(LM_upperleg_v, vcross);
    LM_upperleg_c = vcross.col(AZ) * qd(LM_Q2_JOINT);
    
    //  - The bias force term:
    LM_upperleg_p += vxIv(LM_upperleg_v, LM_upperleg_AI);
    
    // + Link LR_upperleg
    //  - The spatial velocity:
    LR_upperleg_v = (motionTransforms-> fr_LR_upperleg_X_fr_LR_hipassembly) * LR_hipassembly_v;
    LR_upperleg_v(AZ) += qd(LR_Q2_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(LR_upperleg_v, vcross);
    LR_upperleg_c = vcross.col(AZ) * qd(LR_Q2_JOINT);
    
    //  - The bias force term:
    LR_upperleg_p += vxIv(LR_upperleg_v, LR_upperleg_AI);
    
    // + Link RM_upperleg
    //  - The spatial velocity:
    RM_upperleg_v = (motionTransforms-> fr_RM_upperleg_X_fr_RM_hipassembly) * RM_hipassembly_v;
    RM_upperleg_v(AZ) += qd(RM_Q2_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(RM_upperleg_v, vcross);
    RM_upperleg_c = vcross.col(AZ) * qd(RM_Q2_JOINT);
    
    //  - The bias force term:
    RM_upperleg_p += vxIv(RM_upperleg_v, RM_upperleg_AI);
    
    // + Link RR_upperleg
    //  - The spatial velocity:
    RR_upperleg_v = (motionTransforms-> fr_RR_upperleg_X_fr_RR_hipassembly) * RR_hipassembly_v;
    RR_upperleg_v(AZ) += qd(RR_Q2_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(RR_upperleg_v, vcross);
    RR_upperleg_c = vcross.col(AZ) * qd(RR_Q2_JOINT);
    
    //  - The bias force term:
    RR_upperleg_p += vxIv(RR_upperleg_v, RR_upperleg_AI);
    
    // + Link LM_lowerleg
    //  - The spatial velocity:
    LM_lowerleg_v = (motionTransforms-> fr_LM_lowerleg_X_fr_LM_upperleg) * LM_upperleg_v;
    LM_lowerleg_v(AZ) += qd(LM_Q3_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(LM_lowerleg_v, vcross);
    LM_lowerleg_c = vcross.col(AZ) * qd(LM_Q3_JOINT);
    
    //  - The bias force term:
    LM_lowerleg_p += vxIv(LM_lowerleg_v, LM_lowerleg_AI);
    
    // + Link LR_lowerleg
    //  - The spatial velocity:
    LR_lowerleg_v = (motionTransforms-> fr_LR_lowerleg_X_fr_LR_upperleg) * LR_upperleg_v;
    LR_lowerleg_v(AZ) += qd(LR_Q3_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(LR_lowerleg_v, vcross);
    LR_lowerleg_c = vcross.col(AZ) * qd(LR_Q3_JOINT);
    
    //  - The bias force term:
    LR_lowerleg_p += vxIv(LR_lowerleg_v, LR_lowerleg_AI);
    
    // + Link RM_lowerleg
    //  - The spatial velocity:
    RM_lowerleg_v = (motionTransforms-> fr_RM_lowerleg_X_fr_RM_upperleg) * RM_upperleg_v;
    RM_lowerleg_v(AZ) += qd(RM_Q3_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(RM_lowerleg_v, vcross);
    RM_lowerleg_c = vcross.col(AZ) * qd(RM_Q3_JOINT);
    
    //  - The bias force term:
    RM_lowerleg_p += vxIv(RM_lowerleg_v, RM_lowerleg_AI);
    
    // + Link RR_lowerleg
    //  - The spatial velocity:
    RR_lowerleg_v = (motionTransforms-> fr_RR_lowerleg_X_fr_RR_upperleg) * RR_upperleg_v;
    RR_lowerleg_v(AZ) += qd(RR_Q3_JOINT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(RR_lowerleg_v, vcross);
    RR_lowerleg_c = vcross.col(AZ) * qd(RR_Q3_JOINT);
    
    //  - The bias force term:
    RR_lowerleg_p += vxIv(RR_lowerleg_v, RR_lowerleg_AI);
    
    // + The floating base body
    trunk_p += vxIv(trunk_v, trunk_AI);
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66d IaB;
    Force pa;
    
    // + Link RR_lowerleg
    RR_lowerleg_u = tau(RR_Q3_JOINT) - RR_lowerleg_p(AZ);
    RR_lowerleg_U = RR_lowerleg_AI.col(AZ);
    RR_lowerleg_D = RR_lowerleg_U(AZ);
    
    compute_Ia_revolute(RR_lowerleg_AI, RR_lowerleg_U, RR_lowerleg_D, Ia_r);  // same as: Ia_r = RR_lowerleg_AI - RR_lowerleg_U/RR_lowerleg_D * RR_lowerleg_U.transpose();
    pa = RR_lowerleg_p + Ia_r * RR_lowerleg_c + RR_lowerleg_U * RR_lowerleg_u/RR_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RR_lowerleg_X_fr_RR_upperleg, IaB);
    RR_upperleg_AI += IaB;
    RR_upperleg_p += (motionTransforms-> fr_RR_lowerleg_X_fr_RR_upperleg).transpose() * pa;
    
    // + Link RM_lowerleg
    RM_lowerleg_u = tau(RM_Q3_JOINT) - RM_lowerleg_p(AZ);
    RM_lowerleg_U = RM_lowerleg_AI.col(AZ);
    RM_lowerleg_D = RM_lowerleg_U(AZ);
    
    compute_Ia_revolute(RM_lowerleg_AI, RM_lowerleg_U, RM_lowerleg_D, Ia_r);  // same as: Ia_r = RM_lowerleg_AI - RM_lowerleg_U/RM_lowerleg_D * RM_lowerleg_U.transpose();
    pa = RM_lowerleg_p + Ia_r * RM_lowerleg_c + RM_lowerleg_U * RM_lowerleg_u/RM_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RM_lowerleg_X_fr_RM_upperleg, IaB);
    RM_upperleg_AI += IaB;
    RM_upperleg_p += (motionTransforms-> fr_RM_lowerleg_X_fr_RM_upperleg).transpose() * pa;
    
    // + Link LR_lowerleg
    LR_lowerleg_u = tau(LR_Q3_JOINT) - LR_lowerleg_p(AZ);
    LR_lowerleg_U = LR_lowerleg_AI.col(AZ);
    LR_lowerleg_D = LR_lowerleg_U(AZ);
    
    compute_Ia_revolute(LR_lowerleg_AI, LR_lowerleg_U, LR_lowerleg_D, Ia_r);  // same as: Ia_r = LR_lowerleg_AI - LR_lowerleg_U/LR_lowerleg_D * LR_lowerleg_U.transpose();
    pa = LR_lowerleg_p + Ia_r * LR_lowerleg_c + LR_lowerleg_U * LR_lowerleg_u/LR_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LR_lowerleg_X_fr_LR_upperleg, IaB);
    LR_upperleg_AI += IaB;
    LR_upperleg_p += (motionTransforms-> fr_LR_lowerleg_X_fr_LR_upperleg).transpose() * pa;
    
    // + Link LM_lowerleg
    LM_lowerleg_u = tau(LM_Q3_JOINT) - LM_lowerleg_p(AZ);
    LM_lowerleg_U = LM_lowerleg_AI.col(AZ);
    LM_lowerleg_D = LM_lowerleg_U(AZ);
    
    compute_Ia_revolute(LM_lowerleg_AI, LM_lowerleg_U, LM_lowerleg_D, Ia_r);  // same as: Ia_r = LM_lowerleg_AI - LM_lowerleg_U/LM_lowerleg_D * LM_lowerleg_U.transpose();
    pa = LM_lowerleg_p + Ia_r * LM_lowerleg_c + LM_lowerleg_U * LM_lowerleg_u/LM_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LM_lowerleg_X_fr_LM_upperleg, IaB);
    LM_upperleg_AI += IaB;
    LM_upperleg_p += (motionTransforms-> fr_LM_lowerleg_X_fr_LM_upperleg).transpose() * pa;
    
    // + Link RR_upperleg
    RR_upperleg_u = tau(RR_Q2_JOINT) - RR_upperleg_p(AZ);
    RR_upperleg_U = RR_upperleg_AI.col(AZ);
    RR_upperleg_D = RR_upperleg_U(AZ);
    
    compute_Ia_revolute(RR_upperleg_AI, RR_upperleg_U, RR_upperleg_D, Ia_r);  // same as: Ia_r = RR_upperleg_AI - RR_upperleg_U/RR_upperleg_D * RR_upperleg_U.transpose();
    pa = RR_upperleg_p + Ia_r * RR_upperleg_c + RR_upperleg_U * RR_upperleg_u/RR_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RR_upperleg_X_fr_RR_hipassembly, IaB);
    RR_hipassembly_AI += IaB;
    RR_hipassembly_p += (motionTransforms-> fr_RR_upperleg_X_fr_RR_hipassembly).transpose() * pa;
    
    // + Link RM_upperleg
    RM_upperleg_u = tau(RM_Q2_JOINT) - RM_upperleg_p(AZ);
    RM_upperleg_U = RM_upperleg_AI.col(AZ);
    RM_upperleg_D = RM_upperleg_U(AZ);
    
    compute_Ia_revolute(RM_upperleg_AI, RM_upperleg_U, RM_upperleg_D, Ia_r);  // same as: Ia_r = RM_upperleg_AI - RM_upperleg_U/RM_upperleg_D * RM_upperleg_U.transpose();
    pa = RM_upperleg_p + Ia_r * RM_upperleg_c + RM_upperleg_U * RM_upperleg_u/RM_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RM_upperleg_X_fr_RM_hipassembly, IaB);
    RM_hipassembly_AI += IaB;
    RM_hipassembly_p += (motionTransforms-> fr_RM_upperleg_X_fr_RM_hipassembly).transpose() * pa;
    
    // + Link LR_upperleg
    LR_upperleg_u = tau(LR_Q2_JOINT) - LR_upperleg_p(AZ);
    LR_upperleg_U = LR_upperleg_AI.col(AZ);
    LR_upperleg_D = LR_upperleg_U(AZ);
    
    compute_Ia_revolute(LR_upperleg_AI, LR_upperleg_U, LR_upperleg_D, Ia_r);  // same as: Ia_r = LR_upperleg_AI - LR_upperleg_U/LR_upperleg_D * LR_upperleg_U.transpose();
    pa = LR_upperleg_p + Ia_r * LR_upperleg_c + LR_upperleg_U * LR_upperleg_u/LR_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LR_upperleg_X_fr_LR_hipassembly, IaB);
    LR_hipassembly_AI += IaB;
    LR_hipassembly_p += (motionTransforms-> fr_LR_upperleg_X_fr_LR_hipassembly).transpose() * pa;
    
    // + Link LM_upperleg
    LM_upperleg_u = tau(LM_Q2_JOINT) - LM_upperleg_p(AZ);
    LM_upperleg_U = LM_upperleg_AI.col(AZ);
    LM_upperleg_D = LM_upperleg_U(AZ);
    
    compute_Ia_revolute(LM_upperleg_AI, LM_upperleg_U, LM_upperleg_D, Ia_r);  // same as: Ia_r = LM_upperleg_AI - LM_upperleg_U/LM_upperleg_D * LM_upperleg_U.transpose();
    pa = LM_upperleg_p + Ia_r * LM_upperleg_c + LM_upperleg_U * LM_upperleg_u/LM_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LM_upperleg_X_fr_LM_hipassembly, IaB);
    LM_hipassembly_AI += IaB;
    LM_hipassembly_p += (motionTransforms-> fr_LM_upperleg_X_fr_LM_hipassembly).transpose() * pa;
    
    // + Link RR_hipassembly
    RR_hipassembly_u = tau(RR_Q1_JOINT) - RR_hipassembly_p(AZ);
    RR_hipassembly_U = RR_hipassembly_AI.col(AZ);
    RR_hipassembly_D = RR_hipassembly_U(AZ);
    
    compute_Ia_revolute(RR_hipassembly_AI, RR_hipassembly_U, RR_hipassembly_D, Ia_r);  // same as: Ia_r = RR_hipassembly_AI - RR_hipassembly_U/RR_hipassembly_D * RR_hipassembly_U.transpose();
    pa = RR_hipassembly_p + Ia_r * RR_hipassembly_c + RR_hipassembly_U * RR_hipassembly_u/RR_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RR_hipassembly_X_fr_trunk, IaB);
    trunk_AI += IaB;
    trunk_p += (motionTransforms-> fr_RR_hipassembly_X_fr_trunk).transpose() * pa;
    
    // + Link RM_hipassembly
    RM_hipassembly_u = tau(RM_Q1_JOINT) - RM_hipassembly_p(AZ);
    RM_hipassembly_U = RM_hipassembly_AI.col(AZ);
    RM_hipassembly_D = RM_hipassembly_U(AZ);
    
    compute_Ia_revolute(RM_hipassembly_AI, RM_hipassembly_U, RM_hipassembly_D, Ia_r);  // same as: Ia_r = RM_hipassembly_AI - RM_hipassembly_U/RM_hipassembly_D * RM_hipassembly_U.transpose();
    pa = RM_hipassembly_p + Ia_r * RM_hipassembly_c + RM_hipassembly_U * RM_hipassembly_u/RM_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RM_hipassembly_X_fr_trunk, IaB);
    trunk_AI += IaB;
    trunk_p += (motionTransforms-> fr_RM_hipassembly_X_fr_trunk).transpose() * pa;
    
    // + Link LR_hipassembly
    LR_hipassembly_u = tau(LR_Q1_JOINT) - LR_hipassembly_p(AZ);
    LR_hipassembly_U = LR_hipassembly_AI.col(AZ);
    LR_hipassembly_D = LR_hipassembly_U(AZ);
    
    compute_Ia_revolute(LR_hipassembly_AI, LR_hipassembly_U, LR_hipassembly_D, Ia_r);  // same as: Ia_r = LR_hipassembly_AI - LR_hipassembly_U/LR_hipassembly_D * LR_hipassembly_U.transpose();
    pa = LR_hipassembly_p + Ia_r * LR_hipassembly_c + LR_hipassembly_U * LR_hipassembly_u/LR_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LR_hipassembly_X_fr_trunk, IaB);
    trunk_AI += IaB;
    trunk_p += (motionTransforms-> fr_LR_hipassembly_X_fr_trunk).transpose() * pa;
    
    // + Link LM_hipassembly
    LM_hipassembly_u = tau(LM_Q1_JOINT) - LM_hipassembly_p(AZ);
    LM_hipassembly_U = LM_hipassembly_AI.col(AZ);
    LM_hipassembly_D = LM_hipassembly_U(AZ);
    
    compute_Ia_revolute(LM_hipassembly_AI, LM_hipassembly_U, LM_hipassembly_D, Ia_r);  // same as: Ia_r = LM_hipassembly_AI - LM_hipassembly_U/LM_hipassembly_D * LM_hipassembly_U.transpose();
    pa = LM_hipassembly_p + Ia_r * LM_hipassembly_c + LM_hipassembly_U * LM_hipassembly_u/LM_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LM_hipassembly_X_fr_trunk, IaB);
    trunk_AI += IaB;
    trunk_p += (motionTransforms-> fr_LM_hipassembly_X_fr_trunk).transpose() * pa;
    
    // + Link RF_lowerleg
    RF_lowerleg_u = tau(RF_Q3_JOINT) - RF_lowerleg_p(AZ);
    RF_lowerleg_U = RF_lowerleg_AI.col(AZ);
    RF_lowerleg_D = RF_lowerleg_U(AZ);
    
    compute_Ia_revolute(RF_lowerleg_AI, RF_lowerleg_U, RF_lowerleg_D, Ia_r);  // same as: Ia_r = RF_lowerleg_AI - RF_lowerleg_U/RF_lowerleg_D * RF_lowerleg_U.transpose();
    pa = RF_lowerleg_p + Ia_r * RF_lowerleg_c + RF_lowerleg_U * RF_lowerleg_u/RF_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg, IaB);
    RF_upperleg_AI += IaB;
    RF_upperleg_p += (motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg).transpose() * pa;
    
    // + Link LF_lowerleg
    LF_lowerleg_u = tau(LF_Q3_JOINT) - LF_lowerleg_p(AZ);
    LF_lowerleg_U = LF_lowerleg_AI.col(AZ);
    LF_lowerleg_D = LF_lowerleg_U(AZ);
    
    compute_Ia_revolute(LF_lowerleg_AI, LF_lowerleg_U, LF_lowerleg_D, Ia_r);  // same as: Ia_r = LF_lowerleg_AI - LF_lowerleg_U/LF_lowerleg_D * LF_lowerleg_U.transpose();
    pa = LF_lowerleg_p + Ia_r * LF_lowerleg_c + LF_lowerleg_U * LF_lowerleg_u/LF_lowerleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg, IaB);
    LF_upperleg_AI += IaB;
    LF_upperleg_p += (motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg).transpose() * pa;
    
    // + Link RF_upperleg
    RF_upperleg_u = tau(RF_Q2_JOINT) - RF_upperleg_p(AZ);
    RF_upperleg_U = RF_upperleg_AI.col(AZ);
    RF_upperleg_D = RF_upperleg_U(AZ);
    
    compute_Ia_revolute(RF_upperleg_AI, RF_upperleg_U, RF_upperleg_D, Ia_r);  // same as: Ia_r = RF_upperleg_AI - RF_upperleg_U/RF_upperleg_D * RF_upperleg_U.transpose();
    pa = RF_upperleg_p + Ia_r * RF_upperleg_c + RF_upperleg_U * RF_upperleg_u/RF_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_upperleg_X_fr_RF_hipassembly, IaB);
    RF_hipassembly_AI += IaB;
    RF_hipassembly_p += (motionTransforms-> fr_RF_upperleg_X_fr_RF_hipassembly).transpose() * pa;
    
    // + Link LF_upperleg
    LF_upperleg_u = tau(LF_Q2_JOINT) - LF_upperleg_p(AZ);
    LF_upperleg_U = LF_upperleg_AI.col(AZ);
    LF_upperleg_D = LF_upperleg_U(AZ);
    
    compute_Ia_revolute(LF_upperleg_AI, LF_upperleg_U, LF_upperleg_D, Ia_r);  // same as: Ia_r = LF_upperleg_AI - LF_upperleg_U/LF_upperleg_D * LF_upperleg_U.transpose();
    pa = LF_upperleg_p + Ia_r * LF_upperleg_c + LF_upperleg_U * LF_upperleg_u/LF_upperleg_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_upperleg_X_fr_LF_hipassembly, IaB);
    LF_hipassembly_AI += IaB;
    LF_hipassembly_p += (motionTransforms-> fr_LF_upperleg_X_fr_LF_hipassembly).transpose() * pa;
    
    // + Link RF_hipassembly
    RF_hipassembly_u = tau(RF_Q1_JOINT) - RF_hipassembly_p(AZ);
    RF_hipassembly_U = RF_hipassembly_AI.col(AZ);
    RF_hipassembly_D = RF_hipassembly_U(AZ);
    
    compute_Ia_revolute(RF_hipassembly_AI, RF_hipassembly_U, RF_hipassembly_D, Ia_r);  // same as: Ia_r = RF_hipassembly_AI - RF_hipassembly_U/RF_hipassembly_D * RF_hipassembly_U.transpose();
    pa = RF_hipassembly_p + Ia_r * RF_hipassembly_c + RF_hipassembly_U * RF_hipassembly_u/RF_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_hipassembly_X_fr_trunk, IaB);
    trunk_AI += IaB;
    trunk_p += (motionTransforms-> fr_RF_hipassembly_X_fr_trunk).transpose() * pa;
    
    // + Link LF_hipassembly
    LF_hipassembly_u = tau(LF_Q1_JOINT) - LF_hipassembly_p(AZ);
    LF_hipassembly_U = LF_hipassembly_AI.col(AZ);
    LF_hipassembly_D = LF_hipassembly_U(AZ);
    
    compute_Ia_revolute(LF_hipassembly_AI, LF_hipassembly_U, LF_hipassembly_D, Ia_r);  // same as: Ia_r = LF_hipassembly_AI - LF_hipassembly_U/LF_hipassembly_D * LF_hipassembly_U.transpose();
    pa = LF_hipassembly_p + Ia_r * LF_hipassembly_c + LF_hipassembly_U * LF_hipassembly_u/LF_hipassembly_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_hipassembly_X_fr_trunk, IaB);
    trunk_AI += IaB;
    trunk_p += (motionTransforms-> fr_LF_hipassembly_X_fr_trunk).transpose() * pa;
    
    // + The acceleration of the floating base trunk, without gravity
    Eigen::LLT<Matrix66d> llt(trunk_AI);
    trunk_a = - llt.solve(trunk_p);  // trunk_a = - IA^-1 * trunk_p
    
    // ---------------------- THIRD PASS ---------------------- //
    LF_hipassembly_a = (motionTransforms-> fr_LF_hipassembly_X_fr_trunk) * trunk_a + LF_hipassembly_c;
    qdd(LF_Q1_JOINT) = (LF_hipassembly_u - LF_hipassembly_U.dot(LF_hipassembly_a)) / LF_hipassembly_D;
    LF_hipassembly_a(AZ) += qdd(LF_Q1_JOINT);
    
    RF_hipassembly_a = (motionTransforms-> fr_RF_hipassembly_X_fr_trunk) * trunk_a + RF_hipassembly_c;
    qdd(RF_Q1_JOINT) = (RF_hipassembly_u - RF_hipassembly_U.dot(RF_hipassembly_a)) / RF_hipassembly_D;
    RF_hipassembly_a(AZ) += qdd(RF_Q1_JOINT);
    
    LF_upperleg_a = (motionTransforms-> fr_LF_upperleg_X_fr_LF_hipassembly) * LF_hipassembly_a + LF_upperleg_c;
    qdd(LF_Q2_JOINT) = (LF_upperleg_u - LF_upperleg_U.dot(LF_upperleg_a)) / LF_upperleg_D;
    LF_upperleg_a(AZ) += qdd(LF_Q2_JOINT);
    
    RF_upperleg_a = (motionTransforms-> fr_RF_upperleg_X_fr_RF_hipassembly) * RF_hipassembly_a + RF_upperleg_c;
    qdd(RF_Q2_JOINT) = (RF_upperleg_u - RF_upperleg_U.dot(RF_upperleg_a)) / RF_upperleg_D;
    RF_upperleg_a(AZ) += qdd(RF_Q2_JOINT);
    
    LF_lowerleg_a = (motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_a + LF_lowerleg_c;
    qdd(LF_Q3_JOINT) = (LF_lowerleg_u - LF_lowerleg_U.dot(LF_lowerleg_a)) / LF_lowerleg_D;
    LF_lowerleg_a(AZ) += qdd(LF_Q3_JOINT);
    
    RF_lowerleg_a = (motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_a + RF_lowerleg_c;
    qdd(RF_Q3_JOINT) = (RF_lowerleg_u - RF_lowerleg_U.dot(RF_lowerleg_a)) / RF_lowerleg_D;
    RF_lowerleg_a(AZ) += qdd(RF_Q3_JOINT);
    
    LM_hipassembly_a = (motionTransforms-> fr_LM_hipassembly_X_fr_trunk) * trunk_a + LM_hipassembly_c;
    qdd(LM_Q1_JOINT) = (LM_hipassembly_u - LM_hipassembly_U.dot(LM_hipassembly_a)) / LM_hipassembly_D;
    LM_hipassembly_a(AZ) += qdd(LM_Q1_JOINT);
    
    LR_hipassembly_a = (motionTransforms-> fr_LR_hipassembly_X_fr_trunk) * trunk_a + LR_hipassembly_c;
    qdd(LR_Q1_JOINT) = (LR_hipassembly_u - LR_hipassembly_U.dot(LR_hipassembly_a)) / LR_hipassembly_D;
    LR_hipassembly_a(AZ) += qdd(LR_Q1_JOINT);
    
    RM_hipassembly_a = (motionTransforms-> fr_RM_hipassembly_X_fr_trunk) * trunk_a + RM_hipassembly_c;
    qdd(RM_Q1_JOINT) = (RM_hipassembly_u - RM_hipassembly_U.dot(RM_hipassembly_a)) / RM_hipassembly_D;
    RM_hipassembly_a(AZ) += qdd(RM_Q1_JOINT);
    
    RR_hipassembly_a = (motionTransforms-> fr_RR_hipassembly_X_fr_trunk) * trunk_a + RR_hipassembly_c;
    qdd(RR_Q1_JOINT) = (RR_hipassembly_u - RR_hipassembly_U.dot(RR_hipassembly_a)) / RR_hipassembly_D;
    RR_hipassembly_a(AZ) += qdd(RR_Q1_JOINT);
    
    LM_upperleg_a = (motionTransforms-> fr_LM_upperleg_X_fr_LM_hipassembly) * LM_hipassembly_a + LM_upperleg_c;
    qdd(LM_Q2_JOINT) = (LM_upperleg_u - LM_upperleg_U.dot(LM_upperleg_a)) / LM_upperleg_D;
    LM_upperleg_a(AZ) += qdd(LM_Q2_JOINT);
    
    LR_upperleg_a = (motionTransforms-> fr_LR_upperleg_X_fr_LR_hipassembly) * LR_hipassembly_a + LR_upperleg_c;
    qdd(LR_Q2_JOINT) = (LR_upperleg_u - LR_upperleg_U.dot(LR_upperleg_a)) / LR_upperleg_D;
    LR_upperleg_a(AZ) += qdd(LR_Q2_JOINT);
    
    RM_upperleg_a = (motionTransforms-> fr_RM_upperleg_X_fr_RM_hipassembly) * RM_hipassembly_a + RM_upperleg_c;
    qdd(RM_Q2_JOINT) = (RM_upperleg_u - RM_upperleg_U.dot(RM_upperleg_a)) / RM_upperleg_D;
    RM_upperleg_a(AZ) += qdd(RM_Q2_JOINT);
    
    RR_upperleg_a = (motionTransforms-> fr_RR_upperleg_X_fr_RR_hipassembly) * RR_hipassembly_a + RR_upperleg_c;
    qdd(RR_Q2_JOINT) = (RR_upperleg_u - RR_upperleg_U.dot(RR_upperleg_a)) / RR_upperleg_D;
    RR_upperleg_a(AZ) += qdd(RR_Q2_JOINT);
    
    LM_lowerleg_a = (motionTransforms-> fr_LM_lowerleg_X_fr_LM_upperleg) * LM_upperleg_a + LM_lowerleg_c;
    qdd(LM_Q3_JOINT) = (LM_lowerleg_u - LM_lowerleg_U.dot(LM_lowerleg_a)) / LM_lowerleg_D;
    LM_lowerleg_a(AZ) += qdd(LM_Q3_JOINT);
    
    LR_lowerleg_a = (motionTransforms-> fr_LR_lowerleg_X_fr_LR_upperleg) * LR_upperleg_a + LR_lowerleg_c;
    qdd(LR_Q3_JOINT) = (LR_lowerleg_u - LR_lowerleg_U.dot(LR_lowerleg_a)) / LR_lowerleg_D;
    LR_lowerleg_a(AZ) += qdd(LR_Q3_JOINT);
    
    RM_lowerleg_a = (motionTransforms-> fr_RM_lowerleg_X_fr_RM_upperleg) * RM_upperleg_a + RM_lowerleg_c;
    qdd(RM_Q3_JOINT) = (RM_lowerleg_u - RM_lowerleg_U.dot(RM_lowerleg_a)) / RM_lowerleg_D;
    RM_lowerleg_a(AZ) += qdd(RM_Q3_JOINT);
    
    RR_lowerleg_a = (motionTransforms-> fr_RR_lowerleg_X_fr_RR_upperleg) * RR_upperleg_a + RR_lowerleg_c;
    qdd(RR_Q3_JOINT) = (RR_lowerleg_u - RR_lowerleg_U.dot(RR_lowerleg_a)) / RR_lowerleg_D;
    RR_lowerleg_a(AZ) += qdd(RR_Q3_JOINT);
    
    
    // + Add gravity to the acceleration of the floating base
    trunk_a += g;
}

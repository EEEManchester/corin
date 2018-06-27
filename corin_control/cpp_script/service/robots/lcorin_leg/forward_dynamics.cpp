#include "forward_dynamics.h"

#include <Eigen/Cholesky>
#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

// Initialization of static-const data
const iit::LCORIN_LEG::dyn::ForwardDynamics::ExtForces
    iit::LCORIN_LEG::dyn::ForwardDynamics::zeroExtForces(Force::Zero());

iit::LCORIN_LEG::dyn::ForwardDynamics::ForwardDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    l1_v.setZero();
    l1_c.setZero();
    l2_v.setZero();
    l2_c.setZero();
    l3_v.setZero();
    l3_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

void iit::LCORIN_LEG::dyn::ForwardDynamics::fd(
    JointState& qdd,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    l1_AI = inertiaProps->getTensor_l1();
    l1_p = - fext[L1];
    l2_AI = inertiaProps->getTensor_l2();
    l2_p = - fext[L2];
    l3_AI = inertiaProps->getTensor_l3();
    l3_p = - fext[L3];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link l1
    //  - The spatial velocity:
    l1_v(AZ) = qd(Q1);
    
    //  - The bias force term:
    l1_p += vxIv(qd(Q1), l1_AI);
    
    // + Link l2
    //  - The spatial velocity:
    l2_v = (motionTransforms-> fr_l2_X_fr_l1) * l1_v;
    l2_v(AZ) += qd(Q2);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(l2_v, vcross);
    l2_c = vcross.col(AZ) * qd(Q2);
    
    //  - The bias force term:
    l2_p += vxIv(l2_v, l2_AI);
    
    // + Link l3
    //  - The spatial velocity:
    l3_v = (motionTransforms-> fr_l3_X_fr_l2) * l2_v;
    l3_v(AZ) += qd(Q3);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx(l3_v, vcross);
    l3_c = vcross.col(AZ) * qd(Q3);
    
    //  - The bias force term:
    l3_p += vxIv(l3_v, l3_AI);
    
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66d IaB;
    Force pa;
    
    // + Link l3
    l3_u = tau(Q3) - l3_p(AZ);
    l3_U = l3_AI.col(AZ);
    l3_D = l3_U(AZ);
    
    compute_Ia_revolute(l3_AI, l3_U, l3_D, Ia_r);  // same as: Ia_r = l3_AI - l3_U/l3_D * l3_U.transpose();
    pa = l3_p + Ia_r * l3_c + l3_U * l3_u/l3_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_l3_X_fr_l2, IaB);
    l2_AI += IaB;
    l2_p += (motionTransforms-> fr_l3_X_fr_l2).transpose() * pa;
    
    // + Link l2
    l2_u = tau(Q2) - l2_p(AZ);
    l2_U = l2_AI.col(AZ);
    l2_D = l2_U(AZ);
    
    compute_Ia_revolute(l2_AI, l2_U, l2_D, Ia_r);  // same as: Ia_r = l2_AI - l2_U/l2_D * l2_U.transpose();
    pa = l2_p + Ia_r * l2_c + l2_U * l2_u/l2_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_l2_X_fr_l1, IaB);
    l1_AI += IaB;
    l1_p += (motionTransforms-> fr_l2_X_fr_l1).transpose() * pa;
    
    // + Link l1
    l1_u = tau(Q1) - l1_p(AZ);
    l1_U = l1_AI.col(AZ);
    l1_D = l1_U(AZ);
    
    
    
    // ---------------------- THIRD PASS ---------------------- //
    l1_a = (motionTransforms-> fr_l1_X_fr_base).col(LZ) * (iit::rbd::g);
    qdd(Q1) = (l1_u - l1_U.dot(l1_a)) / l1_D;
    l1_a(AZ) += qdd(Q1);
    
    l2_a = (motionTransforms-> fr_l2_X_fr_l1) * l1_a + l2_c;
    qdd(Q2) = (l2_u - l2_U.dot(l2_a)) / l2_D;
    l2_a(AZ) += qdd(Q2);
    
    l3_a = (motionTransforms-> fr_l3_X_fr_l2) * l2_a + l3_c;
    qdd(Q3) = (l3_u - l3_U.dot(l3_a)) / l3_D;
    l3_a(AZ) += qdd(Q3);
    
    
}

#include "forward_dynamics.h"

#include <Eigen/Cholesky>
#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

// Initialization of static-const data
const iit::SLM::dyn::ForwardDynamics::ExtForces
    iit::SLM::dyn::ForwardDynamics::zeroExtForces(Force::Zero());

iit::SLM::dyn::ForwardDynamics::ForwardDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    l1_v.setZero();
    l1_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

void iit::SLM::dyn::ForwardDynamics::fd(
    JointState& qdd,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    l1_AI = inertiaProps->getTensor_l1();
    l1_p = - fext[L1];
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
    
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66d IaB;
    Force pa;
    
    // + Link l1
    l1_u = tau(Q1) - l1_p(AZ);
    l1_U = l1_AI.col(AZ);
    l1_D = l1_U(AZ);
    
    
    
    // ---------------------- THIRD PASS ---------------------- //
    l1_a = (motionTransforms-> fr_l1_X_fr_base).col(LZ) * (iit::rbd::g);
    qdd(Q1) = (l1_u - l1_U.dot(l1_a)) / l1_D;
    l1_a(AZ) += qdd(Q1);
    
    
}

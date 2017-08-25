#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"
#ifndef EIGEN_NO_DEBUG
    #include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace iit::DLM_YP::dyn;

// Initialization of static-const data
const iit::DLM_YP::dyn::InverseDynamics::ExtForces
iit::DLM_YP::dyn::InverseDynamics::zeroExtForces(Force::Zero());

iit::DLM_YP::dyn::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    l1_I(inertiaProps->getTensor_l1() ),
    l2_I(inertiaProps->getTensor_l2() )
    {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot DLM_YP, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    l1_v.setZero();
    l2_v.setZero();

    vcross.setZero();
}

void iit::DLM_YP::dyn::InverseDynamics::id(
    JointState& jForces,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

void iit::DLM_YP::dyn::InverseDynamics::G_terms(JointState& jForces)
{
    // Link 'l1'
    l1_a = (xm->fr_l1_X_fr_base).col(iit::rbd::LZ) * iit::rbd::g;
    l1_f = l1_I * l1_a;
    // Link 'l2'
    l2_a = (xm->fr_l2_X_fr_l1) * l1_a;
    l2_f = l2_I * l2_a;

    secondPass(jForces);
}

void iit::DLM_YP::dyn::InverseDynamics::C_terms(JointState& jForces, const JointState& qd)
{
    // Link 'l1'
    l1_v(iit::rbd::AZ) = qd(Q1);   // l1_v = vJ, for the first link of a fixed base robot
    
    l1_f = vxIv(qd(Q1), l1_I);
    
    // Link 'l2'
    l2_v = ((xm->fr_l2_X_fr_l1) * l1_v);
    l2_v(iit::rbd::AZ) += qd(Q2);
    
    motionCrossProductMx(l2_v, vcross);
    
    l2_a = (vcross.col(iit::rbd::AZ) * qd(Q2));
    
    l2_f = l2_I * l2_a + vxIv(l2_v, l2_I);
    

    secondPass(jForces);
}


void iit::DLM_YP::dyn::InverseDynamics::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    // First pass, link 'l1'
    l1_a = (xm->fr_l1_X_fr_base).col(iit::rbd::LZ) * iit::rbd::g;
    l1_a(iit::rbd::AZ) += qdd(Q1);
    l1_v(iit::rbd::AZ) = qd(Q1);   // l1_v = vJ, for the first link of a fixed base robot
    
    l1_f = l1_I * l1_a + vxIv(qd(Q1), l1_I)  - fext[L1];
    
    // First pass, link 'l2'
    l2_v = ((xm->fr_l2_X_fr_l1) * l1_v);
    l2_v(iit::rbd::AZ) += qd(Q2);
    
    motionCrossProductMx(l2_v, vcross);
    
    l2_a = (xm->fr_l2_X_fr_l1) * l1_a + vcross.col(iit::rbd::AZ) * qd(Q2);
    l2_a(iit::rbd::AZ) += qdd(Q2);
    
    l2_f = l2_I * l2_a + vxIv(l2_v, l2_I) - fext[L2];
    
}

void iit::DLM_YP::dyn::InverseDynamics::secondPass(JointState& jForces)
{
    // Link 'l2'
    jForces(Q2) = l2_f(iit::rbd::AZ);
    l1_f += xm->fr_l2_X_fr_l1.transpose() * l2_f;
    // Link 'l1'
    jForces(Q1) = l1_f(iit::rbd::AZ);
}

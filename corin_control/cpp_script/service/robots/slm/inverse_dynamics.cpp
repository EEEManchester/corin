#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"
#ifndef EIGEN_NO_DEBUG
    #include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace iit::SLM::dyn;

// Initialization of static-const data
const iit::SLM::dyn::InverseDynamics::ExtForces
iit::SLM::dyn::InverseDynamics::zeroExtForces(Force::Zero());

iit::SLM::dyn::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    l1_I(inertiaProps->getTensor_l1() )
    {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot SLM, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    l1_v.setZero();

    vcross.setZero();
}

void iit::SLM::dyn::InverseDynamics::id(
    JointState& jForces,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

void iit::SLM::dyn::InverseDynamics::G_terms(JointState& jForces)
{
    // Link 'l1'
    l1_a = (xm->fr_l1_X_fr_base).col(iit::rbd::LZ) * iit::rbd::g;
    l1_f = l1_I * l1_a;

    secondPass(jForces);
}

void iit::SLM::dyn::InverseDynamics::C_terms(JointState& jForces, const JointState& qd)
{
    // Link 'l1'
    l1_v(iit::rbd::AZ) = qd(Q1);   // l1_v = vJ, for the first link of a fixed base robot
    
    l1_f = vxIv(qd(Q1), l1_I);
    

    secondPass(jForces);
}


void iit::SLM::dyn::InverseDynamics::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    // First pass, link 'l1'
    l1_a = (xm->fr_l1_X_fr_base).col(iit::rbd::LZ) * iit::rbd::g;
    l1_a(iit::rbd::AZ) += qdd(Q1);
    l1_v(iit::rbd::AZ) = qd(Q1);   // l1_v = vJ, for the first link of a fixed base robot
    
    l1_f = l1_I * l1_a + vxIv(qd(Q1), l1_I)  - fext[L1];
    
}

void iit::SLM::dyn::InverseDynamics::secondPass(JointState& jForces)
{
    // Link 'l1'
    jForces(Q1) = l1_f(iit::rbd::AZ);
}

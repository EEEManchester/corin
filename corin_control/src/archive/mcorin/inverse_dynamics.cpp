#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"
#ifndef EIGEN_NO_DEBUG
    #include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace iit::mcorin::dyn;

// Initialization of static-const data
const iit::mcorin::dyn::InverseDynamics::ExtForces
iit::mcorin::dyn::InverseDynamics::zeroExtForces(Force::Zero());

iit::mcorin::dyn::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    LF_hipassembly_I(inertiaProps->getTensor_LF_hipassembly() ),
    LF_upperleg_I(inertiaProps->getTensor_LF_upperleg() ),
    LF_lowerleg_I(inertiaProps->getTensor_LF_lowerleg() ),
    LM_hipassembly_I(inertiaProps->getTensor_LM_hipassembly() ),
    LM_upperleg_I(inertiaProps->getTensor_LM_upperleg() ),
    LM_lowerleg_I(inertiaProps->getTensor_LM_lowerleg() ),
    LR_hipassembly_I(inertiaProps->getTensor_LR_hipassembly() ),
    LR_upperleg_I(inertiaProps->getTensor_LR_upperleg() ),
    LR_lowerleg_I(inertiaProps->getTensor_LR_lowerleg() ),
    RF_hipassembly_I(inertiaProps->getTensor_RF_hipassembly() ),
    RF_upperleg_I(inertiaProps->getTensor_RF_upperleg() ),
    RF_lowerleg_I(inertiaProps->getTensor_RF_lowerleg() ),
    RM_hipassembly_I(inertiaProps->getTensor_RM_hipassembly() ),
    RM_upperleg_I(inertiaProps->getTensor_RM_upperleg() ),
    RM_lowerleg_I(inertiaProps->getTensor_RM_lowerleg() ),
    RR_hipassembly_I(inertiaProps->getTensor_RR_hipassembly() ),
    RR_upperleg_I(inertiaProps->getTensor_RR_upperleg() ),
    RR_lowerleg_I(inertiaProps->getTensor_RR_lowerleg() )
    ,
        trunk_I( inertiaProps->getTensor_trunk() ),
        LF_lowerleg_Ic(LF_lowerleg_I),
        LM_lowerleg_Ic(LM_lowerleg_I),
        LR_lowerleg_Ic(LR_lowerleg_I),
        RF_lowerleg_Ic(RF_lowerleg_I),
        RM_lowerleg_Ic(RM_lowerleg_I),
        RR_lowerleg_Ic(RR_lowerleg_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot mcorin, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    LF_hipassembly_v.setZero();
    LF_upperleg_v.setZero();
    LF_lowerleg_v.setZero();
    LM_hipassembly_v.setZero();
    LM_upperleg_v.setZero();
    LM_lowerleg_v.setZero();
    LR_hipassembly_v.setZero();
    LR_upperleg_v.setZero();
    LR_lowerleg_v.setZero();
    RF_hipassembly_v.setZero();
    RF_upperleg_v.setZero();
    RF_lowerleg_v.setZero();
    RM_hipassembly_v.setZero();
    RM_upperleg_v.setZero();
    RM_lowerleg_v.setZero();
    RR_hipassembly_v.setZero();
    RR_upperleg_v.setZero();
    RR_lowerleg_v.setZero();

    vcross.setZero();
}

void iit::mcorin::dyn::InverseDynamics::id(
    JointState& jForces, Acceleration& trunk_a,
    const Acceleration& g, const Velocity& trunk_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    trunk_Ic = trunk_I;
    LF_hipassembly_Ic = LF_hipassembly_I;
    LF_upperleg_Ic = LF_upperleg_I;
    LM_hipassembly_Ic = LM_hipassembly_I;
    LM_upperleg_Ic = LM_upperleg_I;
    LR_hipassembly_Ic = LR_hipassembly_I;
    LR_upperleg_Ic = LR_upperleg_I;
    RF_hipassembly_Ic = RF_hipassembly_I;
    RF_upperleg_Ic = RF_upperleg_I;
    RM_hipassembly_Ic = RM_hipassembly_I;
    RM_upperleg_Ic = RM_upperleg_I;
    RR_hipassembly_Ic = RR_hipassembly_I;
    RR_upperleg_Ic = RR_upperleg_I;

    // First pass, link 'LF_hipassembly'
    LF_hipassembly_v = ((xm->fr_LF_hipassembly_X_fr_trunk) * trunk_v);
    LF_hipassembly_v(iit::rbd::AZ) += qd(LF_Q1_JOINT);
    
    motionCrossProductMx(LF_hipassembly_v, vcross);
    
    LF_hipassembly_a = (vcross.col(iit::rbd::AZ) * qd(LF_Q1_JOINT));
    LF_hipassembly_a(iit::rbd::AZ) += qdd(LF_Q1_JOINT);
    
    LF_hipassembly_f = LF_hipassembly_I * LF_hipassembly_a + vxIv(LF_hipassembly_v, LF_hipassembly_I);
    
    // First pass, link 'RF_hipassembly'
    RF_hipassembly_v = ((xm->fr_RF_hipassembly_X_fr_trunk) * trunk_v);
    RF_hipassembly_v(iit::rbd::AZ) += qd(RF_Q1_JOINT);
    
    motionCrossProductMx(RF_hipassembly_v, vcross);
    
    RF_hipassembly_a = (vcross.col(iit::rbd::AZ) * qd(RF_Q1_JOINT));
    RF_hipassembly_a(iit::rbd::AZ) += qdd(RF_Q1_JOINT);
    
    RF_hipassembly_f = RF_hipassembly_I * RF_hipassembly_a + vxIv(RF_hipassembly_v, RF_hipassembly_I);
    
    // First pass, link 'LF_upperleg'
    LF_upperleg_v = ((xm->fr_LF_upperleg_X_fr_LF_hipassembly) * LF_hipassembly_v);
    LF_upperleg_v(iit::rbd::AZ) += qd(LF_Q2_JOINT);
    
    motionCrossProductMx(LF_upperleg_v, vcross);
    
    LF_upperleg_a = (xm->fr_LF_upperleg_X_fr_LF_hipassembly) * LF_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(LF_Q2_JOINT);
    LF_upperleg_a(iit::rbd::AZ) += qdd(LF_Q2_JOINT);
    
    LF_upperleg_f = LF_upperleg_I * LF_upperleg_a + vxIv(LF_upperleg_v, LF_upperleg_I);
    
    // First pass, link 'RF_upperleg'
    RF_upperleg_v = ((xm->fr_RF_upperleg_X_fr_RF_hipassembly) * RF_hipassembly_v);
    RF_upperleg_v(iit::rbd::AZ) += qd(RF_Q2_JOINT);
    
    motionCrossProductMx(RF_upperleg_v, vcross);
    
    RF_upperleg_a = (xm->fr_RF_upperleg_X_fr_RF_hipassembly) * RF_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(RF_Q2_JOINT);
    RF_upperleg_a(iit::rbd::AZ) += qdd(RF_Q2_JOINT);
    
    RF_upperleg_f = RF_upperleg_I * RF_upperleg_a + vxIv(RF_upperleg_v, RF_upperleg_I);
    
    // First pass, link 'LF_lowerleg'
    LF_lowerleg_v = ((xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_v);
    LF_lowerleg_v(iit::rbd::AZ) += qd(LF_Q3_JOINT);
    
    motionCrossProductMx(LF_lowerleg_v, vcross);
    
    LF_lowerleg_a = (xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LF_Q3_JOINT);
    LF_lowerleg_a(iit::rbd::AZ) += qdd(LF_Q3_JOINT);
    
    LF_lowerleg_f = LF_lowerleg_I * LF_lowerleg_a + vxIv(LF_lowerleg_v, LF_lowerleg_I);
    
    // First pass, link 'RF_lowerleg'
    RF_lowerleg_v = ((xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_v);
    RF_lowerleg_v(iit::rbd::AZ) += qd(RF_Q3_JOINT);
    
    motionCrossProductMx(RF_lowerleg_v, vcross);
    
    RF_lowerleg_a = (xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RF_Q3_JOINT);
    RF_lowerleg_a(iit::rbd::AZ) += qdd(RF_Q3_JOINT);
    
    RF_lowerleg_f = RF_lowerleg_I * RF_lowerleg_a + vxIv(RF_lowerleg_v, RF_lowerleg_I);
    
    // First pass, link 'LM_hipassembly'
    LM_hipassembly_v = ((xm->fr_LM_hipassembly_X_fr_trunk) * trunk_v);
    LM_hipassembly_v(iit::rbd::AZ) += qd(LM_Q1_JOINT);
    
    motionCrossProductMx(LM_hipassembly_v, vcross);
    
    LM_hipassembly_a = (vcross.col(iit::rbd::AZ) * qd(LM_Q1_JOINT));
    LM_hipassembly_a(iit::rbd::AZ) += qdd(LM_Q1_JOINT);
    
    LM_hipassembly_f = LM_hipassembly_I * LM_hipassembly_a + vxIv(LM_hipassembly_v, LM_hipassembly_I);
    
    // First pass, link 'LR_hipassembly'
    LR_hipassembly_v = ((xm->fr_LR_hipassembly_X_fr_trunk) * trunk_v);
    LR_hipassembly_v(iit::rbd::AZ) += qd(LR_Q1_JOINT);
    
    motionCrossProductMx(LR_hipassembly_v, vcross);
    
    LR_hipassembly_a = (vcross.col(iit::rbd::AZ) * qd(LR_Q1_JOINT));
    LR_hipassembly_a(iit::rbd::AZ) += qdd(LR_Q1_JOINT);
    
    LR_hipassembly_f = LR_hipassembly_I * LR_hipassembly_a + vxIv(LR_hipassembly_v, LR_hipassembly_I);
    
    // First pass, link 'RM_hipassembly'
    RM_hipassembly_v = ((xm->fr_RM_hipassembly_X_fr_trunk) * trunk_v);
    RM_hipassembly_v(iit::rbd::AZ) += qd(RM_Q1_JOINT);
    
    motionCrossProductMx(RM_hipassembly_v, vcross);
    
    RM_hipassembly_a = (vcross.col(iit::rbd::AZ) * qd(RM_Q1_JOINT));
    RM_hipassembly_a(iit::rbd::AZ) += qdd(RM_Q1_JOINT);
    
    RM_hipassembly_f = RM_hipassembly_I * RM_hipassembly_a + vxIv(RM_hipassembly_v, RM_hipassembly_I);
    
    // First pass, link 'RR_hipassembly'
    RR_hipassembly_v = ((xm->fr_RR_hipassembly_X_fr_trunk) * trunk_v);
    RR_hipassembly_v(iit::rbd::AZ) += qd(RR_Q1_JOINT);
    
    motionCrossProductMx(RR_hipassembly_v, vcross);
    
    RR_hipassembly_a = (vcross.col(iit::rbd::AZ) * qd(RR_Q1_JOINT));
    RR_hipassembly_a(iit::rbd::AZ) += qdd(RR_Q1_JOINT);
    
    RR_hipassembly_f = RR_hipassembly_I * RR_hipassembly_a + vxIv(RR_hipassembly_v, RR_hipassembly_I);
    
    // First pass, link 'LM_upperleg'
    LM_upperleg_v = ((xm->fr_LM_upperleg_X_fr_LM_hipassembly) * LM_hipassembly_v);
    LM_upperleg_v(iit::rbd::AZ) += qd(LM_Q2_JOINT);
    
    motionCrossProductMx(LM_upperleg_v, vcross);
    
    LM_upperleg_a = (xm->fr_LM_upperleg_X_fr_LM_hipassembly) * LM_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(LM_Q2_JOINT);
    LM_upperleg_a(iit::rbd::AZ) += qdd(LM_Q2_JOINT);
    
    LM_upperleg_f = LM_upperleg_I * LM_upperleg_a + vxIv(LM_upperleg_v, LM_upperleg_I);
    
    // First pass, link 'LR_upperleg'
    LR_upperleg_v = ((xm->fr_LR_upperleg_X_fr_LR_hipassembly) * LR_hipassembly_v);
    LR_upperleg_v(iit::rbd::AZ) += qd(LR_Q2_JOINT);
    
    motionCrossProductMx(LR_upperleg_v, vcross);
    
    LR_upperleg_a = (xm->fr_LR_upperleg_X_fr_LR_hipassembly) * LR_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(LR_Q2_JOINT);
    LR_upperleg_a(iit::rbd::AZ) += qdd(LR_Q2_JOINT);
    
    LR_upperleg_f = LR_upperleg_I * LR_upperleg_a + vxIv(LR_upperleg_v, LR_upperleg_I);
    
    // First pass, link 'RM_upperleg'
    RM_upperleg_v = ((xm->fr_RM_upperleg_X_fr_RM_hipassembly) * RM_hipassembly_v);
    RM_upperleg_v(iit::rbd::AZ) += qd(RM_Q2_JOINT);
    
    motionCrossProductMx(RM_upperleg_v, vcross);
    
    RM_upperleg_a = (xm->fr_RM_upperleg_X_fr_RM_hipassembly) * RM_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(RM_Q2_JOINT);
    RM_upperleg_a(iit::rbd::AZ) += qdd(RM_Q2_JOINT);
    
    RM_upperleg_f = RM_upperleg_I * RM_upperleg_a + vxIv(RM_upperleg_v, RM_upperleg_I);
    
    // First pass, link 'RR_upperleg'
    RR_upperleg_v = ((xm->fr_RR_upperleg_X_fr_RR_hipassembly) * RR_hipassembly_v);
    RR_upperleg_v(iit::rbd::AZ) += qd(RR_Q2_JOINT);
    
    motionCrossProductMx(RR_upperleg_v, vcross);
    
    RR_upperleg_a = (xm->fr_RR_upperleg_X_fr_RR_hipassembly) * RR_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(RR_Q2_JOINT);
    RR_upperleg_a(iit::rbd::AZ) += qdd(RR_Q2_JOINT);
    
    RR_upperleg_f = RR_upperleg_I * RR_upperleg_a + vxIv(RR_upperleg_v, RR_upperleg_I);
    
    // First pass, link 'LM_lowerleg'
    LM_lowerleg_v = ((xm->fr_LM_lowerleg_X_fr_LM_upperleg) * LM_upperleg_v);
    LM_lowerleg_v(iit::rbd::AZ) += qd(LM_Q3_JOINT);
    
    motionCrossProductMx(LM_lowerleg_v, vcross);
    
    LM_lowerleg_a = (xm->fr_LM_lowerleg_X_fr_LM_upperleg) * LM_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LM_Q3_JOINT);
    LM_lowerleg_a(iit::rbd::AZ) += qdd(LM_Q3_JOINT);
    
    LM_lowerleg_f = LM_lowerleg_I * LM_lowerleg_a + vxIv(LM_lowerleg_v, LM_lowerleg_I);
    
    // First pass, link 'LR_lowerleg'
    LR_lowerleg_v = ((xm->fr_LR_lowerleg_X_fr_LR_upperleg) * LR_upperleg_v);
    LR_lowerleg_v(iit::rbd::AZ) += qd(LR_Q3_JOINT);
    
    motionCrossProductMx(LR_lowerleg_v, vcross);
    
    LR_lowerleg_a = (xm->fr_LR_lowerleg_X_fr_LR_upperleg) * LR_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LR_Q3_JOINT);
    LR_lowerleg_a(iit::rbd::AZ) += qdd(LR_Q3_JOINT);
    
    LR_lowerleg_f = LR_lowerleg_I * LR_lowerleg_a + vxIv(LR_lowerleg_v, LR_lowerleg_I);
    
    // First pass, link 'RM_lowerleg'
    RM_lowerleg_v = ((xm->fr_RM_lowerleg_X_fr_RM_upperleg) * RM_upperleg_v);
    RM_lowerleg_v(iit::rbd::AZ) += qd(RM_Q3_JOINT);
    
    motionCrossProductMx(RM_lowerleg_v, vcross);
    
    RM_lowerleg_a = (xm->fr_RM_lowerleg_X_fr_RM_upperleg) * RM_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RM_Q3_JOINT);
    RM_lowerleg_a(iit::rbd::AZ) += qdd(RM_Q3_JOINT);
    
    RM_lowerleg_f = RM_lowerleg_I * RM_lowerleg_a + vxIv(RM_lowerleg_v, RM_lowerleg_I);
    
    // First pass, link 'RR_lowerleg'
    RR_lowerleg_v = ((xm->fr_RR_lowerleg_X_fr_RR_upperleg) * RR_upperleg_v);
    RR_lowerleg_v(iit::rbd::AZ) += qd(RR_Q3_JOINT);
    
    motionCrossProductMx(RR_lowerleg_v, vcross);
    
    RR_lowerleg_a = (xm->fr_RR_lowerleg_X_fr_RR_upperleg) * RR_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RR_Q3_JOINT);
    RR_lowerleg_a(iit::rbd::AZ) += qdd(RR_Q3_JOINT);
    
    RR_lowerleg_f = RR_lowerleg_I * RR_lowerleg_a + vxIv(RR_lowerleg_v, RR_lowerleg_I);
    
    // The force exerted on the floating base by the links
    trunk_f = vxIv(trunk_v, trunk_I);
    

    // Add the external forces:
    trunk_f -= fext[TRUNK];
    LF_hipassembly_f -= fext[LF_HIPASSEMBLY];
    RF_hipassembly_f -= fext[RF_HIPASSEMBLY];
    LF_upperleg_f -= fext[LF_UPPERLEG];
    RF_upperleg_f -= fext[RF_UPPERLEG];
    LF_lowerleg_f -= fext[LF_LOWERLEG];
    RF_lowerleg_f -= fext[RF_LOWERLEG];
    LM_hipassembly_f -= fext[LM_HIPASSEMBLY];
    LR_hipassembly_f -= fext[LR_HIPASSEMBLY];
    RM_hipassembly_f -= fext[RM_HIPASSEMBLY];
    RR_hipassembly_f -= fext[RR_HIPASSEMBLY];
    LM_upperleg_f -= fext[LM_UPPERLEG];
    LR_upperleg_f -= fext[LR_UPPERLEG];
    RM_upperleg_f -= fext[RM_UPPERLEG];
    RR_upperleg_f -= fext[RR_UPPERLEG];
    LM_lowerleg_f -= fext[LM_LOWERLEG];
    LR_lowerleg_f -= fext[LR_LOWERLEG];
    RM_lowerleg_f -= fext[RM_LOWERLEG];
    RR_lowerleg_f -= fext[RR_LOWERLEG];

    RR_upperleg_Ic = RR_upperleg_Ic + (xm->fr_RR_lowerleg_X_fr_RR_upperleg).transpose() * RR_lowerleg_Ic * (xm->fr_RR_lowerleg_X_fr_RR_upperleg);
    RR_upperleg_f = RR_upperleg_f + (xm->fr_RR_lowerleg_X_fr_RR_upperleg).transpose() * RR_lowerleg_f;
    
    RM_upperleg_Ic = RM_upperleg_Ic + (xm->fr_RM_lowerleg_X_fr_RM_upperleg).transpose() * RM_lowerleg_Ic * (xm->fr_RM_lowerleg_X_fr_RM_upperleg);
    RM_upperleg_f = RM_upperleg_f + (xm->fr_RM_lowerleg_X_fr_RM_upperleg).transpose() * RM_lowerleg_f;
    
    LR_upperleg_Ic = LR_upperleg_Ic + (xm->fr_LR_lowerleg_X_fr_LR_upperleg).transpose() * LR_lowerleg_Ic * (xm->fr_LR_lowerleg_X_fr_LR_upperleg);
    LR_upperleg_f = LR_upperleg_f + (xm->fr_LR_lowerleg_X_fr_LR_upperleg).transpose() * LR_lowerleg_f;
    
    LM_upperleg_Ic = LM_upperleg_Ic + (xm->fr_LM_lowerleg_X_fr_LM_upperleg).transpose() * LM_lowerleg_Ic * (xm->fr_LM_lowerleg_X_fr_LM_upperleg);
    LM_upperleg_f = LM_upperleg_f + (xm->fr_LM_lowerleg_X_fr_LM_upperleg).transpose() * LM_lowerleg_f;
    
    RR_hipassembly_Ic = RR_hipassembly_Ic + (xm->fr_RR_upperleg_X_fr_RR_hipassembly).transpose() * RR_upperleg_Ic * (xm->fr_RR_upperleg_X_fr_RR_hipassembly);
    RR_hipassembly_f = RR_hipassembly_f + (xm->fr_RR_upperleg_X_fr_RR_hipassembly).transpose() * RR_upperleg_f;
    
    RM_hipassembly_Ic = RM_hipassembly_Ic + (xm->fr_RM_upperleg_X_fr_RM_hipassembly).transpose() * RM_upperleg_Ic * (xm->fr_RM_upperleg_X_fr_RM_hipassembly);
    RM_hipassembly_f = RM_hipassembly_f + (xm->fr_RM_upperleg_X_fr_RM_hipassembly).transpose() * RM_upperleg_f;
    
    LR_hipassembly_Ic = LR_hipassembly_Ic + (xm->fr_LR_upperleg_X_fr_LR_hipassembly).transpose() * LR_upperleg_Ic * (xm->fr_LR_upperleg_X_fr_LR_hipassembly);
    LR_hipassembly_f = LR_hipassembly_f + (xm->fr_LR_upperleg_X_fr_LR_hipassembly).transpose() * LR_upperleg_f;
    
    LM_hipassembly_Ic = LM_hipassembly_Ic + (xm->fr_LM_upperleg_X_fr_LM_hipassembly).transpose() * LM_upperleg_Ic * (xm->fr_LM_upperleg_X_fr_LM_hipassembly);
    LM_hipassembly_f = LM_hipassembly_f + (xm->fr_LM_upperleg_X_fr_LM_hipassembly).transpose() * LM_upperleg_f;
    
    trunk_Ic = trunk_Ic + (xm->fr_RR_hipassembly_X_fr_trunk).transpose() * RR_hipassembly_Ic * (xm->fr_RR_hipassembly_X_fr_trunk);
    trunk_f = trunk_f + (xm->fr_RR_hipassembly_X_fr_trunk).transpose() * RR_hipassembly_f;
    
    trunk_Ic = trunk_Ic + (xm->fr_RM_hipassembly_X_fr_trunk).transpose() * RM_hipassembly_Ic * (xm->fr_RM_hipassembly_X_fr_trunk);
    trunk_f = trunk_f + (xm->fr_RM_hipassembly_X_fr_trunk).transpose() * RM_hipassembly_f;
    
    trunk_Ic = trunk_Ic + (xm->fr_LR_hipassembly_X_fr_trunk).transpose() * LR_hipassembly_Ic * (xm->fr_LR_hipassembly_X_fr_trunk);
    trunk_f = trunk_f + (xm->fr_LR_hipassembly_X_fr_trunk).transpose() * LR_hipassembly_f;
    
    trunk_Ic = trunk_Ic + (xm->fr_LM_hipassembly_X_fr_trunk).transpose() * LM_hipassembly_Ic * (xm->fr_LM_hipassembly_X_fr_trunk);
    trunk_f = trunk_f + (xm->fr_LM_hipassembly_X_fr_trunk).transpose() * LM_hipassembly_f;
    
    RF_upperleg_Ic = RF_upperleg_Ic + (xm->fr_RF_lowerleg_X_fr_RF_upperleg).transpose() * RF_lowerleg_Ic * (xm->fr_RF_lowerleg_X_fr_RF_upperleg);
    RF_upperleg_f = RF_upperleg_f + (xm->fr_RF_lowerleg_X_fr_RF_upperleg).transpose() * RF_lowerleg_f;
    
    LF_upperleg_Ic = LF_upperleg_Ic + (xm->fr_LF_lowerleg_X_fr_LF_upperleg).transpose() * LF_lowerleg_Ic * (xm->fr_LF_lowerleg_X_fr_LF_upperleg);
    LF_upperleg_f = LF_upperleg_f + (xm->fr_LF_lowerleg_X_fr_LF_upperleg).transpose() * LF_lowerleg_f;
    
    RF_hipassembly_Ic = RF_hipassembly_Ic + (xm->fr_RF_upperleg_X_fr_RF_hipassembly).transpose() * RF_upperleg_Ic * (xm->fr_RF_upperleg_X_fr_RF_hipassembly);
    RF_hipassembly_f = RF_hipassembly_f + (xm->fr_RF_upperleg_X_fr_RF_hipassembly).transpose() * RF_upperleg_f;
    
    LF_hipassembly_Ic = LF_hipassembly_Ic + (xm->fr_LF_upperleg_X_fr_LF_hipassembly).transpose() * LF_upperleg_Ic * (xm->fr_LF_upperleg_X_fr_LF_hipassembly);
    LF_hipassembly_f = LF_hipassembly_f + (xm->fr_LF_upperleg_X_fr_LF_hipassembly).transpose() * LF_upperleg_f;
    
    trunk_Ic = trunk_Ic + (xm->fr_RF_hipassembly_X_fr_trunk).transpose() * RF_hipassembly_Ic * (xm->fr_RF_hipassembly_X_fr_trunk);
    trunk_f = trunk_f + (xm->fr_RF_hipassembly_X_fr_trunk).transpose() * RF_hipassembly_f;
    
    trunk_Ic = trunk_Ic + (xm->fr_LF_hipassembly_X_fr_trunk).transpose() * LF_hipassembly_Ic * (xm->fr_LF_hipassembly_X_fr_trunk);
    trunk_f = trunk_f + (xm->fr_LF_hipassembly_X_fr_trunk).transpose() * LF_hipassembly_f;
    

    // The base acceleration due to the force due to the movement of the links
    trunk_a = - trunk_Ic.inverse() * trunk_f;
    
    LF_hipassembly_a = xm->fr_LF_hipassembly_X_fr_trunk * trunk_a;
    jForces(LF_Q1_JOINT) = (LF_hipassembly_Ic.row(iit::rbd::AZ) * LF_hipassembly_a + LF_hipassembly_f(iit::rbd::AZ));
    
    RF_hipassembly_a = xm->fr_RF_hipassembly_X_fr_trunk * trunk_a;
    jForces(RF_Q1_JOINT) = (RF_hipassembly_Ic.row(iit::rbd::AZ) * RF_hipassembly_a + RF_hipassembly_f(iit::rbd::AZ));
    
    LF_upperleg_a = xm->fr_LF_upperleg_X_fr_LF_hipassembly * LF_hipassembly_a;
    jForces(LF_Q2_JOINT) = (LF_upperleg_Ic.row(iit::rbd::AZ) * LF_upperleg_a + LF_upperleg_f(iit::rbd::AZ));
    
    RF_upperleg_a = xm->fr_RF_upperleg_X_fr_RF_hipassembly * RF_hipassembly_a;
    jForces(RF_Q2_JOINT) = (RF_upperleg_Ic.row(iit::rbd::AZ) * RF_upperleg_a + RF_upperleg_f(iit::rbd::AZ));
    
    LF_lowerleg_a = xm->fr_LF_lowerleg_X_fr_LF_upperleg * LF_upperleg_a;
    jForces(LF_Q3_JOINT) = (LF_lowerleg_Ic.row(iit::rbd::AZ) * LF_lowerleg_a + LF_lowerleg_f(iit::rbd::AZ));
    
    RF_lowerleg_a = xm->fr_RF_lowerleg_X_fr_RF_upperleg * RF_upperleg_a;
    jForces(RF_Q3_JOINT) = (RF_lowerleg_Ic.row(iit::rbd::AZ) * RF_lowerleg_a + RF_lowerleg_f(iit::rbd::AZ));
    
    LM_hipassembly_a = xm->fr_LM_hipassembly_X_fr_trunk * trunk_a;
    jForces(LM_Q1_JOINT) = (LM_hipassembly_Ic.row(iit::rbd::AZ) * LM_hipassembly_a + LM_hipassembly_f(iit::rbd::AZ));
    
    LR_hipassembly_a = xm->fr_LR_hipassembly_X_fr_trunk * trunk_a;
    jForces(LR_Q1_JOINT) = (LR_hipassembly_Ic.row(iit::rbd::AZ) * LR_hipassembly_a + LR_hipassembly_f(iit::rbd::AZ));
    
    RM_hipassembly_a = xm->fr_RM_hipassembly_X_fr_trunk * trunk_a;
    jForces(RM_Q1_JOINT) = (RM_hipassembly_Ic.row(iit::rbd::AZ) * RM_hipassembly_a + RM_hipassembly_f(iit::rbd::AZ));
    
    RR_hipassembly_a = xm->fr_RR_hipassembly_X_fr_trunk * trunk_a;
    jForces(RR_Q1_JOINT) = (RR_hipassembly_Ic.row(iit::rbd::AZ) * RR_hipassembly_a + RR_hipassembly_f(iit::rbd::AZ));
    
    LM_upperleg_a = xm->fr_LM_upperleg_X_fr_LM_hipassembly * LM_hipassembly_a;
    jForces(LM_Q2_JOINT) = (LM_upperleg_Ic.row(iit::rbd::AZ) * LM_upperleg_a + LM_upperleg_f(iit::rbd::AZ));
    
    LR_upperleg_a = xm->fr_LR_upperleg_X_fr_LR_hipassembly * LR_hipassembly_a;
    jForces(LR_Q2_JOINT) = (LR_upperleg_Ic.row(iit::rbd::AZ) * LR_upperleg_a + LR_upperleg_f(iit::rbd::AZ));
    
    RM_upperleg_a = xm->fr_RM_upperleg_X_fr_RM_hipassembly * RM_hipassembly_a;
    jForces(RM_Q2_JOINT) = (RM_upperleg_Ic.row(iit::rbd::AZ) * RM_upperleg_a + RM_upperleg_f(iit::rbd::AZ));
    
    RR_upperleg_a = xm->fr_RR_upperleg_X_fr_RR_hipassembly * RR_hipassembly_a;
    jForces(RR_Q2_JOINT) = (RR_upperleg_Ic.row(iit::rbd::AZ) * RR_upperleg_a + RR_upperleg_f(iit::rbd::AZ));
    
    LM_lowerleg_a = xm->fr_LM_lowerleg_X_fr_LM_upperleg * LM_upperleg_a;
    jForces(LM_Q3_JOINT) = (LM_lowerleg_Ic.row(iit::rbd::AZ) * LM_lowerleg_a + LM_lowerleg_f(iit::rbd::AZ));
    
    LR_lowerleg_a = xm->fr_LR_lowerleg_X_fr_LR_upperleg * LR_upperleg_a;
    jForces(LR_Q3_JOINT) = (LR_lowerleg_Ic.row(iit::rbd::AZ) * LR_lowerleg_a + LR_lowerleg_f(iit::rbd::AZ));
    
    RM_lowerleg_a = xm->fr_RM_lowerleg_X_fr_RM_upperleg * RM_upperleg_a;
    jForces(RM_Q3_JOINT) = (RM_lowerleg_Ic.row(iit::rbd::AZ) * RM_lowerleg_a + RM_lowerleg_f(iit::rbd::AZ));
    
    RR_lowerleg_a = xm->fr_RR_lowerleg_X_fr_RR_upperleg * RR_upperleg_a;
    jForces(RR_Q3_JOINT) = (RR_lowerleg_Ic.row(iit::rbd::AZ) * RR_lowerleg_a + RR_lowerleg_f(iit::rbd::AZ));
    

    trunk_a += g;
}


void iit::mcorin::dyn::InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g)
{
    const Acceleration& trunk_a = -g;

    // Link 'LF_hipassembly'
    LF_hipassembly_a = (xm->fr_LF_hipassembly_X_fr_trunk) * trunk_a;
    LF_hipassembly_f = LF_hipassembly_I * LF_hipassembly_a;
    // Link 'RF_hipassembly'
    RF_hipassembly_a = (xm->fr_RF_hipassembly_X_fr_trunk) * trunk_a;
    RF_hipassembly_f = RF_hipassembly_I * RF_hipassembly_a;
    // Link 'LF_upperleg'
    LF_upperleg_a = (xm->fr_LF_upperleg_X_fr_LF_hipassembly) * LF_hipassembly_a;
    LF_upperleg_f = LF_upperleg_I * LF_upperleg_a;
    // Link 'RF_upperleg'
    RF_upperleg_a = (xm->fr_RF_upperleg_X_fr_RF_hipassembly) * RF_hipassembly_a;
    RF_upperleg_f = RF_upperleg_I * RF_upperleg_a;
    // Link 'LF_lowerleg'
    LF_lowerleg_a = (xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_a;
    LF_lowerleg_f = LF_lowerleg_I * LF_lowerleg_a;
    // Link 'RF_lowerleg'
    RF_lowerleg_a = (xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_a;
    RF_lowerleg_f = RF_lowerleg_I * RF_lowerleg_a;
    // Link 'LM_hipassembly'
    LM_hipassembly_a = (xm->fr_LM_hipassembly_X_fr_trunk) * trunk_a;
    LM_hipassembly_f = LM_hipassembly_I * LM_hipassembly_a;
    // Link 'LR_hipassembly'
    LR_hipassembly_a = (xm->fr_LR_hipassembly_X_fr_trunk) * trunk_a;
    LR_hipassembly_f = LR_hipassembly_I * LR_hipassembly_a;
    // Link 'RM_hipassembly'
    RM_hipassembly_a = (xm->fr_RM_hipassembly_X_fr_trunk) * trunk_a;
    RM_hipassembly_f = RM_hipassembly_I * RM_hipassembly_a;
    // Link 'RR_hipassembly'
    RR_hipassembly_a = (xm->fr_RR_hipassembly_X_fr_trunk) * trunk_a;
    RR_hipassembly_f = RR_hipassembly_I * RR_hipassembly_a;
    // Link 'LM_upperleg'
    LM_upperleg_a = (xm->fr_LM_upperleg_X_fr_LM_hipassembly) * LM_hipassembly_a;
    LM_upperleg_f = LM_upperleg_I * LM_upperleg_a;
    // Link 'LR_upperleg'
    LR_upperleg_a = (xm->fr_LR_upperleg_X_fr_LR_hipassembly) * LR_hipassembly_a;
    LR_upperleg_f = LR_upperleg_I * LR_upperleg_a;
    // Link 'RM_upperleg'
    RM_upperleg_a = (xm->fr_RM_upperleg_X_fr_RM_hipassembly) * RM_hipassembly_a;
    RM_upperleg_f = RM_upperleg_I * RM_upperleg_a;
    // Link 'RR_upperleg'
    RR_upperleg_a = (xm->fr_RR_upperleg_X_fr_RR_hipassembly) * RR_hipassembly_a;
    RR_upperleg_f = RR_upperleg_I * RR_upperleg_a;
    // Link 'LM_lowerleg'
    LM_lowerleg_a = (xm->fr_LM_lowerleg_X_fr_LM_upperleg) * LM_upperleg_a;
    LM_lowerleg_f = LM_lowerleg_I * LM_lowerleg_a;
    // Link 'LR_lowerleg'
    LR_lowerleg_a = (xm->fr_LR_lowerleg_X_fr_LR_upperleg) * LR_upperleg_a;
    LR_lowerleg_f = LR_lowerleg_I * LR_lowerleg_a;
    // Link 'RM_lowerleg'
    RM_lowerleg_a = (xm->fr_RM_lowerleg_X_fr_RM_upperleg) * RM_upperleg_a;
    RM_lowerleg_f = RM_lowerleg_I * RM_lowerleg_a;
    // Link 'RR_lowerleg'
    RR_lowerleg_a = (xm->fr_RR_lowerleg_X_fr_RR_upperleg) * RR_upperleg_a;
    RR_lowerleg_f = RR_lowerleg_I * RR_lowerleg_a;

    trunk_f = trunk_I * trunk_a;

    secondPass_fullyActuated(jForces);

    baseWrench = trunk_f;
}

void iit::mcorin::dyn::InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& trunk_v, const JointState& qd)
{
    // Link 'LF_hipassembly'
    LF_hipassembly_v = ((xm->fr_LF_hipassembly_X_fr_trunk) * trunk_v);
    LF_hipassembly_v(iit::rbd::AZ) += qd(LF_Q1_JOINT);
    motionCrossProductMx(LF_hipassembly_v, vcross);
    LF_hipassembly_a = (vcross.col(iit::rbd::AZ) * qd(LF_Q1_JOINT));
    LF_hipassembly_f = LF_hipassembly_I * LF_hipassembly_a + vxIv(LF_hipassembly_v, LF_hipassembly_I);
    
    // Link 'RF_hipassembly'
    RF_hipassembly_v = ((xm->fr_RF_hipassembly_X_fr_trunk) * trunk_v);
    RF_hipassembly_v(iit::rbd::AZ) += qd(RF_Q1_JOINT);
    motionCrossProductMx(RF_hipassembly_v, vcross);
    RF_hipassembly_a = (vcross.col(iit::rbd::AZ) * qd(RF_Q1_JOINT));
    RF_hipassembly_f = RF_hipassembly_I * RF_hipassembly_a + vxIv(RF_hipassembly_v, RF_hipassembly_I);
    
    // Link 'LF_upperleg'
    LF_upperleg_v = ((xm->fr_LF_upperleg_X_fr_LF_hipassembly) * LF_hipassembly_v);
    LF_upperleg_v(iit::rbd::AZ) += qd(LF_Q2_JOINT);
    motionCrossProductMx(LF_upperleg_v, vcross);
    LF_upperleg_a = (xm->fr_LF_upperleg_X_fr_LF_hipassembly) * LF_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(LF_Q2_JOINT);
    LF_upperleg_f = LF_upperleg_I * LF_upperleg_a + vxIv(LF_upperleg_v, LF_upperleg_I);
    
    // Link 'RF_upperleg'
    RF_upperleg_v = ((xm->fr_RF_upperleg_X_fr_RF_hipassembly) * RF_hipassembly_v);
    RF_upperleg_v(iit::rbd::AZ) += qd(RF_Q2_JOINT);
    motionCrossProductMx(RF_upperleg_v, vcross);
    RF_upperleg_a = (xm->fr_RF_upperleg_X_fr_RF_hipassembly) * RF_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(RF_Q2_JOINT);
    RF_upperleg_f = RF_upperleg_I * RF_upperleg_a + vxIv(RF_upperleg_v, RF_upperleg_I);
    
    // Link 'LF_lowerleg'
    LF_lowerleg_v = ((xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_v);
    LF_lowerleg_v(iit::rbd::AZ) += qd(LF_Q3_JOINT);
    motionCrossProductMx(LF_lowerleg_v, vcross);
    LF_lowerleg_a = (xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LF_Q3_JOINT);
    LF_lowerleg_f = LF_lowerleg_I * LF_lowerleg_a + vxIv(LF_lowerleg_v, LF_lowerleg_I);
    
    // Link 'RF_lowerleg'
    RF_lowerleg_v = ((xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_v);
    RF_lowerleg_v(iit::rbd::AZ) += qd(RF_Q3_JOINT);
    motionCrossProductMx(RF_lowerleg_v, vcross);
    RF_lowerleg_a = (xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RF_Q3_JOINT);
    RF_lowerleg_f = RF_lowerleg_I * RF_lowerleg_a + vxIv(RF_lowerleg_v, RF_lowerleg_I);
    
    // Link 'LM_hipassembly'
    LM_hipassembly_v = ((xm->fr_LM_hipassembly_X_fr_trunk) * trunk_v);
    LM_hipassembly_v(iit::rbd::AZ) += qd(LM_Q1_JOINT);
    motionCrossProductMx(LM_hipassembly_v, vcross);
    LM_hipassembly_a = (vcross.col(iit::rbd::AZ) * qd(LM_Q1_JOINT));
    LM_hipassembly_f = LM_hipassembly_I * LM_hipassembly_a + vxIv(LM_hipassembly_v, LM_hipassembly_I);
    
    // Link 'LR_hipassembly'
    LR_hipassembly_v = ((xm->fr_LR_hipassembly_X_fr_trunk) * trunk_v);
    LR_hipassembly_v(iit::rbd::AZ) += qd(LR_Q1_JOINT);
    motionCrossProductMx(LR_hipassembly_v, vcross);
    LR_hipassembly_a = (vcross.col(iit::rbd::AZ) * qd(LR_Q1_JOINT));
    LR_hipassembly_f = LR_hipassembly_I * LR_hipassembly_a + vxIv(LR_hipassembly_v, LR_hipassembly_I);
    
    // Link 'RM_hipassembly'
    RM_hipassembly_v = ((xm->fr_RM_hipassembly_X_fr_trunk) * trunk_v);
    RM_hipassembly_v(iit::rbd::AZ) += qd(RM_Q1_JOINT);
    motionCrossProductMx(RM_hipassembly_v, vcross);
    RM_hipassembly_a = (vcross.col(iit::rbd::AZ) * qd(RM_Q1_JOINT));
    RM_hipassembly_f = RM_hipassembly_I * RM_hipassembly_a + vxIv(RM_hipassembly_v, RM_hipassembly_I);
    
    // Link 'RR_hipassembly'
    RR_hipassembly_v = ((xm->fr_RR_hipassembly_X_fr_trunk) * trunk_v);
    RR_hipassembly_v(iit::rbd::AZ) += qd(RR_Q1_JOINT);
    motionCrossProductMx(RR_hipassembly_v, vcross);
    RR_hipassembly_a = (vcross.col(iit::rbd::AZ) * qd(RR_Q1_JOINT));
    RR_hipassembly_f = RR_hipassembly_I * RR_hipassembly_a + vxIv(RR_hipassembly_v, RR_hipassembly_I);
    
    // Link 'LM_upperleg'
    LM_upperleg_v = ((xm->fr_LM_upperleg_X_fr_LM_hipassembly) * LM_hipassembly_v);
    LM_upperleg_v(iit::rbd::AZ) += qd(LM_Q2_JOINT);
    motionCrossProductMx(LM_upperleg_v, vcross);
    LM_upperleg_a = (xm->fr_LM_upperleg_X_fr_LM_hipassembly) * LM_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(LM_Q2_JOINT);
    LM_upperleg_f = LM_upperleg_I * LM_upperleg_a + vxIv(LM_upperleg_v, LM_upperleg_I);
    
    // Link 'LR_upperleg'
    LR_upperleg_v = ((xm->fr_LR_upperleg_X_fr_LR_hipassembly) * LR_hipassembly_v);
    LR_upperleg_v(iit::rbd::AZ) += qd(LR_Q2_JOINT);
    motionCrossProductMx(LR_upperleg_v, vcross);
    LR_upperleg_a = (xm->fr_LR_upperleg_X_fr_LR_hipassembly) * LR_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(LR_Q2_JOINT);
    LR_upperleg_f = LR_upperleg_I * LR_upperleg_a + vxIv(LR_upperleg_v, LR_upperleg_I);
    
    // Link 'RM_upperleg'
    RM_upperleg_v = ((xm->fr_RM_upperleg_X_fr_RM_hipassembly) * RM_hipassembly_v);
    RM_upperleg_v(iit::rbd::AZ) += qd(RM_Q2_JOINT);
    motionCrossProductMx(RM_upperleg_v, vcross);
    RM_upperleg_a = (xm->fr_RM_upperleg_X_fr_RM_hipassembly) * RM_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(RM_Q2_JOINT);
    RM_upperleg_f = RM_upperleg_I * RM_upperleg_a + vxIv(RM_upperleg_v, RM_upperleg_I);
    
    // Link 'RR_upperleg'
    RR_upperleg_v = ((xm->fr_RR_upperleg_X_fr_RR_hipassembly) * RR_hipassembly_v);
    RR_upperleg_v(iit::rbd::AZ) += qd(RR_Q2_JOINT);
    motionCrossProductMx(RR_upperleg_v, vcross);
    RR_upperleg_a = (xm->fr_RR_upperleg_X_fr_RR_hipassembly) * RR_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(RR_Q2_JOINT);
    RR_upperleg_f = RR_upperleg_I * RR_upperleg_a + vxIv(RR_upperleg_v, RR_upperleg_I);
    
    // Link 'LM_lowerleg'
    LM_lowerleg_v = ((xm->fr_LM_lowerleg_X_fr_LM_upperleg) * LM_upperleg_v);
    LM_lowerleg_v(iit::rbd::AZ) += qd(LM_Q3_JOINT);
    motionCrossProductMx(LM_lowerleg_v, vcross);
    LM_lowerleg_a = (xm->fr_LM_lowerleg_X_fr_LM_upperleg) * LM_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LM_Q3_JOINT);
    LM_lowerleg_f = LM_lowerleg_I * LM_lowerleg_a + vxIv(LM_lowerleg_v, LM_lowerleg_I);
    
    // Link 'LR_lowerleg'
    LR_lowerleg_v = ((xm->fr_LR_lowerleg_X_fr_LR_upperleg) * LR_upperleg_v);
    LR_lowerleg_v(iit::rbd::AZ) += qd(LR_Q3_JOINT);
    motionCrossProductMx(LR_lowerleg_v, vcross);
    LR_lowerleg_a = (xm->fr_LR_lowerleg_X_fr_LR_upperleg) * LR_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LR_Q3_JOINT);
    LR_lowerleg_f = LR_lowerleg_I * LR_lowerleg_a + vxIv(LR_lowerleg_v, LR_lowerleg_I);
    
    // Link 'RM_lowerleg'
    RM_lowerleg_v = ((xm->fr_RM_lowerleg_X_fr_RM_upperleg) * RM_upperleg_v);
    RM_lowerleg_v(iit::rbd::AZ) += qd(RM_Q3_JOINT);
    motionCrossProductMx(RM_lowerleg_v, vcross);
    RM_lowerleg_a = (xm->fr_RM_lowerleg_X_fr_RM_upperleg) * RM_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RM_Q3_JOINT);
    RM_lowerleg_f = RM_lowerleg_I * RM_lowerleg_a + vxIv(RM_lowerleg_v, RM_lowerleg_I);
    
    // Link 'RR_lowerleg'
    RR_lowerleg_v = ((xm->fr_RR_lowerleg_X_fr_RR_upperleg) * RR_upperleg_v);
    RR_lowerleg_v(iit::rbd::AZ) += qd(RR_Q3_JOINT);
    motionCrossProductMx(RR_lowerleg_v, vcross);
    RR_lowerleg_a = (xm->fr_RR_lowerleg_X_fr_RR_upperleg) * RR_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RR_Q3_JOINT);
    RR_lowerleg_f = RR_lowerleg_I * RR_lowerleg_a + vxIv(RR_lowerleg_v, RR_lowerleg_I);
    

    trunk_f = vxIv(trunk_v, trunk_I);

    secondPass_fullyActuated(jForces);

    baseWrench = trunk_f;
}

void iit::mcorin::dyn::InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& trunk_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    Acceleration trunk_a = baseAccel -g;

    // First pass, link 'LF_hipassembly'
    LF_hipassembly_v = ((xm->fr_LF_hipassembly_X_fr_trunk) * trunk_v);
    LF_hipassembly_v(iit::rbd::AZ) += qd(LF_Q1_JOINT);
    
    motionCrossProductMx(LF_hipassembly_v, vcross);
    
    LF_hipassembly_a = (xm->fr_LF_hipassembly_X_fr_trunk) * trunk_a + vcross.col(iit::rbd::AZ) * qd(LF_Q1_JOINT);
    LF_hipassembly_a(iit::rbd::AZ) += qdd(LF_Q1_JOINT);
    
    LF_hipassembly_f = LF_hipassembly_I * LF_hipassembly_a + vxIv(LF_hipassembly_v, LF_hipassembly_I) - fext[LF_HIPASSEMBLY];
    
    // First pass, link 'RF_hipassembly'
    RF_hipassembly_v = ((xm->fr_RF_hipassembly_X_fr_trunk) * trunk_v);
    RF_hipassembly_v(iit::rbd::AZ) += qd(RF_Q1_JOINT);
    
    motionCrossProductMx(RF_hipassembly_v, vcross);
    
    RF_hipassembly_a = (xm->fr_RF_hipassembly_X_fr_trunk) * trunk_a + vcross.col(iit::rbd::AZ) * qd(RF_Q1_JOINT);
    RF_hipassembly_a(iit::rbd::AZ) += qdd(RF_Q1_JOINT);
    
    RF_hipassembly_f = RF_hipassembly_I * RF_hipassembly_a + vxIv(RF_hipassembly_v, RF_hipassembly_I) - fext[RF_HIPASSEMBLY];
    
    // First pass, link 'LF_upperleg'
    LF_upperleg_v = ((xm->fr_LF_upperleg_X_fr_LF_hipassembly) * LF_hipassembly_v);
    LF_upperleg_v(iit::rbd::AZ) += qd(LF_Q2_JOINT);
    
    motionCrossProductMx(LF_upperleg_v, vcross);
    
    LF_upperleg_a = (xm->fr_LF_upperleg_X_fr_LF_hipassembly) * LF_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(LF_Q2_JOINT);
    LF_upperleg_a(iit::rbd::AZ) += qdd(LF_Q2_JOINT);
    
    LF_upperleg_f = LF_upperleg_I * LF_upperleg_a + vxIv(LF_upperleg_v, LF_upperleg_I) - fext[LF_UPPERLEG];
    
    // First pass, link 'RF_upperleg'
    RF_upperleg_v = ((xm->fr_RF_upperleg_X_fr_RF_hipassembly) * RF_hipassembly_v);
    RF_upperleg_v(iit::rbd::AZ) += qd(RF_Q2_JOINT);
    
    motionCrossProductMx(RF_upperleg_v, vcross);
    
    RF_upperleg_a = (xm->fr_RF_upperleg_X_fr_RF_hipassembly) * RF_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(RF_Q2_JOINT);
    RF_upperleg_a(iit::rbd::AZ) += qdd(RF_Q2_JOINT);
    
    RF_upperleg_f = RF_upperleg_I * RF_upperleg_a + vxIv(RF_upperleg_v, RF_upperleg_I) - fext[RF_UPPERLEG];
    
    // First pass, link 'LF_lowerleg'
    LF_lowerleg_v = ((xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_v);
    LF_lowerleg_v(iit::rbd::AZ) += qd(LF_Q3_JOINT);
    
    motionCrossProductMx(LF_lowerleg_v, vcross);
    
    LF_lowerleg_a = (xm->fr_LF_lowerleg_X_fr_LF_upperleg) * LF_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LF_Q3_JOINT);
    LF_lowerleg_a(iit::rbd::AZ) += qdd(LF_Q3_JOINT);
    
    LF_lowerleg_f = LF_lowerleg_I * LF_lowerleg_a + vxIv(LF_lowerleg_v, LF_lowerleg_I) - fext[LF_LOWERLEG];
    
    // First pass, link 'RF_lowerleg'
    RF_lowerleg_v = ((xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_v);
    RF_lowerleg_v(iit::rbd::AZ) += qd(RF_Q3_JOINT);
    
    motionCrossProductMx(RF_lowerleg_v, vcross);
    
    RF_lowerleg_a = (xm->fr_RF_lowerleg_X_fr_RF_upperleg) * RF_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RF_Q3_JOINT);
    RF_lowerleg_a(iit::rbd::AZ) += qdd(RF_Q3_JOINT);
    
    RF_lowerleg_f = RF_lowerleg_I * RF_lowerleg_a + vxIv(RF_lowerleg_v, RF_lowerleg_I) - fext[RF_LOWERLEG];
    
    // First pass, link 'LM_hipassembly'
    LM_hipassembly_v = ((xm->fr_LM_hipassembly_X_fr_trunk) * trunk_v);
    LM_hipassembly_v(iit::rbd::AZ) += qd(LM_Q1_JOINT);
    
    motionCrossProductMx(LM_hipassembly_v, vcross);
    
    LM_hipassembly_a = (xm->fr_LM_hipassembly_X_fr_trunk) * trunk_a + vcross.col(iit::rbd::AZ) * qd(LM_Q1_JOINT);
    LM_hipassembly_a(iit::rbd::AZ) += qdd(LM_Q1_JOINT);
    
    LM_hipassembly_f = LM_hipassembly_I * LM_hipassembly_a + vxIv(LM_hipassembly_v, LM_hipassembly_I) - fext[LM_HIPASSEMBLY];
    
    // First pass, link 'LR_hipassembly'
    LR_hipassembly_v = ((xm->fr_LR_hipassembly_X_fr_trunk) * trunk_v);
    LR_hipassembly_v(iit::rbd::AZ) += qd(LR_Q1_JOINT);
    
    motionCrossProductMx(LR_hipassembly_v, vcross);
    
    LR_hipassembly_a = (xm->fr_LR_hipassembly_X_fr_trunk) * trunk_a + vcross.col(iit::rbd::AZ) * qd(LR_Q1_JOINT);
    LR_hipassembly_a(iit::rbd::AZ) += qdd(LR_Q1_JOINT);
    
    LR_hipassembly_f = LR_hipassembly_I * LR_hipassembly_a + vxIv(LR_hipassembly_v, LR_hipassembly_I) - fext[LR_HIPASSEMBLY];
    
    // First pass, link 'RM_hipassembly'
    RM_hipassembly_v = ((xm->fr_RM_hipassembly_X_fr_trunk) * trunk_v);
    RM_hipassembly_v(iit::rbd::AZ) += qd(RM_Q1_JOINT);
    
    motionCrossProductMx(RM_hipassembly_v, vcross);
    
    RM_hipassembly_a = (xm->fr_RM_hipassembly_X_fr_trunk) * trunk_a + vcross.col(iit::rbd::AZ) * qd(RM_Q1_JOINT);
    RM_hipassembly_a(iit::rbd::AZ) += qdd(RM_Q1_JOINT);
    
    RM_hipassembly_f = RM_hipassembly_I * RM_hipassembly_a + vxIv(RM_hipassembly_v, RM_hipassembly_I) - fext[RM_HIPASSEMBLY];
    
    // First pass, link 'RR_hipassembly'
    RR_hipassembly_v = ((xm->fr_RR_hipassembly_X_fr_trunk) * trunk_v);
    RR_hipassembly_v(iit::rbd::AZ) += qd(RR_Q1_JOINT);
    
    motionCrossProductMx(RR_hipassembly_v, vcross);
    
    RR_hipassembly_a = (xm->fr_RR_hipassembly_X_fr_trunk) * trunk_a + vcross.col(iit::rbd::AZ) * qd(RR_Q1_JOINT);
    RR_hipassembly_a(iit::rbd::AZ) += qdd(RR_Q1_JOINT);
    
    RR_hipassembly_f = RR_hipassembly_I * RR_hipassembly_a + vxIv(RR_hipassembly_v, RR_hipassembly_I) - fext[RR_HIPASSEMBLY];
    
    // First pass, link 'LM_upperleg'
    LM_upperleg_v = ((xm->fr_LM_upperleg_X_fr_LM_hipassembly) * LM_hipassembly_v);
    LM_upperleg_v(iit::rbd::AZ) += qd(LM_Q2_JOINT);
    
    motionCrossProductMx(LM_upperleg_v, vcross);
    
    LM_upperleg_a = (xm->fr_LM_upperleg_X_fr_LM_hipassembly) * LM_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(LM_Q2_JOINT);
    LM_upperleg_a(iit::rbd::AZ) += qdd(LM_Q2_JOINT);
    
    LM_upperleg_f = LM_upperleg_I * LM_upperleg_a + vxIv(LM_upperleg_v, LM_upperleg_I) - fext[LM_UPPERLEG];
    
    // First pass, link 'LR_upperleg'
    LR_upperleg_v = ((xm->fr_LR_upperleg_X_fr_LR_hipassembly) * LR_hipassembly_v);
    LR_upperleg_v(iit::rbd::AZ) += qd(LR_Q2_JOINT);
    
    motionCrossProductMx(LR_upperleg_v, vcross);
    
    LR_upperleg_a = (xm->fr_LR_upperleg_X_fr_LR_hipassembly) * LR_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(LR_Q2_JOINT);
    LR_upperleg_a(iit::rbd::AZ) += qdd(LR_Q2_JOINT);
    
    LR_upperleg_f = LR_upperleg_I * LR_upperleg_a + vxIv(LR_upperleg_v, LR_upperleg_I) - fext[LR_UPPERLEG];
    
    // First pass, link 'RM_upperleg'
    RM_upperleg_v = ((xm->fr_RM_upperleg_X_fr_RM_hipassembly) * RM_hipassembly_v);
    RM_upperleg_v(iit::rbd::AZ) += qd(RM_Q2_JOINT);
    
    motionCrossProductMx(RM_upperleg_v, vcross);
    
    RM_upperleg_a = (xm->fr_RM_upperleg_X_fr_RM_hipassembly) * RM_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(RM_Q2_JOINT);
    RM_upperleg_a(iit::rbd::AZ) += qdd(RM_Q2_JOINT);
    
    RM_upperleg_f = RM_upperleg_I * RM_upperleg_a + vxIv(RM_upperleg_v, RM_upperleg_I) - fext[RM_UPPERLEG];
    
    // First pass, link 'RR_upperleg'
    RR_upperleg_v = ((xm->fr_RR_upperleg_X_fr_RR_hipassembly) * RR_hipassembly_v);
    RR_upperleg_v(iit::rbd::AZ) += qd(RR_Q2_JOINT);
    
    motionCrossProductMx(RR_upperleg_v, vcross);
    
    RR_upperleg_a = (xm->fr_RR_upperleg_X_fr_RR_hipassembly) * RR_hipassembly_a + vcross.col(iit::rbd::AZ) * qd(RR_Q2_JOINT);
    RR_upperleg_a(iit::rbd::AZ) += qdd(RR_Q2_JOINT);
    
    RR_upperleg_f = RR_upperleg_I * RR_upperleg_a + vxIv(RR_upperleg_v, RR_upperleg_I) - fext[RR_UPPERLEG];
    
    // First pass, link 'LM_lowerleg'
    LM_lowerleg_v = ((xm->fr_LM_lowerleg_X_fr_LM_upperleg) * LM_upperleg_v);
    LM_lowerleg_v(iit::rbd::AZ) += qd(LM_Q3_JOINT);
    
    motionCrossProductMx(LM_lowerleg_v, vcross);
    
    LM_lowerleg_a = (xm->fr_LM_lowerleg_X_fr_LM_upperleg) * LM_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LM_Q3_JOINT);
    LM_lowerleg_a(iit::rbd::AZ) += qdd(LM_Q3_JOINT);
    
    LM_lowerleg_f = LM_lowerleg_I * LM_lowerleg_a + vxIv(LM_lowerleg_v, LM_lowerleg_I) - fext[LM_LOWERLEG];
    
    // First pass, link 'LR_lowerleg'
    LR_lowerleg_v = ((xm->fr_LR_lowerleg_X_fr_LR_upperleg) * LR_upperleg_v);
    LR_lowerleg_v(iit::rbd::AZ) += qd(LR_Q3_JOINT);
    
    motionCrossProductMx(LR_lowerleg_v, vcross);
    
    LR_lowerleg_a = (xm->fr_LR_lowerleg_X_fr_LR_upperleg) * LR_upperleg_a + vcross.col(iit::rbd::AZ) * qd(LR_Q3_JOINT);
    LR_lowerleg_a(iit::rbd::AZ) += qdd(LR_Q3_JOINT);
    
    LR_lowerleg_f = LR_lowerleg_I * LR_lowerleg_a + vxIv(LR_lowerleg_v, LR_lowerleg_I) - fext[LR_LOWERLEG];
    
    // First pass, link 'RM_lowerleg'
    RM_lowerleg_v = ((xm->fr_RM_lowerleg_X_fr_RM_upperleg) * RM_upperleg_v);
    RM_lowerleg_v(iit::rbd::AZ) += qd(RM_Q3_JOINT);
    
    motionCrossProductMx(RM_lowerleg_v, vcross);
    
    RM_lowerleg_a = (xm->fr_RM_lowerleg_X_fr_RM_upperleg) * RM_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RM_Q3_JOINT);
    RM_lowerleg_a(iit::rbd::AZ) += qdd(RM_Q3_JOINT);
    
    RM_lowerleg_f = RM_lowerleg_I * RM_lowerleg_a + vxIv(RM_lowerleg_v, RM_lowerleg_I) - fext[RM_LOWERLEG];
    
    // First pass, link 'RR_lowerleg'
    RR_lowerleg_v = ((xm->fr_RR_lowerleg_X_fr_RR_upperleg) * RR_upperleg_v);
    RR_lowerleg_v(iit::rbd::AZ) += qd(RR_Q3_JOINT);
    
    motionCrossProductMx(RR_lowerleg_v, vcross);
    
    RR_lowerleg_a = (xm->fr_RR_lowerleg_X_fr_RR_upperleg) * RR_upperleg_a + vcross.col(iit::rbd::AZ) * qd(RR_Q3_JOINT);
    RR_lowerleg_a(iit::rbd::AZ) += qdd(RR_Q3_JOINT);
    
    RR_lowerleg_f = RR_lowerleg_I * RR_lowerleg_a + vxIv(RR_lowerleg_v, RR_lowerleg_I) - fext[RR_LOWERLEG];
    

    // The base
    trunk_f = trunk_I * trunk_a + vxIv(trunk_v, trunk_I) - fext[TRUNK];

    secondPass_fullyActuated(jForces);

    baseWrench = trunk_f;
}


void iit::mcorin::dyn::InverseDynamics::secondPass_fullyActuated(JointState& jForces)
{
    // Link 'RR_lowerleg'
    jForces(RR_Q3_JOINT) = RR_lowerleg_f(iit::rbd::AZ);
    RR_upperleg_f += xm->fr_RR_lowerleg_X_fr_RR_upperleg.transpose() * RR_lowerleg_f;
    // Link 'RM_lowerleg'
    jForces(RM_Q3_JOINT) = RM_lowerleg_f(iit::rbd::AZ);
    RM_upperleg_f += xm->fr_RM_lowerleg_X_fr_RM_upperleg.transpose() * RM_lowerleg_f;
    // Link 'LR_lowerleg'
    jForces(LR_Q3_JOINT) = LR_lowerleg_f(iit::rbd::AZ);
    LR_upperleg_f += xm->fr_LR_lowerleg_X_fr_LR_upperleg.transpose() * LR_lowerleg_f;
    // Link 'LM_lowerleg'
    jForces(LM_Q3_JOINT) = LM_lowerleg_f(iit::rbd::AZ);
    LM_upperleg_f += xm->fr_LM_lowerleg_X_fr_LM_upperleg.transpose() * LM_lowerleg_f;
    // Link 'RR_upperleg'
    jForces(RR_Q2_JOINT) = RR_upperleg_f(iit::rbd::AZ);
    RR_hipassembly_f += xm->fr_RR_upperleg_X_fr_RR_hipassembly.transpose() * RR_upperleg_f;
    // Link 'RM_upperleg'
    jForces(RM_Q2_JOINT) = RM_upperleg_f(iit::rbd::AZ);
    RM_hipassembly_f += xm->fr_RM_upperleg_X_fr_RM_hipassembly.transpose() * RM_upperleg_f;
    // Link 'LR_upperleg'
    jForces(LR_Q2_JOINT) = LR_upperleg_f(iit::rbd::AZ);
    LR_hipassembly_f += xm->fr_LR_upperleg_X_fr_LR_hipassembly.transpose() * LR_upperleg_f;
    // Link 'LM_upperleg'
    jForces(LM_Q2_JOINT) = LM_upperleg_f(iit::rbd::AZ);
    LM_hipassembly_f += xm->fr_LM_upperleg_X_fr_LM_hipassembly.transpose() * LM_upperleg_f;
    // Link 'RR_hipassembly'
    jForces(RR_Q1_JOINT) = RR_hipassembly_f(iit::rbd::AZ);
    trunk_f += xm->fr_RR_hipassembly_X_fr_trunk.transpose() * RR_hipassembly_f;
    // Link 'RM_hipassembly'
    jForces(RM_Q1_JOINT) = RM_hipassembly_f(iit::rbd::AZ);
    trunk_f += xm->fr_RM_hipassembly_X_fr_trunk.transpose() * RM_hipassembly_f;
    // Link 'LR_hipassembly'
    jForces(LR_Q1_JOINT) = LR_hipassembly_f(iit::rbd::AZ);
    trunk_f += xm->fr_LR_hipassembly_X_fr_trunk.transpose() * LR_hipassembly_f;
    // Link 'LM_hipassembly'
    jForces(LM_Q1_JOINT) = LM_hipassembly_f(iit::rbd::AZ);
    trunk_f += xm->fr_LM_hipassembly_X_fr_trunk.transpose() * LM_hipassembly_f;
    // Link 'RF_lowerleg'
    jForces(RF_Q3_JOINT) = RF_lowerleg_f(iit::rbd::AZ);
    RF_upperleg_f += xm->fr_RF_lowerleg_X_fr_RF_upperleg.transpose() * RF_lowerleg_f;
    // Link 'LF_lowerleg'
    jForces(LF_Q3_JOINT) = LF_lowerleg_f(iit::rbd::AZ);
    LF_upperleg_f += xm->fr_LF_lowerleg_X_fr_LF_upperleg.transpose() * LF_lowerleg_f;
    // Link 'RF_upperleg'
    jForces(RF_Q2_JOINT) = RF_upperleg_f(iit::rbd::AZ);
    RF_hipassembly_f += xm->fr_RF_upperleg_X_fr_RF_hipassembly.transpose() * RF_upperleg_f;
    // Link 'LF_upperleg'
    jForces(LF_Q2_JOINT) = LF_upperleg_f(iit::rbd::AZ);
    LF_hipassembly_f += xm->fr_LF_upperleg_X_fr_LF_hipassembly.transpose() * LF_upperleg_f;
    // Link 'RF_hipassembly'
    jForces(RF_Q1_JOINT) = RF_hipassembly_f(iit::rbd::AZ);
    trunk_f += xm->fr_RF_hipassembly_X_fr_trunk.transpose() * RF_hipassembly_f;
    // Link 'LF_hipassembly'
    jForces(LF_Q1_JOINT) = LF_hipassembly_f(iit::rbd::AZ);
    trunk_f += xm->fr_LF_hipassembly_X_fr_trunk.transpose() * LF_hipassembly_f;
}

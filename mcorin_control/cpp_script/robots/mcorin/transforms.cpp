#include "transforms.h"

// Constructors

iit::mcorin::MotionTransforms::MotionTransforms
    ()
     :
    fr_trunk_X_LF_hipassemblyCOM(),
    fr_trunk_X_LF_upperlegCOM(),
    fr_trunk_X_LF_lowerlegCOM(),
    fr_trunk_X_LM_hipassemblyCOM(),
    fr_trunk_X_LM_upperlegCOM(),
    fr_trunk_X_LM_lowerlegCOM(),
    fr_trunk_X_LR_hipassemblyCOM(),
    fr_trunk_X_LR_upperlegCOM(),
    fr_trunk_X_LR_lowerlegCOM(),
    fr_trunk_X_RF_hipassemblyCOM(),
    fr_trunk_X_RF_upperlegCOM(),
    fr_trunk_X_RF_lowerlegCOM(),
    fr_trunk_X_RM_hipassemblyCOM(),
    fr_trunk_X_RM_upperlegCOM(),
    fr_trunk_X_RM_lowerlegCOM(),
    fr_trunk_X_RR_hipassemblyCOM(),
    fr_trunk_X_RR_upperlegCOM(),
    fr_trunk_X_RR_lowerlegCOM(),
    fr_LF_upperleg_X_fr_trunk(),
    fr_LF_lowerleg_X_fr_trunk(),
    fr_LM_upperleg_X_fr_trunk(),
    fr_LM_lowerleg_X_fr_trunk(),
    fr_LR_upperleg_X_fr_trunk(),
    fr_LR_lowerleg_X_fr_trunk(),
    fr_RF_upperleg_X_fr_trunk(),
    fr_RF_lowerleg_X_fr_trunk(),
    fr_RM_upperleg_X_fr_trunk(),
    fr_RM_lowerleg_X_fr_trunk(),
    fr_RR_upperleg_X_fr_trunk(),
    fr_RR_lowerleg_X_fr_trunk(),
    fr_trunk_X_LF_foot(),
    fr_trunk_X_fr_LF_q1_joint(),
    fr_trunk_X_fr_LF_q2_joint(),
    fr_trunk_X_fr_LF_q3_joint(),
    fr_trunk_X_LM_foot(),
    fr_trunk_X_fr_LM_q1_joint(),
    fr_trunk_X_fr_LM_q2_joint(),
    fr_trunk_X_fr_LM_q3_joint(),
    fr_trunk_X_LR_foot(),
    fr_trunk_X_fr_LR_q1_joint(),
    fr_trunk_X_fr_LR_q2_joint(),
    fr_trunk_X_fr_LR_q3_joint(),
    fr_trunk_X_RF_foot(),
    fr_trunk_X_fr_RF_q1_joint(),
    fr_trunk_X_fr_RF_q2_joint(),
    fr_trunk_X_fr_RF_q3_joint(),
    fr_trunk_X_RM_foot(),
    fr_trunk_X_fr_RM_q1_joint(),
    fr_trunk_X_fr_RM_q2_joint(),
    fr_trunk_X_fr_RM_q3_joint(),
    fr_trunk_X_RR_foot(),
    fr_trunk_X_fr_RR_q1_joint(),
    fr_trunk_X_fr_RR_q2_joint(),
    fr_trunk_X_fr_RR_q3_joint(),
    fr_LF_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_LF_hipassembly(),
    fr_LF_upperleg_X_fr_LF_hipassembly(),
    fr_LF_hipassembly_X_fr_LF_upperleg(),
    fr_LF_lowerleg_X_fr_LF_upperleg(),
    fr_LF_upperleg_X_fr_LF_lowerleg(),
    fr_LM_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_LM_hipassembly(),
    fr_LM_upperleg_X_fr_LM_hipassembly(),
    fr_LM_hipassembly_X_fr_LM_upperleg(),
    fr_LM_lowerleg_X_fr_LM_upperleg(),
    fr_LM_upperleg_X_fr_LM_lowerleg(),
    fr_LR_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_LR_hipassembly(),
    fr_LR_upperleg_X_fr_LR_hipassembly(),
    fr_LR_hipassembly_X_fr_LR_upperleg(),
    fr_LR_lowerleg_X_fr_LR_upperleg(),
    fr_LR_upperleg_X_fr_LR_lowerleg(),
    fr_RF_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_RF_hipassembly(),
    fr_RF_upperleg_X_fr_RF_hipassembly(),
    fr_RF_hipassembly_X_fr_RF_upperleg(),
    fr_RF_lowerleg_X_fr_RF_upperleg(),
    fr_RF_upperleg_X_fr_RF_lowerleg(),
    fr_RM_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_RM_hipassembly(),
    fr_RM_upperleg_X_fr_RM_hipassembly(),
    fr_RM_hipassembly_X_fr_RM_upperleg(),
    fr_RM_lowerleg_X_fr_RM_upperleg(),
    fr_RM_upperleg_X_fr_RM_lowerleg(),
    fr_RR_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_RR_hipassembly(),
    fr_RR_upperleg_X_fr_RR_hipassembly(),
    fr_RR_hipassembly_X_fr_RR_upperleg(),
    fr_RR_lowerleg_X_fr_RR_upperleg(),
    fr_RR_upperleg_X_fr_RR_lowerleg()
{
    updateParameters();
}
void iit::mcorin::MotionTransforms::updateParameters() {
}

iit::mcorin::ForceTransforms::ForceTransforms
    ()
     :
    fr_trunk_X_LF_hipassemblyCOM(),
    fr_trunk_X_LF_upperlegCOM(),
    fr_trunk_X_LF_lowerlegCOM(),
    fr_trunk_X_LM_hipassemblyCOM(),
    fr_trunk_X_LM_upperlegCOM(),
    fr_trunk_X_LM_lowerlegCOM(),
    fr_trunk_X_LR_hipassemblyCOM(),
    fr_trunk_X_LR_upperlegCOM(),
    fr_trunk_X_LR_lowerlegCOM(),
    fr_trunk_X_RF_hipassemblyCOM(),
    fr_trunk_X_RF_upperlegCOM(),
    fr_trunk_X_RF_lowerlegCOM(),
    fr_trunk_X_RM_hipassemblyCOM(),
    fr_trunk_X_RM_upperlegCOM(),
    fr_trunk_X_RM_lowerlegCOM(),
    fr_trunk_X_RR_hipassemblyCOM(),
    fr_trunk_X_RR_upperlegCOM(),
    fr_trunk_X_RR_lowerlegCOM(),
    fr_LF_upperleg_X_fr_trunk(),
    fr_LF_lowerleg_X_fr_trunk(),
    fr_LM_upperleg_X_fr_trunk(),
    fr_LM_lowerleg_X_fr_trunk(),
    fr_LR_upperleg_X_fr_trunk(),
    fr_LR_lowerleg_X_fr_trunk(),
    fr_RF_upperleg_X_fr_trunk(),
    fr_RF_lowerleg_X_fr_trunk(),
    fr_RM_upperleg_X_fr_trunk(),
    fr_RM_lowerleg_X_fr_trunk(),
    fr_RR_upperleg_X_fr_trunk(),
    fr_RR_lowerleg_X_fr_trunk(),
    fr_trunk_X_LF_foot(),
    fr_trunk_X_fr_LF_q1_joint(),
    fr_trunk_X_fr_LF_q2_joint(),
    fr_trunk_X_fr_LF_q3_joint(),
    fr_trunk_X_LM_foot(),
    fr_trunk_X_fr_LM_q1_joint(),
    fr_trunk_X_fr_LM_q2_joint(),
    fr_trunk_X_fr_LM_q3_joint(),
    fr_trunk_X_LR_foot(),
    fr_trunk_X_fr_LR_q1_joint(),
    fr_trunk_X_fr_LR_q2_joint(),
    fr_trunk_X_fr_LR_q3_joint(),
    fr_trunk_X_RF_foot(),
    fr_trunk_X_fr_RF_q1_joint(),
    fr_trunk_X_fr_RF_q2_joint(),
    fr_trunk_X_fr_RF_q3_joint(),
    fr_trunk_X_RM_foot(),
    fr_trunk_X_fr_RM_q1_joint(),
    fr_trunk_X_fr_RM_q2_joint(),
    fr_trunk_X_fr_RM_q3_joint(),
    fr_trunk_X_RR_foot(),
    fr_trunk_X_fr_RR_q1_joint(),
    fr_trunk_X_fr_RR_q2_joint(),
    fr_trunk_X_fr_RR_q3_joint(),
    fr_LF_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_LF_hipassembly(),
    fr_LF_upperleg_X_fr_LF_hipassembly(),
    fr_LF_hipassembly_X_fr_LF_upperleg(),
    fr_LF_lowerleg_X_fr_LF_upperleg(),
    fr_LF_upperleg_X_fr_LF_lowerleg(),
    fr_LM_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_LM_hipassembly(),
    fr_LM_upperleg_X_fr_LM_hipassembly(),
    fr_LM_hipassembly_X_fr_LM_upperleg(),
    fr_LM_lowerleg_X_fr_LM_upperleg(),
    fr_LM_upperleg_X_fr_LM_lowerleg(),
    fr_LR_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_LR_hipassembly(),
    fr_LR_upperleg_X_fr_LR_hipassembly(),
    fr_LR_hipassembly_X_fr_LR_upperleg(),
    fr_LR_lowerleg_X_fr_LR_upperleg(),
    fr_LR_upperleg_X_fr_LR_lowerleg(),
    fr_RF_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_RF_hipassembly(),
    fr_RF_upperleg_X_fr_RF_hipassembly(),
    fr_RF_hipassembly_X_fr_RF_upperleg(),
    fr_RF_lowerleg_X_fr_RF_upperleg(),
    fr_RF_upperleg_X_fr_RF_lowerleg(),
    fr_RM_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_RM_hipassembly(),
    fr_RM_upperleg_X_fr_RM_hipassembly(),
    fr_RM_hipassembly_X_fr_RM_upperleg(),
    fr_RM_lowerleg_X_fr_RM_upperleg(),
    fr_RM_upperleg_X_fr_RM_lowerleg(),
    fr_RR_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_RR_hipassembly(),
    fr_RR_upperleg_X_fr_RR_hipassembly(),
    fr_RR_hipassembly_X_fr_RR_upperleg(),
    fr_RR_lowerleg_X_fr_RR_upperleg(),
    fr_RR_upperleg_X_fr_RR_lowerleg()
{
    updateParameters();
}
void iit::mcorin::ForceTransforms::updateParameters() {
}

iit::mcorin::HomogeneousTransforms::HomogeneousTransforms
    ()
     :
    fr_trunk_X_LF_hipassemblyCOM(),
    fr_trunk_X_LF_upperlegCOM(),
    fr_trunk_X_LF_lowerlegCOM(),
    fr_trunk_X_LM_hipassemblyCOM(),
    fr_trunk_X_LM_upperlegCOM(),
    fr_trunk_X_LM_lowerlegCOM(),
    fr_trunk_X_LR_hipassemblyCOM(),
    fr_trunk_X_LR_upperlegCOM(),
    fr_trunk_X_LR_lowerlegCOM(),
    fr_trunk_X_RF_hipassemblyCOM(),
    fr_trunk_X_RF_upperlegCOM(),
    fr_trunk_X_RF_lowerlegCOM(),
    fr_trunk_X_RM_hipassemblyCOM(),
    fr_trunk_X_RM_upperlegCOM(),
    fr_trunk_X_RM_lowerlegCOM(),
    fr_trunk_X_RR_hipassemblyCOM(),
    fr_trunk_X_RR_upperlegCOM(),
    fr_trunk_X_RR_lowerlegCOM(),
    fr_LF_upperleg_X_fr_trunk(),
    fr_LF_lowerleg_X_fr_trunk(),
    fr_LM_upperleg_X_fr_trunk(),
    fr_LM_lowerleg_X_fr_trunk(),
    fr_LR_upperleg_X_fr_trunk(),
    fr_LR_lowerleg_X_fr_trunk(),
    fr_RF_upperleg_X_fr_trunk(),
    fr_RF_lowerleg_X_fr_trunk(),
    fr_RM_upperleg_X_fr_trunk(),
    fr_RM_lowerleg_X_fr_trunk(),
    fr_RR_upperleg_X_fr_trunk(),
    fr_RR_lowerleg_X_fr_trunk(),
    fr_trunk_X_LF_foot(),
    fr_trunk_X_fr_LF_q1_joint(),
    fr_trunk_X_fr_LF_q2_joint(),
    fr_trunk_X_fr_LF_q3_joint(),
    fr_trunk_X_LM_foot(),
    fr_trunk_X_fr_LM_q1_joint(),
    fr_trunk_X_fr_LM_q2_joint(),
    fr_trunk_X_fr_LM_q3_joint(),
    fr_trunk_X_LR_foot(),
    fr_trunk_X_fr_LR_q1_joint(),
    fr_trunk_X_fr_LR_q2_joint(),
    fr_trunk_X_fr_LR_q3_joint(),
    fr_trunk_X_RF_foot(),
    fr_trunk_X_fr_RF_q1_joint(),
    fr_trunk_X_fr_RF_q2_joint(),
    fr_trunk_X_fr_RF_q3_joint(),
    fr_trunk_X_RM_foot(),
    fr_trunk_X_fr_RM_q1_joint(),
    fr_trunk_X_fr_RM_q2_joint(),
    fr_trunk_X_fr_RM_q3_joint(),
    fr_trunk_X_RR_foot(),
    fr_trunk_X_fr_RR_q1_joint(),
    fr_trunk_X_fr_RR_q2_joint(),
    fr_trunk_X_fr_RR_q3_joint(),
    fr_LF_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_LF_hipassembly(),
    fr_LF_upperleg_X_fr_LF_hipassembly(),
    fr_LF_hipassembly_X_fr_LF_upperleg(),
    fr_LF_lowerleg_X_fr_LF_upperleg(),
    fr_LF_upperleg_X_fr_LF_lowerleg(),
    fr_LM_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_LM_hipassembly(),
    fr_LM_upperleg_X_fr_LM_hipassembly(),
    fr_LM_hipassembly_X_fr_LM_upperleg(),
    fr_LM_lowerleg_X_fr_LM_upperleg(),
    fr_LM_upperleg_X_fr_LM_lowerleg(),
    fr_LR_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_LR_hipassembly(),
    fr_LR_upperleg_X_fr_LR_hipassembly(),
    fr_LR_hipassembly_X_fr_LR_upperleg(),
    fr_LR_lowerleg_X_fr_LR_upperleg(),
    fr_LR_upperleg_X_fr_LR_lowerleg(),
    fr_RF_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_RF_hipassembly(),
    fr_RF_upperleg_X_fr_RF_hipassembly(),
    fr_RF_hipassembly_X_fr_RF_upperleg(),
    fr_RF_lowerleg_X_fr_RF_upperleg(),
    fr_RF_upperleg_X_fr_RF_lowerleg(),
    fr_RM_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_RM_hipassembly(),
    fr_RM_upperleg_X_fr_RM_hipassembly(),
    fr_RM_hipassembly_X_fr_RM_upperleg(),
    fr_RM_lowerleg_X_fr_RM_upperleg(),
    fr_RM_upperleg_X_fr_RM_lowerleg(),
    fr_RR_hipassembly_X_fr_trunk(),
    fr_trunk_X_fr_RR_hipassembly(),
    fr_RR_upperleg_X_fr_RR_hipassembly(),
    fr_RR_hipassembly_X_fr_RR_upperleg(),
    fr_RR_lowerleg_X_fr_RR_upperleg(),
    fr_RR_upperleg_X_fr_RR_lowerleg()
{
    updateParameters();
}
void iit::mcorin::HomogeneousTransforms::updateParameters() {
}

iit::mcorin::MotionTransforms::Type_fr_trunk_X_LF_hipassemblyCOM::Type_fr_trunk_X_LF_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_LF_hipassemblyCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_LF_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LF_q1_joint__;
    (*this)(0,1) = - cos__q_LF_q1_joint__;
    (*this)(1,0) =  cos__q_LF_q1_joint__;
    (*this)(1,1) = - sin__q_LF_q1_joint__;
    (*this)(3,2) = (( 0.05821 *  cos__q_LF_q1_joint__) +  0.075);
    (*this)(3,3) = - sin__q_LF_q1_joint__;
    (*this)(3,4) = - cos__q_LF_q1_joint__;
    (*this)(4,2) = (( 0.05821 *  sin__q_LF_q1_joint__) -  0.125);
    (*this)(4,3) =  cos__q_LF_q1_joint__;
    (*this)(4,4) = - sin__q_LF_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__));
    (*this)(5,1) = (((- 0.125 *  sin__q_LF_q1_joint__) + ( 0.075 *  cos__q_LF_q1_joint__)) +  0.05821);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_LF_upperlegCOM::Type_fr_trunk_X_LF_upperlegCOM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_LF_upperlegCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_LF_upperlegCOM::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(0,1) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(1,0) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(1,1) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(2,0) =  sin__q_LF_q2_joint__;
    (*this)(2,1) =  cos__q_LF_q2_joint__;
    (*this)(3,0) = ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  sin__q_LF_q2_joint__);
    (*this)(3,1) = ((( 0.075 + ( 0.077 *  cos__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) + ( 0.1213 *  cos__q_LF_q1_joint__));
    (*this)(3,2) = ((- 0.1213 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__);
    (*this)(3,3) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(3,4) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(3,5) =  cos__q_LF_q1_joint__;
    (*this)(4,0) = ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  sin__q_LF_q2_joint__);
    (*this)(4,1) = (((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__) + ( 0.1213 *  sin__q_LF_q1_joint__));
    (*this)(4,2) = (( 0.1213 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__);
    (*this)(4,3) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(4,4) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(4,5) =  sin__q_LF_q1_joint__;
    (*this)(5,0) = ((( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__);
    (*this)(5,1) = (((- 0.075 *  sin__q_LF_q1_joint__) - ( 0.125 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__);
    (*this)(5,2) = ((((- 0.1213 *  cos__q_LF_q2_joint__) + ( 0.125 *  sin__q_LF_q1_joint__)) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    (*this)(5,3) =  sin__q_LF_q2_joint__;
    (*this)(5,4) =  cos__q_LF_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_LF_lowerlegCOM::Type_fr_trunk_X_LF_lowerlegCOM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_LF_lowerlegCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_LF_lowerlegCOM::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q3_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,1) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(1,0) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(1,1) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(2,0) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(2,1) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    (*this)(3,0) = (((( 0.15 *  cos__q_LF_q1_joint__) + ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__)) *  sin__q_LF_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,1) = (((((- 0.075 - ( 0.077 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.15 *  cos__q_LF_q1_joint__) + ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__)) *  cos__q_LF_q3_joint__)) + ( 0.08853 *  cos__q_LF_q1_joint__));
    (*this)(3,2) = (((((- 0.08853 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - ((( 0.08853 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) - (( 0.15 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__));
    (*this)(3,3) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,4) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,5) =  cos__q_LF_q1_joint__;
    (*this)(4,0) = (((( 0.15 *  sin__q_LF_q1_joint__) + ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__)) *  sin__q_LF_q3_joint__) + (((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(4,1) = ((((( 0.125 - ( 0.077 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.15 *  sin__q_LF_q1_joint__) + ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__)) *  cos__q_LF_q3_joint__)) + ( 0.08853 *  sin__q_LF_q1_joint__));
    (*this)(4,2) = ((((( 0.08853 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.08853 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) + (( 0.15 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__));
    (*this)(4,3) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(4,4) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(4,5) =  sin__q_LF_q1_joint__;
    (*this)(5,0) = (((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (((( 0.125 *  cos__q_LF_q1_joint__) + ( 0.075 *  sin__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(5,1) = (((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(5,2) = ((((((( 0.08853 *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( 0.08853 *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) - ( 0.15 *  cos__q_LF_q2_joint__)) + ( 0.125 *  sin__q_LF_q1_joint__)) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    (*this)(5,3) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(5,4) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_LM_hipassemblyCOM::Type_fr_trunk_X_LM_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_LM_hipassemblyCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_LM_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LM_q1_joint__;
    (*this)(0,1) = - cos__q_LM_q1_joint__;
    (*this)(1,0) =  cos__q_LM_q1_joint__;
    (*this)(1,1) = - sin__q_LM_q1_joint__;
    (*this)(3,2) = (( 0.05821 *  cos__q_LM_q1_joint__) +  0.075);
    (*this)(3,3) = - sin__q_LM_q1_joint__;
    (*this)(3,4) = - cos__q_LM_q1_joint__;
    (*this)(4,2) = ( 0.05821 *  sin__q_LM_q1_joint__);
    (*this)(4,3) =  cos__q_LM_q1_joint__;
    (*this)(4,4) = - sin__q_LM_q1_joint__;
    (*this)(5,0) = ( 0.075 *  sin__q_LM_q1_joint__);
    (*this)(5,1) = (( 0.075 *  cos__q_LM_q1_joint__) +  0.05821);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_LM_upperlegCOM::Type_fr_trunk_X_LM_upperlegCOM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_LM_upperlegCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_LM_upperlegCOM::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(0,1) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(1,0) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(1,1) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(2,0) =  sin__q_LM_q2_joint__;
    (*this)(2,1) =  cos__q_LM_q2_joint__;
    (*this)(3,0) = ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  sin__q_LM_q2_joint__);
    (*this)(3,1) = ((( 0.075 + ( 0.077 *  cos__q_LM_q1_joint__)) *  cos__q_LM_q2_joint__) + ( 0.1213 *  cos__q_LM_q1_joint__));
    (*this)(3,2) = ((- 0.1213 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(3,3) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(3,4) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(3,5) =  cos__q_LM_q1_joint__;
    (*this)(4,0) = (( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(4,1) = ((( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) + ( 0.1213 *  sin__q_LM_q1_joint__));
    (*this)(4,2) = (( 0.1213 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(4,3) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(4,4) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(4,5) =  sin__q_LM_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__);
    (*this)(5,1) = ((- 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(5,2) = (((- 0.1213 *  cos__q_LM_q2_joint__) - ( 0.075 *  cos__q_LM_q1_joint__)) -  0.077);
    (*this)(5,3) =  sin__q_LM_q2_joint__;
    (*this)(5,4) =  cos__q_LM_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_LM_lowerlegCOM::Type_fr_trunk_X_LM_lowerlegCOM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_LM_lowerlegCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_LM_lowerlegCOM::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q3_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,1) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(1,0) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(1,1) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(2,0) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(2,1) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    (*this)(3,0) = (((( 0.15 *  cos__q_LM_q1_joint__) + ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__)) *  sin__q_LM_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,1) = (((((- 0.075 - ( 0.077 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.15 *  cos__q_LM_q1_joint__) + ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__)) *  cos__q_LM_q3_joint__)) + ( 0.08853 *  cos__q_LM_q1_joint__));
    (*this)(3,2) = (((((- 0.08853 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.08853 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) - (( 0.15 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__));
    (*this)(3,3) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,4) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,5) =  cos__q_LM_q1_joint__;
    (*this)(4,0) = (((( 0.15 *  sin__q_LM_q1_joint__) + (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) *  sin__q_LM_q3_joint__) + ((( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(4,1) = (((((- 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.15 *  sin__q_LM_q1_joint__) + (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) *  cos__q_LM_q3_joint__)) + ( 0.08853 *  sin__q_LM_q1_joint__));
    (*this)(4,2) = ((((( 0.08853 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.08853 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) + (( 0.15 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__));
    (*this)(4,3) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(4,4) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(4,5) =  sin__q_LM_q1_joint__;
    (*this)(5,0) = (((( 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - ((( 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(5,1) = ((((- 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(5,2) = (((((( 0.08853 *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( 0.08853 *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) - ( 0.15 *  cos__q_LM_q2_joint__)) - ( 0.075 *  cos__q_LM_q1_joint__)) -  0.077);
    (*this)(5,3) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(5,4) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_LR_hipassemblyCOM::Type_fr_trunk_X_LR_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_LR_hipassemblyCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_LR_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LR_q1_joint__;
    (*this)(0,1) = - cos__q_LR_q1_joint__;
    (*this)(1,0) =  cos__q_LR_q1_joint__;
    (*this)(1,1) = - sin__q_LR_q1_joint__;
    (*this)(3,2) = (( 0.05821 *  cos__q_LR_q1_joint__) +  0.075);
    (*this)(3,3) = - sin__q_LR_q1_joint__;
    (*this)(3,4) = - cos__q_LR_q1_joint__;
    (*this)(4,2) = (( 0.05821 *  sin__q_LR_q1_joint__) +  0.125);
    (*this)(4,3) =  cos__q_LR_q1_joint__;
    (*this)(4,4) = - sin__q_LR_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__));
    (*this)(5,1) = ((( 0.125 *  sin__q_LR_q1_joint__) + ( 0.075 *  cos__q_LR_q1_joint__)) +  0.05821);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_LR_upperlegCOM::Type_fr_trunk_X_LR_upperlegCOM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_LR_upperlegCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_LR_upperlegCOM::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(0,1) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(1,0) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(1,1) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(2,0) =  sin__q_LR_q2_joint__;
    (*this)(2,1) =  cos__q_LR_q2_joint__;
    (*this)(3,0) = ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  sin__q_LR_q2_joint__);
    (*this)(3,1) = ((( 0.075 + ( 0.077 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) + ( 0.1213 *  cos__q_LR_q1_joint__));
    (*this)(3,2) = ((- 0.1213 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__);
    (*this)(3,3) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(3,4) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(3,5) =  cos__q_LR_q1_joint__;
    (*this)(4,0) = ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  sin__q_LR_q2_joint__);
    (*this)(4,1) = ((( 0.125 + ( 0.077 *  sin__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) + ( 0.1213 *  sin__q_LR_q1_joint__));
    (*this)(4,2) = (( 0.1213 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__);
    (*this)(4,3) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(4,4) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(4,5) =  sin__q_LR_q1_joint__;
    (*this)(5,0) = ((( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__);
    (*this)(5,1) = ((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__);
    (*this)(5,2) = ((((- 0.1213 *  cos__q_LR_q2_joint__) - ( 0.125 *  sin__q_LR_q1_joint__)) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    (*this)(5,3) =  sin__q_LR_q2_joint__;
    (*this)(5,4) =  cos__q_LR_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_LR_lowerlegCOM::Type_fr_trunk_X_LR_lowerlegCOM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_LR_lowerlegCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_LR_lowerlegCOM::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q3_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,1) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(1,0) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(1,1) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(2,0) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(2,1) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    (*this)(3,0) = (((( 0.15 *  cos__q_LR_q1_joint__) + ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__)) *  sin__q_LR_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,1) = (((((- 0.075 - ( 0.077 *  cos__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.15 *  cos__q_LR_q1_joint__) + ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__)) *  cos__q_LR_q3_joint__)) + ( 0.08853 *  cos__q_LR_q1_joint__));
    (*this)(3,2) = (((((- 0.08853 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - ((( 0.08853 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) - (( 0.15 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__));
    (*this)(3,3) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,4) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,5) =  cos__q_LR_q1_joint__;
    (*this)(4,0) = (((( 0.15 *  sin__q_LR_q1_joint__) + ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__)) *  sin__q_LR_q3_joint__) + ((( 0.125 + ( 0.077 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(4,1) = (((((- 0.125 - ( 0.077 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.15 *  sin__q_LR_q1_joint__) + ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__)) *  cos__q_LR_q3_joint__)) + ( 0.08853 *  sin__q_LR_q1_joint__));
    (*this)(4,2) = ((((( 0.08853 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.08853 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) + (( 0.15 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__));
    (*this)(4,3) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(4,4) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(4,5) =  sin__q_LR_q1_joint__;
    (*this)(5,0) = ((((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (((( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(5,1) = ((((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(5,2) = ((((((( 0.08853 *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( 0.08853 *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) - ( 0.15 *  cos__q_LR_q2_joint__)) - ( 0.125 *  sin__q_LR_q1_joint__)) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    (*this)(5,3) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(5,4) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_RF_hipassemblyCOM::Type_fr_trunk_X_RF_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_RF_hipassemblyCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_RF_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RF_q1_joint__;
    (*this)(0,1) =  cos__q_RF_q1_joint__;
    (*this)(1,0) = - cos__q_RF_q1_joint__;
    (*this)(1,1) =  sin__q_RF_q1_joint__;
    (*this)(3,2) = ((- 0.05821 *  cos__q_RF_q1_joint__) -  0.075);
    (*this)(3,3) =  sin__q_RF_q1_joint__;
    (*this)(3,4) =  cos__q_RF_q1_joint__;
    (*this)(4,2) = ((- 0.05821 *  sin__q_RF_q1_joint__) -  0.125);
    (*this)(4,3) = - cos__q_RF_q1_joint__;
    (*this)(4,4) =  sin__q_RF_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__));
    (*this)(5,1) = ((( 0.125 *  sin__q_RF_q1_joint__) + ( 0.075 *  cos__q_RF_q1_joint__)) +  0.05821);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_RF_upperlegCOM::Type_fr_trunk_X_RF_upperlegCOM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_RF_upperlegCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_RF_upperlegCOM::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(0,1) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(1,0) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(1,1) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(2,0) =  sin__q_RF_q2_joint__;
    (*this)(2,1) =  cos__q_RF_q2_joint__;
    (*this)(3,0) = (((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  sin__q_RF_q2_joint__);
    (*this)(3,1) = (((- 0.075 - ( 0.077 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) - ( 0.1213 *  cos__q_RF_q1_joint__));
    (*this)(3,2) = (( 0.1213 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__);
    (*this)(3,3) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(3,4) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(3,5) = - cos__q_RF_q1_joint__;
    (*this)(4,0) = (((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  sin__q_RF_q2_joint__);
    (*this)(4,1) = (((- 0.125 - ( 0.077 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) - ( 0.1213 *  sin__q_RF_q1_joint__));
    (*this)(4,2) = ((- 0.1213 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__);
    (*this)(4,3) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(4,4) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(4,5) = - sin__q_RF_q1_joint__;
    (*this)(5,0) = ((( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__);
    (*this)(5,1) = ((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__);
    (*this)(5,2) = ((((- 0.1213 *  cos__q_RF_q2_joint__) - ( 0.125 *  sin__q_RF_q1_joint__)) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    (*this)(5,3) =  sin__q_RF_q2_joint__;
    (*this)(5,4) =  cos__q_RF_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_RF_lowerlegCOM::Type_fr_trunk_X_RF_lowerlegCOM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_RF_lowerlegCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_RF_lowerlegCOM::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q3_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(0,1) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(1,0) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,1) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(2,0) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(2,1) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    (*this)(3,0) = ((((((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(3,1) = ((((( 0.075 + ( 0.077 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q3_joint__)) - ( 0.08853 *  cos__q_RF_q1_joint__));
    (*this)(3,2) = ((((( 0.08853 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + ((( 0.08853 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) + (( 0.15 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__));
    (*this)(3,3) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(3,4) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(3,5) = - cos__q_RF_q1_joint__;
    (*this)(4,0) = ((((((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q3_joint__) + (((- 0.125 - ( 0.077 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,1) = ((((( 0.125 + ( 0.077 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q3_joint__)) - ( 0.08853 *  sin__q_RF_q1_joint__));
    (*this)(4,2) = (((((- 0.08853 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - ((( 0.08853 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) - (( 0.15 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__));
    (*this)(4,3) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,4) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,5) = - sin__q_RF_q1_joint__;
    (*this)(5,0) = ((((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(5,1) = ((((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(5,2) = ((((((( 0.08853 *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( 0.08853 *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) - ( 0.15 *  cos__q_RF_q2_joint__)) - ( 0.125 *  sin__q_RF_q1_joint__)) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    (*this)(5,3) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(5,4) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_RM_hipassemblyCOM::Type_fr_trunk_X_RM_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_RM_hipassemblyCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_RM_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RM_q1_joint__;
    (*this)(0,1) =  cos__q_RM_q1_joint__;
    (*this)(1,0) = - cos__q_RM_q1_joint__;
    (*this)(1,1) =  sin__q_RM_q1_joint__;
    (*this)(3,2) = ((- 0.05821 *  cos__q_RM_q1_joint__) -  0.075);
    (*this)(3,3) =  sin__q_RM_q1_joint__;
    (*this)(3,4) =  cos__q_RM_q1_joint__;
    (*this)(4,2) = (- 0.05821 *  sin__q_RM_q1_joint__);
    (*this)(4,3) = - cos__q_RM_q1_joint__;
    (*this)(4,4) =  sin__q_RM_q1_joint__;
    (*this)(5,0) = ( 0.075 *  sin__q_RM_q1_joint__);
    (*this)(5,1) = (( 0.075 *  cos__q_RM_q1_joint__) +  0.05821);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_RM_upperlegCOM::Type_fr_trunk_X_RM_upperlegCOM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_RM_upperlegCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_RM_upperlegCOM::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(0,1) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(1,0) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(1,1) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(2,0) =  sin__q_RM_q2_joint__;
    (*this)(2,1) =  cos__q_RM_q2_joint__;
    (*this)(3,0) = (((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  sin__q_RM_q2_joint__);
    (*this)(3,1) = (((- 0.075 - ( 0.077 *  cos__q_RM_q1_joint__)) *  cos__q_RM_q2_joint__) - ( 0.1213 *  cos__q_RM_q1_joint__));
    (*this)(3,2) = (( 0.1213 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(3,3) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(3,4) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(3,5) = - cos__q_RM_q1_joint__;
    (*this)(4,0) = ((- 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(4,1) = (((- 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) - ( 0.1213 *  sin__q_RM_q1_joint__));
    (*this)(4,2) = ((- 0.1213 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(4,3) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(4,4) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(4,5) = - sin__q_RM_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__);
    (*this)(5,1) = ((- 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(5,2) = (((- 0.1213 *  cos__q_RM_q2_joint__) - ( 0.075 *  cos__q_RM_q1_joint__)) -  0.077);
    (*this)(5,3) =  sin__q_RM_q2_joint__;
    (*this)(5,4) =  cos__q_RM_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_RM_lowerlegCOM::Type_fr_trunk_X_RM_lowerlegCOM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_RM_lowerlegCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_RM_lowerlegCOM::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q3_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(0,1) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(1,0) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,1) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(2,0) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(2,1) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    (*this)(3,0) = ((((((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(3,1) = ((((( 0.075 + ( 0.077 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (((((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__)) *  cos__q_RM_q3_joint__)) - ( 0.08853 *  cos__q_RM_q1_joint__));
    (*this)(3,2) = ((((( 0.08853 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + ((( 0.08853 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) + (( 0.15 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__));
    (*this)(3,3) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(3,4) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(3,5) = - cos__q_RM_q1_joint__;
    (*this)(4,0) = ((((- 0.15 *  sin__q_RM_q1_joint__) - (( 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) *  sin__q_RM_q3_joint__) - ((( 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,1) = ((((( 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (((- 0.15 *  sin__q_RM_q1_joint__) - (( 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) *  cos__q_RM_q3_joint__)) - ( 0.08853 *  sin__q_RM_q1_joint__));
    (*this)(4,2) = (((((- 0.08853 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.08853 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) - (( 0.15 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__));
    (*this)(4,3) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,4) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,5) = - sin__q_RM_q1_joint__;
    (*this)(5,0) = (((( 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - ((( 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(5,1) = ((((- 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(5,2) = (((((( 0.08853 *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( 0.08853 *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) - ( 0.15 *  cos__q_RM_q2_joint__)) - ( 0.075 *  cos__q_RM_q1_joint__)) -  0.077);
    (*this)(5,3) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(5,4) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_RR_hipassemblyCOM::Type_fr_trunk_X_RR_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_RR_hipassemblyCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_RR_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RR_q1_joint__;
    (*this)(0,1) =  cos__q_RR_q1_joint__;
    (*this)(1,0) = - cos__q_RR_q1_joint__;
    (*this)(1,1) =  sin__q_RR_q1_joint__;
    (*this)(3,2) = ((- 0.05821 *  cos__q_RR_q1_joint__) -  0.075);
    (*this)(3,3) =  sin__q_RR_q1_joint__;
    (*this)(3,4) =  cos__q_RR_q1_joint__;
    (*this)(4,2) = ( 0.125 - ( 0.05821 *  sin__q_RR_q1_joint__));
    (*this)(4,3) = - cos__q_RR_q1_joint__;
    (*this)(4,4) =  sin__q_RR_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__));
    (*this)(5,1) = (((- 0.125 *  sin__q_RR_q1_joint__) + ( 0.075 *  cos__q_RR_q1_joint__)) +  0.05821);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_RR_upperlegCOM::Type_fr_trunk_X_RR_upperlegCOM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_RR_upperlegCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_RR_upperlegCOM::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(0,1) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(1,0) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(1,1) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(2,0) =  sin__q_RR_q2_joint__;
    (*this)(2,1) =  cos__q_RR_q2_joint__;
    (*this)(3,0) = (((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  sin__q_RR_q2_joint__);
    (*this)(3,1) = (((- 0.075 - ( 0.077 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.1213 *  cos__q_RR_q1_joint__));
    (*this)(3,2) = (( 0.1213 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__);
    (*this)(3,3) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(3,4) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(3,5) = - cos__q_RR_q1_joint__;
    (*this)(4,0) = (( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__);
    (*this)(4,1) = ((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.1213 *  sin__q_RR_q1_joint__));
    (*this)(4,2) = ((- 0.1213 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__);
    (*this)(4,3) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(4,4) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(4,5) = - sin__q_RR_q1_joint__;
    (*this)(5,0) = ((( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__);
    (*this)(5,1) = (((- 0.075 *  sin__q_RR_q1_joint__) - ( 0.125 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__);
    (*this)(5,2) = ((((- 0.1213 *  cos__q_RR_q2_joint__) + ( 0.125 *  sin__q_RR_q1_joint__)) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    (*this)(5,3) =  sin__q_RR_q2_joint__;
    (*this)(5,4) =  cos__q_RR_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_RR_lowerlegCOM::Type_fr_trunk_X_RR_lowerlegCOM()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_RR_lowerlegCOM& iit::mcorin::MotionTransforms::Type_fr_trunk_X_RR_lowerlegCOM::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q3_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(0,1) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(1,0) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,1) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(2,0) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(2,1) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    (*this)(3,0) = ((((((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(3,1) = ((((( 0.075 + ( 0.077 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q3_joint__)) - ( 0.08853 *  cos__q_RR_q1_joint__));
    (*this)(3,2) = ((((( 0.08853 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((( 0.08853 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) + (( 0.15 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__));
    (*this)(3,3) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(3,4) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(3,5) = - cos__q_RR_q1_joint__;
    (*this)(4,0) = ((((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q3_joint__) + ((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,1) = (((((( 0.077 *  sin__q_RR_q1_joint__) -  0.125) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q3_joint__)) - ( 0.08853 *  sin__q_RR_q1_joint__));
    (*this)(4,2) = (((((- 0.08853 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - ((( 0.08853 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) - (( 0.15 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__));
    (*this)(4,3) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,4) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,5) = - sin__q_RR_q1_joint__;
    (*this)(5,0) = (((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((( 0.125 *  cos__q_RR_q1_joint__) + ( 0.075 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(5,1) = (((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(5,2) = ((((((( 0.08853 *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( 0.08853 *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) - ( 0.15 *  cos__q_RR_q2_joint__)) + ( 0.125 *  sin__q_RR_q1_joint__)) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    (*this)(5,3) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(5,4) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LF_upperleg_X_fr_trunk::Type_fr_LF_upperleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_LF_upperleg_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_LF_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(0,1) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(0,2) =  sin__q_LF_q2_joint__;
    (*this)(1,0) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(1,1) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(1,2) =  cos__q_LF_q2_joint__;
    (*this)(2,0) =  cos__q_LF_q1_joint__;
    (*this)(2,1) =  sin__q_LF_q1_joint__;
    (*this)(3,0) = ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  sin__q_LF_q2_joint__);
    (*this)(3,1) = ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  sin__q_LF_q2_joint__);
    (*this)(3,2) = ((( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__);
    (*this)(3,3) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(3,4) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(3,5) =  sin__q_LF_q2_joint__;
    (*this)(4,0) = ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__);
    (*this)(4,1) = ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__);
    (*this)(4,2) = (((- 0.075 *  sin__q_LF_q1_joint__) - ( 0.125 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__);
    (*this)(4,3) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(4,4) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(4,5) =  cos__q_LF_q2_joint__;
    (*this)(5,2) = ((( 0.125 *  sin__q_LF_q1_joint__) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    (*this)(5,3) =  cos__q_LF_q1_joint__;
    (*this)(5,4) =  sin__q_LF_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LF_lowerleg_X_fr_trunk::Type_fr_LF_lowerleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_LF_lowerleg_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_LF_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q3_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,1) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(0,2) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(1,0) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,1) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,2) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    (*this)(2,0) =  cos__q_LF_q1_joint__;
    (*this)(2,1) =  sin__q_LF_q1_joint__;
    (*this)(3,0) = (((( 0.15 *  cos__q_LF_q1_joint__) + ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__)) *  sin__q_LF_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,1) = (((( 0.15 *  sin__q_LF_q1_joint__) + ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__)) *  sin__q_LF_q3_joint__) + (((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,2) = (((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (((( 0.125 *  cos__q_LF_q1_joint__) + ( 0.075 *  sin__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,3) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,4) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(3,5) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(4,0) = ((((- 0.075 - ( 0.077 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.15 *  cos__q_LF_q1_joint__) + ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__)) *  cos__q_LF_q3_joint__));
    (*this)(4,1) = (((( 0.125 - ( 0.077 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.15 *  sin__q_LF_q1_joint__) + ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__)) *  cos__q_LF_q3_joint__));
    (*this)(4,2) = (((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(4,3) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(4,4) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(4,5) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    (*this)(5,0) = ((- 0.15 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__);
    (*this)(5,1) = (( 0.15 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__);
    (*this)(5,2) = ((((- 0.15 *  cos__q_LF_q2_joint__) + ( 0.125 *  sin__q_LF_q1_joint__)) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    (*this)(5,3) =  cos__q_LF_q1_joint__;
    (*this)(5,4) =  sin__q_LF_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LM_upperleg_X_fr_trunk::Type_fr_LM_upperleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_LM_upperleg_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_LM_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(0,1) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(0,2) =  sin__q_LM_q2_joint__;
    (*this)(1,0) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(1,1) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(1,2) =  cos__q_LM_q2_joint__;
    (*this)(2,0) =  cos__q_LM_q1_joint__;
    (*this)(2,1) =  sin__q_LM_q1_joint__;
    (*this)(3,0) = ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  sin__q_LM_q2_joint__);
    (*this)(3,1) = (( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(3,2) = (( 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__);
    (*this)(3,3) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(3,4) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(3,5) =  sin__q_LM_q2_joint__;
    (*this)(4,0) = ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__);
    (*this)(4,1) = (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__);
    (*this)(4,2) = ((- 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(4,3) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(4,4) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(4,5) =  cos__q_LM_q2_joint__;
    (*this)(5,2) = ((- 0.075 *  cos__q_LM_q1_joint__) -  0.077);
    (*this)(5,3) =  cos__q_LM_q1_joint__;
    (*this)(5,4) =  sin__q_LM_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LM_lowerleg_X_fr_trunk::Type_fr_LM_lowerleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_LM_lowerleg_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_LM_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q3_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,1) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(0,2) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(1,0) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,1) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,2) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    (*this)(2,0) =  cos__q_LM_q1_joint__;
    (*this)(2,1) =  sin__q_LM_q1_joint__;
    (*this)(3,0) = (((( 0.15 *  cos__q_LM_q1_joint__) + ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__)) *  sin__q_LM_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,1) = (((( 0.15 *  sin__q_LM_q1_joint__) + (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) *  sin__q_LM_q3_joint__) + ((( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,2) = (((( 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - ((( 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(3,3) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,4) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(3,5) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(4,0) = ((((- 0.075 - ( 0.077 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.15 *  cos__q_LM_q1_joint__) + ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__)) *  cos__q_LM_q3_joint__));
    (*this)(4,1) = (((( 0.15 *  sin__q_LM_q1_joint__) + (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) *  cos__q_LM_q3_joint__) - ((( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(4,2) = ((((- 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(4,3) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(4,4) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(4,5) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    (*this)(5,0) = ((- 0.15 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(5,1) = (( 0.15 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(5,2) = (((- 0.15 *  cos__q_LM_q2_joint__) - ( 0.075 *  cos__q_LM_q1_joint__)) -  0.077);
    (*this)(5,3) =  cos__q_LM_q1_joint__;
    (*this)(5,4) =  sin__q_LM_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LR_upperleg_X_fr_trunk::Type_fr_LR_upperleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_LR_upperleg_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_LR_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(0,1) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(0,2) =  sin__q_LR_q2_joint__;
    (*this)(1,0) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(1,1) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(1,2) =  cos__q_LR_q2_joint__;
    (*this)(2,0) =  cos__q_LR_q1_joint__;
    (*this)(2,1) =  sin__q_LR_q1_joint__;
    (*this)(3,0) = ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  sin__q_LR_q2_joint__);
    (*this)(3,1) = ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  sin__q_LR_q2_joint__);
    (*this)(3,2) = ((( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__);
    (*this)(3,3) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(3,4) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(3,5) =  sin__q_LR_q2_joint__;
    (*this)(4,0) = ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__);
    (*this)(4,1) = ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__);
    (*this)(4,2) = ((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__);
    (*this)(4,3) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(4,4) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(4,5) =  cos__q_LR_q2_joint__;
    (*this)(5,2) = (((- 0.125 *  sin__q_LR_q1_joint__) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    (*this)(5,3) =  cos__q_LR_q1_joint__;
    (*this)(5,4) =  sin__q_LR_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LR_lowerleg_X_fr_trunk::Type_fr_LR_lowerleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_LR_lowerleg_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_LR_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q3_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,1) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(0,2) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(1,0) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,1) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,2) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    (*this)(2,0) =  cos__q_LR_q1_joint__;
    (*this)(2,1) =  sin__q_LR_q1_joint__;
    (*this)(3,0) = (((( 0.15 *  cos__q_LR_q1_joint__) + ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__)) *  sin__q_LR_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,1) = (((( 0.15 *  sin__q_LR_q1_joint__) + ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__)) *  sin__q_LR_q3_joint__) + ((( 0.125 + ( 0.077 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,2) = ((((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (((( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,3) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,4) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(3,5) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(4,0) = ((((- 0.075 - ( 0.077 *  cos__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.15 *  cos__q_LR_q1_joint__) + ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__)) *  cos__q_LR_q3_joint__));
    (*this)(4,1) = ((((- 0.125 - ( 0.077 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.15 *  sin__q_LR_q1_joint__) + ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__)) *  cos__q_LR_q3_joint__));
    (*this)(4,2) = ((((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(4,3) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(4,4) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(4,5) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    (*this)(5,0) = ((- 0.15 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__);
    (*this)(5,1) = (( 0.15 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__);
    (*this)(5,2) = ((((- 0.15 *  cos__q_LR_q2_joint__) - ( 0.125 *  sin__q_LR_q1_joint__)) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    (*this)(5,3) =  cos__q_LR_q1_joint__;
    (*this)(5,4) =  sin__q_LR_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RF_upperleg_X_fr_trunk::Type_fr_RF_upperleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_RF_upperleg_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_RF_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(0,1) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(0,2) =  sin__q_RF_q2_joint__;
    (*this)(1,0) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(1,1) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(1,2) =  cos__q_RF_q2_joint__;
    (*this)(2,0) = - cos__q_RF_q1_joint__;
    (*this)(2,1) = - sin__q_RF_q1_joint__;
    (*this)(3,0) = (((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  sin__q_RF_q2_joint__);
    (*this)(3,1) = (((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  sin__q_RF_q2_joint__);
    (*this)(3,2) = ((( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__);
    (*this)(3,3) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(3,4) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(3,5) =  sin__q_RF_q2_joint__;
    (*this)(4,0) = (((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__);
    (*this)(4,1) = (((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__);
    (*this)(4,2) = ((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__);
    (*this)(4,3) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(4,4) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(4,5) =  cos__q_RF_q2_joint__;
    (*this)(5,2) = (((- 0.125 *  sin__q_RF_q1_joint__) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    (*this)(5,3) = - cos__q_RF_q1_joint__;
    (*this)(5,4) = - sin__q_RF_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RF_lowerleg_X_fr_trunk::Type_fr_RF_lowerleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_RF_lowerleg_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_RF_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q3_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(0,1) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(0,2) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(1,0) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,1) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,2) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    (*this)(2,0) = - cos__q_RF_q1_joint__;
    (*this)(2,1) = - sin__q_RF_q1_joint__;
    (*this)(3,0) = ((((((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(3,1) = ((((((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q3_joint__) + (((- 0.125 - ( 0.077 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(3,2) = ((((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(3,3) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(3,4) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(3,5) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(4,0) = (((( 0.075 + ( 0.077 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q3_joint__));
    (*this)(4,1) = (((( 0.125 + ( 0.077 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q3_joint__));
    (*this)(4,2) = ((((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,3) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,4) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,5) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    (*this)(5,0) = (( 0.15 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__);
    (*this)(5,1) = ((- 0.15 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__);
    (*this)(5,2) = ((((- 0.15 *  cos__q_RF_q2_joint__) - ( 0.125 *  sin__q_RF_q1_joint__)) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    (*this)(5,3) = - cos__q_RF_q1_joint__;
    (*this)(5,4) = - sin__q_RF_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RM_upperleg_X_fr_trunk::Type_fr_RM_upperleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_RM_upperleg_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_RM_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(0,1) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(0,2) =  sin__q_RM_q2_joint__;
    (*this)(1,0) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(1,1) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(1,2) =  cos__q_RM_q2_joint__;
    (*this)(2,0) = - cos__q_RM_q1_joint__;
    (*this)(2,1) = - sin__q_RM_q1_joint__;
    (*this)(3,0) = (((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  sin__q_RM_q2_joint__);
    (*this)(3,1) = ((- 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(3,2) = (( 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__);
    (*this)(3,3) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(3,4) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(3,5) =  sin__q_RM_q2_joint__;
    (*this)(4,0) = (((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__);
    (*this)(4,1) = ((- 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__);
    (*this)(4,2) = ((- 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(4,3) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(4,4) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(4,5) =  cos__q_RM_q2_joint__;
    (*this)(5,2) = ((- 0.075 *  cos__q_RM_q1_joint__) -  0.077);
    (*this)(5,3) = - cos__q_RM_q1_joint__;
    (*this)(5,4) = - sin__q_RM_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RM_lowerleg_X_fr_trunk::Type_fr_RM_lowerleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_RM_lowerleg_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_RM_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q3_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(0,1) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(0,2) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(1,0) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,1) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,2) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    (*this)(2,0) = - cos__q_RM_q1_joint__;
    (*this)(2,1) = - sin__q_RM_q1_joint__;
    (*this)(3,0) = ((((((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(3,1) = ((((- 0.15 *  sin__q_RM_q1_joint__) - (( 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) *  sin__q_RM_q3_joint__) - ((( 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(3,2) = (((( 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - ((( 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(3,3) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(3,4) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(3,5) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(4,0) = (((( 0.075 + ( 0.077 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (((((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__)) *  cos__q_RM_q3_joint__));
    (*this)(4,1) = (((( 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (((- 0.15 *  sin__q_RM_q1_joint__) - (( 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) *  cos__q_RM_q3_joint__));
    (*this)(4,2) = ((((- 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,3) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,4) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,5) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    (*this)(5,0) = (( 0.15 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(5,1) = ((- 0.15 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(5,2) = (((- 0.15 *  cos__q_RM_q2_joint__) - ( 0.075 *  cos__q_RM_q1_joint__)) -  0.077);
    (*this)(5,3) = - cos__q_RM_q1_joint__;
    (*this)(5,4) = - sin__q_RM_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RR_upperleg_X_fr_trunk::Type_fr_RR_upperleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_RR_upperleg_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_RR_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(0,1) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(0,2) =  sin__q_RR_q2_joint__;
    (*this)(1,0) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(1,1) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(1,2) =  cos__q_RR_q2_joint__;
    (*this)(2,0) = - cos__q_RR_q1_joint__;
    (*this)(2,1) = - sin__q_RR_q1_joint__;
    (*this)(3,0) = (((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  sin__q_RR_q2_joint__);
    (*this)(3,1) = (( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__);
    (*this)(3,2) = ((( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__);
    (*this)(3,3) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(3,4) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(3,5) =  sin__q_RR_q2_joint__;
    (*this)(4,0) = (((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__);
    (*this)(4,1) = (( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__);
    (*this)(4,2) = (((- 0.075 *  sin__q_RR_q1_joint__) - ( 0.125 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__);
    (*this)(4,3) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(4,4) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(4,5) =  cos__q_RR_q2_joint__;
    (*this)(5,2) = ((( 0.125 *  sin__q_RR_q1_joint__) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    (*this)(5,3) = - cos__q_RR_q1_joint__;
    (*this)(5,4) = - sin__q_RR_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RR_lowerleg_X_fr_trunk::Type_fr_RR_lowerleg_X_fr_trunk()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_RR_lowerleg_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_RR_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q3_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(0,1) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(0,2) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(1,0) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,1) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,2) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    (*this)(2,0) = - cos__q_RR_q1_joint__;
    (*this)(2,1) = - sin__q_RR_q1_joint__;
    (*this)(3,0) = ((((((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(3,1) = ((((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q3_joint__) + ((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(3,2) = (((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((( 0.125 *  cos__q_RR_q1_joint__) + ( 0.075 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(3,3) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(3,4) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(3,5) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(4,0) = (((( 0.075 + ( 0.077 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q3_joint__));
    (*this)(4,1) = ((((( 0.077 *  sin__q_RR_q1_joint__) -  0.125) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q3_joint__));
    (*this)(4,2) = (((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,3) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,4) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,5) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    (*this)(5,0) = (( 0.15 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__);
    (*this)(5,1) = ((- 0.15 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__);
    (*this)(5,2) = ((((- 0.15 *  cos__q_RR_q2_joint__) + ( 0.125 *  sin__q_RR_q1_joint__)) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    (*this)(5,3) = - cos__q_RR_q1_joint__;
    (*this)(5,4) = - sin__q_RR_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_LF_foot::Type_fr_trunk_X_LF_foot()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_LF_foot& iit::mcorin::MotionTransforms::Type_fr_trunk_X_LF_foot::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q3_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,1) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(1,0) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(1,1) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(2,0) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(2,1) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    (*this)(3,0) = (((( 0.15 *  cos__q_LF_q1_joint__) + ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__)) *  sin__q_LF_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,1) = (((((- 0.075 - ( 0.077 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.15 *  cos__q_LF_q1_joint__) + ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__)) *  cos__q_LF_q3_joint__)) + ( 0.17 *  cos__q_LF_q1_joint__));
    (*this)(3,2) = (((((- 0.17 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - ((( 0.17 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) - (( 0.15 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__));
    (*this)(3,3) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,4) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,5) =  cos__q_LF_q1_joint__;
    (*this)(4,0) = (((( 0.15 *  sin__q_LF_q1_joint__) + ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__)) *  sin__q_LF_q3_joint__) + (((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(4,1) = ((((( 0.125 - ( 0.077 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.15 *  sin__q_LF_q1_joint__) + ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__)) *  cos__q_LF_q3_joint__)) + ( 0.17 *  sin__q_LF_q1_joint__));
    (*this)(4,2) = ((((( 0.17 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.17 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) + (( 0.15 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__));
    (*this)(4,3) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(4,4) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(4,5) =  sin__q_LF_q1_joint__;
    (*this)(5,0) = (((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (((( 0.125 *  cos__q_LF_q1_joint__) + ( 0.075 *  sin__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(5,1) = (((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(5,2) = ((((((( 0.17 *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( 0.17 *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) - ( 0.15 *  cos__q_LF_q2_joint__)) + ( 0.125 *  sin__q_LF_q1_joint__)) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    (*this)(5,3) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(5,4) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LF_q1_joint::Type_fr_trunk_X_fr_LF_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0.075;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.125;
    (*this)(4,3) = 1.0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0.125;
    (*this)(5,1) = 0.075;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LF_q1_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LF_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LF_q2_joint::Type_fr_trunk_X_fr_LF_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,1) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LF_q2_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LF_q2_joint::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LF_q1_joint__;
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(1,0) =  cos__q_LF_q1_joint__;
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(3,1) = (( 0.077 *  cos__q_LF_q1_joint__) +  0.075);
    (*this)(3,3) = - sin__q_LF_q1_joint__;
    (*this)(3,5) =  cos__q_LF_q1_joint__;
    (*this)(4,1) = (( 0.077 *  sin__q_LF_q1_joint__) -  0.125);
    (*this)(4,3) =  cos__q_LF_q1_joint__;
    (*this)(4,5) =  sin__q_LF_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__));
    (*this)(5,2) = ((( 0.125 *  sin__q_LF_q1_joint__) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LF_q3_joint::Type_fr_trunk_X_fr_LF_q3_joint()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LF_q3_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LF_q3_joint::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(0,1) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(1,0) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(1,1) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(2,0) =  sin__q_LF_q2_joint__;
    (*this)(2,1) =  cos__q_LF_q2_joint__;
    (*this)(3,0) = ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  sin__q_LF_q2_joint__);
    (*this)(3,1) = ((( 0.075 + ( 0.077 *  cos__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) + ( 0.15 *  cos__q_LF_q1_joint__));
    (*this)(3,2) = ((- 0.15 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__);
    (*this)(3,3) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(3,4) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(3,5) =  cos__q_LF_q1_joint__;
    (*this)(4,0) = ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  sin__q_LF_q2_joint__);
    (*this)(4,1) = (((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__) + ( 0.15 *  sin__q_LF_q1_joint__));
    (*this)(4,2) = (( 0.15 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__);
    (*this)(4,3) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(4,4) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(4,5) =  sin__q_LF_q1_joint__;
    (*this)(5,0) = ((( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__);
    (*this)(5,1) = (((- 0.075 *  sin__q_LF_q1_joint__) - ( 0.125 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__);
    (*this)(5,2) = ((((- 0.15 *  cos__q_LF_q2_joint__) + ( 0.125 *  sin__q_LF_q1_joint__)) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    (*this)(5,3) =  sin__q_LF_q2_joint__;
    (*this)(5,4) =  cos__q_LF_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_LM_foot::Type_fr_trunk_X_LM_foot()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_LM_foot& iit::mcorin::MotionTransforms::Type_fr_trunk_X_LM_foot::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q3_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,1) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(1,0) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(1,1) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(2,0) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(2,1) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    (*this)(3,0) = (((( 0.15 *  cos__q_LM_q1_joint__) + ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__)) *  sin__q_LM_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,1) = (((((- 0.075 - ( 0.077 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.15 *  cos__q_LM_q1_joint__) + ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__)) *  cos__q_LM_q3_joint__)) + ( 0.17 *  cos__q_LM_q1_joint__));
    (*this)(3,2) = (((((- 0.17 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.17 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) - (( 0.15 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__));
    (*this)(3,3) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,4) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,5) =  cos__q_LM_q1_joint__;
    (*this)(4,0) = (((( 0.15 *  sin__q_LM_q1_joint__) + (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) *  sin__q_LM_q3_joint__) + ((( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(4,1) = (((((- 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.15 *  sin__q_LM_q1_joint__) + (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) *  cos__q_LM_q3_joint__)) + ( 0.17 *  sin__q_LM_q1_joint__));
    (*this)(4,2) = ((((( 0.17 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.17 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) + (( 0.15 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__));
    (*this)(4,3) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(4,4) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(4,5) =  sin__q_LM_q1_joint__;
    (*this)(5,0) = (((( 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - ((( 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(5,1) = ((((- 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(5,2) = (((((( 0.17 *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( 0.17 *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) - ( 0.15 *  cos__q_LM_q2_joint__)) - ( 0.075 *  cos__q_LM_q1_joint__)) -  0.077);
    (*this)(5,3) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(5,4) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LM_q1_joint::Type_fr_trunk_X_fr_LM_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 1;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0.075;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 1;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0.075;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LM_q1_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LM_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LM_q2_joint::Type_fr_trunk_X_fr_LM_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,1) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LM_q2_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LM_q2_joint::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LM_q1_joint__;
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(1,0) =  cos__q_LM_q1_joint__;
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(3,1) = (( 0.077 *  cos__q_LM_q1_joint__) +  0.075);
    (*this)(3,3) = - sin__q_LM_q1_joint__;
    (*this)(3,5) =  cos__q_LM_q1_joint__;
    (*this)(4,1) = ( 0.077 *  sin__q_LM_q1_joint__);
    (*this)(4,3) =  cos__q_LM_q1_joint__;
    (*this)(4,5) =  sin__q_LM_q1_joint__;
    (*this)(5,0) = ( 0.075 *  sin__q_LM_q1_joint__);
    (*this)(5,2) = ((- 0.075 *  cos__q_LM_q1_joint__) -  0.077);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LM_q3_joint::Type_fr_trunk_X_fr_LM_q3_joint()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LM_q3_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LM_q3_joint::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(0,1) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(1,0) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(1,1) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(2,0) =  sin__q_LM_q2_joint__;
    (*this)(2,1) =  cos__q_LM_q2_joint__;
    (*this)(3,0) = ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  sin__q_LM_q2_joint__);
    (*this)(3,1) = ((( 0.075 + ( 0.077 *  cos__q_LM_q1_joint__)) *  cos__q_LM_q2_joint__) + ( 0.15 *  cos__q_LM_q1_joint__));
    (*this)(3,2) = ((- 0.15 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(3,3) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(3,4) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(3,5) =  cos__q_LM_q1_joint__;
    (*this)(4,0) = (( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(4,1) = ((( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) + ( 0.15 *  sin__q_LM_q1_joint__));
    (*this)(4,2) = (( 0.15 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(4,3) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(4,4) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(4,5) =  sin__q_LM_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__);
    (*this)(5,1) = ((- 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(5,2) = (((- 0.15 *  cos__q_LM_q2_joint__) - ( 0.075 *  cos__q_LM_q1_joint__)) -  0.077);
    (*this)(5,3) =  sin__q_LM_q2_joint__;
    (*this)(5,4) =  cos__q_LM_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_LR_foot::Type_fr_trunk_X_LR_foot()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_LR_foot& iit::mcorin::MotionTransforms::Type_fr_trunk_X_LR_foot::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q3_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,1) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(1,0) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(1,1) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(2,0) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(2,1) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    (*this)(3,0) = (((( 0.15 *  cos__q_LR_q1_joint__) + ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__)) *  sin__q_LR_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,1) = (((((- 0.075 - ( 0.077 *  cos__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.15 *  cos__q_LR_q1_joint__) + ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__)) *  cos__q_LR_q3_joint__)) + ( 0.17 *  cos__q_LR_q1_joint__));
    (*this)(3,2) = (((((- 0.17 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - ((( 0.17 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) - (( 0.15 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__));
    (*this)(3,3) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,4) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,5) =  cos__q_LR_q1_joint__;
    (*this)(4,0) = (((( 0.15 *  sin__q_LR_q1_joint__) + ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__)) *  sin__q_LR_q3_joint__) + ((( 0.125 + ( 0.077 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(4,1) = (((((- 0.125 - ( 0.077 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.15 *  sin__q_LR_q1_joint__) + ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__)) *  cos__q_LR_q3_joint__)) + ( 0.17 *  sin__q_LR_q1_joint__));
    (*this)(4,2) = ((((( 0.17 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.17 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) + (( 0.15 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__));
    (*this)(4,3) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(4,4) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(4,5) =  sin__q_LR_q1_joint__;
    (*this)(5,0) = ((((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (((( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(5,1) = ((((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(5,2) = ((((((( 0.17 *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( 0.17 *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) - ( 0.15 *  cos__q_LR_q2_joint__)) - ( 0.125 *  sin__q_LR_q1_joint__)) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    (*this)(5,3) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(5,4) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LR_q1_joint::Type_fr_trunk_X_fr_LR_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0.075;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0.125;
    (*this)(4,3) = 1.0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = - 0.125;
    (*this)(5,1) = 0.075;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LR_q1_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LR_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LR_q2_joint::Type_fr_trunk_X_fr_LR_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,1) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LR_q2_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LR_q2_joint::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LR_q1_joint__;
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(1,0) =  cos__q_LR_q1_joint__;
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(3,1) = (( 0.077 *  cos__q_LR_q1_joint__) +  0.075);
    (*this)(3,3) = - sin__q_LR_q1_joint__;
    (*this)(3,5) =  cos__q_LR_q1_joint__;
    (*this)(4,1) = (( 0.077 *  sin__q_LR_q1_joint__) +  0.125);
    (*this)(4,3) =  cos__q_LR_q1_joint__;
    (*this)(4,5) =  sin__q_LR_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__));
    (*this)(5,2) = (((- 0.125 *  sin__q_LR_q1_joint__) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LR_q3_joint::Type_fr_trunk_X_fr_LR_q3_joint()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LR_q3_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LR_q3_joint::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(0,1) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(1,0) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(1,1) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(2,0) =  sin__q_LR_q2_joint__;
    (*this)(2,1) =  cos__q_LR_q2_joint__;
    (*this)(3,0) = ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  sin__q_LR_q2_joint__);
    (*this)(3,1) = ((( 0.075 + ( 0.077 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) + ( 0.15 *  cos__q_LR_q1_joint__));
    (*this)(3,2) = ((- 0.15 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__);
    (*this)(3,3) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(3,4) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(3,5) =  cos__q_LR_q1_joint__;
    (*this)(4,0) = ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  sin__q_LR_q2_joint__);
    (*this)(4,1) = ((( 0.125 + ( 0.077 *  sin__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) + ( 0.15 *  sin__q_LR_q1_joint__));
    (*this)(4,2) = (( 0.15 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__);
    (*this)(4,3) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(4,4) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(4,5) =  sin__q_LR_q1_joint__;
    (*this)(5,0) = ((( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__);
    (*this)(5,1) = ((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__);
    (*this)(5,2) = ((((- 0.15 *  cos__q_LR_q2_joint__) - ( 0.125 *  sin__q_LR_q1_joint__)) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    (*this)(5,3) =  sin__q_LR_q2_joint__;
    (*this)(5,4) =  cos__q_LR_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_RF_foot::Type_fr_trunk_X_RF_foot()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_RF_foot& iit::mcorin::MotionTransforms::Type_fr_trunk_X_RF_foot::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q3_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(0,1) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(1,0) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,1) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(2,0) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(2,1) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    (*this)(3,0) = ((((((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(3,1) = ((((( 0.075 + ( 0.077 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q3_joint__)) - ( 0.17 *  cos__q_RF_q1_joint__));
    (*this)(3,2) = ((((( 0.17 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + ((( 0.17 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) + (( 0.15 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__));
    (*this)(3,3) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(3,4) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(3,5) = - cos__q_RF_q1_joint__;
    (*this)(4,0) = ((((((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q3_joint__) + (((- 0.125 - ( 0.077 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,1) = ((((( 0.125 + ( 0.077 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q3_joint__)) - ( 0.17 *  sin__q_RF_q1_joint__));
    (*this)(4,2) = (((((- 0.17 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - ((( 0.17 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) - (( 0.15 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__));
    (*this)(4,3) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,4) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,5) = - sin__q_RF_q1_joint__;
    (*this)(5,0) = ((((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(5,1) = ((((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(5,2) = ((((((( 0.17 *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( 0.17 *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) - ( 0.15 *  cos__q_RF_q2_joint__)) - ( 0.125 *  sin__q_RF_q1_joint__)) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    (*this)(5,3) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(5,4) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RF_q1_joint::Type_fr_trunk_X_fr_RF_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = - 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = - 0.075;
    (*this)(3,3) = 0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.125;
    (*this)(4,3) = - 1.0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = - 0.125;
    (*this)(5,1) = 0.075;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RF_q1_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RF_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RF_q2_joint::Type_fr_trunk_X_fr_RF_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,1) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RF_q2_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RF_q2_joint::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RF_q1_joint__;
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(1,0) = - cos__q_RF_q1_joint__;
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(3,1) = ((- 0.077 *  cos__q_RF_q1_joint__) -  0.075);
    (*this)(3,3) =  sin__q_RF_q1_joint__;
    (*this)(3,5) = - cos__q_RF_q1_joint__;
    (*this)(4,1) = ((- 0.077 *  sin__q_RF_q1_joint__) -  0.125);
    (*this)(4,3) = - cos__q_RF_q1_joint__;
    (*this)(4,5) = - sin__q_RF_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__));
    (*this)(5,2) = (((- 0.125 *  sin__q_RF_q1_joint__) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RF_q3_joint::Type_fr_trunk_X_fr_RF_q3_joint()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RF_q3_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RF_q3_joint::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(0,1) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(1,0) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(1,1) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(2,0) =  sin__q_RF_q2_joint__;
    (*this)(2,1) =  cos__q_RF_q2_joint__;
    (*this)(3,0) = (((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  sin__q_RF_q2_joint__);
    (*this)(3,1) = (((- 0.075 - ( 0.077 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__));
    (*this)(3,2) = (( 0.15 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__);
    (*this)(3,3) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(3,4) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(3,5) = - cos__q_RF_q1_joint__;
    (*this)(4,0) = (((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  sin__q_RF_q2_joint__);
    (*this)(4,1) = (((- 0.125 - ( 0.077 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__));
    (*this)(4,2) = ((- 0.15 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__);
    (*this)(4,3) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(4,4) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(4,5) = - sin__q_RF_q1_joint__;
    (*this)(5,0) = ((( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__);
    (*this)(5,1) = ((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__);
    (*this)(5,2) = ((((- 0.15 *  cos__q_RF_q2_joint__) - ( 0.125 *  sin__q_RF_q1_joint__)) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    (*this)(5,3) =  sin__q_RF_q2_joint__;
    (*this)(5,4) =  cos__q_RF_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_RM_foot::Type_fr_trunk_X_RM_foot()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_RM_foot& iit::mcorin::MotionTransforms::Type_fr_trunk_X_RM_foot::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q3_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(0,1) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(1,0) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,1) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(2,0) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(2,1) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    (*this)(3,0) = ((((((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(3,1) = ((((( 0.075 + ( 0.077 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (((((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__)) *  cos__q_RM_q3_joint__)) - ( 0.17 *  cos__q_RM_q1_joint__));
    (*this)(3,2) = ((((( 0.17 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + ((( 0.17 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) + (( 0.15 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__));
    (*this)(3,3) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(3,4) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(3,5) = - cos__q_RM_q1_joint__;
    (*this)(4,0) = ((((- 0.15 *  sin__q_RM_q1_joint__) - (( 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) *  sin__q_RM_q3_joint__) - ((( 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,1) = ((((( 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (((- 0.15 *  sin__q_RM_q1_joint__) - (( 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) *  cos__q_RM_q3_joint__)) - ( 0.17 *  sin__q_RM_q1_joint__));
    (*this)(4,2) = (((((- 0.17 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.17 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) - (( 0.15 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__));
    (*this)(4,3) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,4) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,5) = - sin__q_RM_q1_joint__;
    (*this)(5,0) = (((( 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - ((( 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(5,1) = ((((- 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(5,2) = (((((( 0.17 *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( 0.17 *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) - ( 0.15 *  cos__q_RM_q2_joint__)) - ( 0.075 *  cos__q_RM_q1_joint__)) -  0.077);
    (*this)(5,3) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(5,4) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RM_q1_joint::Type_fr_trunk_X_fr_RM_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 1;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = - 1;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = - 0.075;
    (*this)(3,3) = 0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = - 1;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0.075;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RM_q1_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RM_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RM_q2_joint::Type_fr_trunk_X_fr_RM_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,1) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RM_q2_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RM_q2_joint::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RM_q1_joint__;
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(1,0) = - cos__q_RM_q1_joint__;
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(3,1) = ((- 0.077 *  cos__q_RM_q1_joint__) -  0.075);
    (*this)(3,3) =  sin__q_RM_q1_joint__;
    (*this)(3,5) = - cos__q_RM_q1_joint__;
    (*this)(4,1) = (- 0.077 *  sin__q_RM_q1_joint__);
    (*this)(4,3) = - cos__q_RM_q1_joint__;
    (*this)(4,5) = - sin__q_RM_q1_joint__;
    (*this)(5,0) = ( 0.075 *  sin__q_RM_q1_joint__);
    (*this)(5,2) = ((- 0.075 *  cos__q_RM_q1_joint__) -  0.077);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RM_q3_joint::Type_fr_trunk_X_fr_RM_q3_joint()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RM_q3_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RM_q3_joint::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(0,1) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(1,0) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(1,1) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(2,0) =  sin__q_RM_q2_joint__;
    (*this)(2,1) =  cos__q_RM_q2_joint__;
    (*this)(3,0) = (((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  sin__q_RM_q2_joint__);
    (*this)(3,1) = (((- 0.075 - ( 0.077 *  cos__q_RM_q1_joint__)) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__));
    (*this)(3,2) = (( 0.15 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(3,3) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(3,4) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(3,5) = - cos__q_RM_q1_joint__;
    (*this)(4,0) = ((- 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(4,1) = (((- 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) - ( 0.15 *  sin__q_RM_q1_joint__));
    (*this)(4,2) = ((- 0.15 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(4,3) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(4,4) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(4,5) = - sin__q_RM_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__);
    (*this)(5,1) = ((- 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(5,2) = (((- 0.15 *  cos__q_RM_q2_joint__) - ( 0.075 *  cos__q_RM_q1_joint__)) -  0.077);
    (*this)(5,3) =  sin__q_RM_q2_joint__;
    (*this)(5,4) =  cos__q_RM_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_RR_foot::Type_fr_trunk_X_RR_foot()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_RR_foot& iit::mcorin::MotionTransforms::Type_fr_trunk_X_RR_foot::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q3_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(0,1) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(1,0) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,1) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(2,0) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(2,1) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    (*this)(3,0) = ((((((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(3,1) = ((((( 0.075 + ( 0.077 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q3_joint__)) - ( 0.17 *  cos__q_RR_q1_joint__));
    (*this)(3,2) = ((((( 0.17 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((( 0.17 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) + (( 0.15 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__));
    (*this)(3,3) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(3,4) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(3,5) = - cos__q_RR_q1_joint__;
    (*this)(4,0) = ((((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q3_joint__) + ((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,1) = (((((( 0.077 *  sin__q_RR_q1_joint__) -  0.125) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q3_joint__)) - ( 0.17 *  sin__q_RR_q1_joint__));
    (*this)(4,2) = (((((- 0.17 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - ((( 0.17 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) - (( 0.15 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__));
    (*this)(4,3) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,4) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,5) = - sin__q_RR_q1_joint__;
    (*this)(5,0) = (((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((( 0.125 *  cos__q_RR_q1_joint__) + ( 0.075 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(5,1) = (((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(5,2) = ((((((( 0.17 *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( 0.17 *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) - ( 0.15 *  cos__q_RR_q2_joint__)) + ( 0.125 *  sin__q_RR_q1_joint__)) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    (*this)(5,3) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(5,4) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RR_q1_joint::Type_fr_trunk_X_fr_RR_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = - 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = - 0.075;
    (*this)(3,3) = 0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0.125;
    (*this)(4,3) = - 1.0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0.125;
    (*this)(5,1) = 0.075;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RR_q1_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RR_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RR_q2_joint::Type_fr_trunk_X_fr_RR_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,1) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RR_q2_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RR_q2_joint::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RR_q1_joint__;
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(1,0) = - cos__q_RR_q1_joint__;
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(3,1) = ((- 0.077 *  cos__q_RR_q1_joint__) -  0.075);
    (*this)(3,3) =  sin__q_RR_q1_joint__;
    (*this)(3,5) = - cos__q_RR_q1_joint__;
    (*this)(4,1) = ( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__));
    (*this)(4,3) = - cos__q_RR_q1_joint__;
    (*this)(4,5) = - sin__q_RR_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__));
    (*this)(5,2) = ((( 0.125 *  sin__q_RR_q1_joint__) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RR_q3_joint::Type_fr_trunk_X_fr_RR_q3_joint()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RR_q3_joint& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RR_q3_joint::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(0,1) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(1,0) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(1,1) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(2,0) =  sin__q_RR_q2_joint__;
    (*this)(2,1) =  cos__q_RR_q2_joint__;
    (*this)(3,0) = (((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  sin__q_RR_q2_joint__);
    (*this)(3,1) = (((- 0.075 - ( 0.077 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__));
    (*this)(3,2) = (( 0.15 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__);
    (*this)(3,3) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(3,4) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(3,5) = - cos__q_RR_q1_joint__;
    (*this)(4,0) = (( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__);
    (*this)(4,1) = ((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__));
    (*this)(4,2) = ((- 0.15 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__);
    (*this)(4,3) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(4,4) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(4,5) = - sin__q_RR_q1_joint__;
    (*this)(5,0) = ((( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__);
    (*this)(5,1) = (((- 0.075 *  sin__q_RR_q1_joint__) - ( 0.125 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__);
    (*this)(5,2) = ((((- 0.15 *  cos__q_RR_q2_joint__) + ( 0.125 *  sin__q_RR_q1_joint__)) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    (*this)(5,3) =  sin__q_RR_q2_joint__;
    (*this)(5,4) =  cos__q_RR_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LF_hipassembly_X_fr_trunk::Type_fr_LF_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0.075;
    (*this)(5,1) = - 0.125;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_LF_hipassembly_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_LF_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LF_q1_joint__;
    (*this)(0,1) =  cos__q_LF_q1_joint__;
    (*this)(1,0) = - cos__q_LF_q1_joint__;
    (*this)(1,1) = - sin__q_LF_q1_joint__;
    (*this)(3,2) = (( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__));
    (*this)(3,3) = - sin__q_LF_q1_joint__;
    (*this)(3,4) =  cos__q_LF_q1_joint__;
    (*this)(4,2) = (( 0.075 *  cos__q_LF_q1_joint__) - ( 0.125 *  sin__q_LF_q1_joint__));
    (*this)(4,3) = - cos__q_LF_q1_joint__;
    (*this)(4,4) = - sin__q_LF_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LF_hipassembly::Type_fr_trunk_X_fr_LF_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0.075;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.125;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LF_hipassembly& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LF_hipassembly::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LF_q1_joint__;
    (*this)(0,1) = - cos__q_LF_q1_joint__;
    (*this)(1,0) =  cos__q_LF_q1_joint__;
    (*this)(1,1) = - sin__q_LF_q1_joint__;
    (*this)(3,3) = - sin__q_LF_q1_joint__;
    (*this)(3,4) = - cos__q_LF_q1_joint__;
    (*this)(4,3) =  cos__q_LF_q1_joint__;
    (*this)(4,4) = - sin__q_LF_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__));
    (*this)(5,1) = (( 0.075 *  cos__q_LF_q1_joint__) - ( 0.125 *  sin__q_LF_q1_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LF_upperleg_X_fr_LF_hipassembly::Type_fr_LF_upperleg_X_fr_LF_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_LF_upperleg_X_fr_LF_hipassembly& iit::mcorin::MotionTransforms::Type_fr_LF_upperleg_X_fr_LF_hipassembly::update(const state_t& q) {
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LF_q2_joint__;
    (*this)(0,2) =  sin__q_LF_q2_joint__;
    (*this)(1,0) = - sin__q_LF_q2_joint__;
    (*this)(1,2) =  cos__q_LF_q2_joint__;
    (*this)(3,1) = (- 0.077 *  sin__q_LF_q2_joint__);
    (*this)(3,3) =  cos__q_LF_q2_joint__;
    (*this)(3,5) =  sin__q_LF_q2_joint__;
    (*this)(4,1) = (- 0.077 *  cos__q_LF_q2_joint__);
    (*this)(4,3) = - sin__q_LF_q2_joint__;
    (*this)(4,5) =  cos__q_LF_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LF_hipassembly_X_fr_LF_upperleg::Type_fr_LF_hipassembly_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_LF_hipassembly_X_fr_LF_upperleg& iit::mcorin::MotionTransforms::Type_fr_LF_hipassembly_X_fr_LF_upperleg::update(const state_t& q) {
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LF_q2_joint__;
    (*this)(0,1) = - sin__q_LF_q2_joint__;
    (*this)(2,0) =  sin__q_LF_q2_joint__;
    (*this)(2,1) =  cos__q_LF_q2_joint__;
    (*this)(3,3) =  cos__q_LF_q2_joint__;
    (*this)(3,4) = - sin__q_LF_q2_joint__;
    (*this)(4,0) = (- 0.077 *  sin__q_LF_q2_joint__);
    (*this)(4,1) = (- 0.077 *  cos__q_LF_q2_joint__);
    (*this)(5,3) =  sin__q_LF_q2_joint__;
    (*this)(5,4) =  cos__q_LF_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LF_lowerleg_X_fr_LF_upperleg::Type_fr_LF_lowerleg_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.15;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::MotionTransforms::Type_fr_LF_lowerleg_X_fr_LF_upperleg& iit::mcorin::MotionTransforms::Type_fr_LF_lowerleg_X_fr_LF_upperleg::update(const state_t& q) {
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q3_joint__;
    
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LF_q3_joint__;
    (*this)(0,1) =  sin__q_LF_q3_joint__;
    (*this)(1,0) = - sin__q_LF_q3_joint__;
    (*this)(1,1) =  cos__q_LF_q3_joint__;
    (*this)(3,2) = ( 0.15 *  sin__q_LF_q3_joint__);
    (*this)(3,3) =  cos__q_LF_q3_joint__;
    (*this)(3,4) =  sin__q_LF_q3_joint__;
    (*this)(4,2) = ( 0.15 *  cos__q_LF_q3_joint__);
    (*this)(4,3) = - sin__q_LF_q3_joint__;
    (*this)(4,4) =  cos__q_LF_q3_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LF_upperleg_X_fr_LF_lowerleg::Type_fr_LF_upperleg_X_fr_LF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.15;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_LF_upperleg_X_fr_LF_lowerleg& iit::mcorin::MotionTransforms::Type_fr_LF_upperleg_X_fr_LF_lowerleg::update(const state_t& q) {
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q3_joint__;
    
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LF_q3_joint__;
    (*this)(0,1) = - sin__q_LF_q3_joint__;
    (*this)(1,0) =  sin__q_LF_q3_joint__;
    (*this)(1,1) =  cos__q_LF_q3_joint__;
    (*this)(3,3) =  cos__q_LF_q3_joint__;
    (*this)(3,4) = - sin__q_LF_q3_joint__;
    (*this)(4,3) =  sin__q_LF_q3_joint__;
    (*this)(4,4) =  cos__q_LF_q3_joint__;
    (*this)(5,0) = ( 0.15 *  sin__q_LF_q3_joint__);
    (*this)(5,1) = ( 0.15 *  cos__q_LF_q3_joint__);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LM_hipassembly_X_fr_trunk::Type_fr_LM_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0.075;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::MotionTransforms::Type_fr_LM_hipassembly_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_LM_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LM_q1_joint__;
    (*this)(0,1) =  cos__q_LM_q1_joint__;
    (*this)(1,0) = - cos__q_LM_q1_joint__;
    (*this)(1,1) = - sin__q_LM_q1_joint__;
    (*this)(3,2) = ( 0.075 *  sin__q_LM_q1_joint__);
    (*this)(3,3) = - sin__q_LM_q1_joint__;
    (*this)(3,4) =  cos__q_LM_q1_joint__;
    (*this)(4,2) = ( 0.075 *  cos__q_LM_q1_joint__);
    (*this)(4,3) = - cos__q_LM_q1_joint__;
    (*this)(4,4) = - sin__q_LM_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LM_hipassembly::Type_fr_trunk_X_fr_LM_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0.075;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LM_hipassembly& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LM_hipassembly::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LM_q1_joint__;
    (*this)(0,1) = - cos__q_LM_q1_joint__;
    (*this)(1,0) =  cos__q_LM_q1_joint__;
    (*this)(1,1) = - sin__q_LM_q1_joint__;
    (*this)(3,3) = - sin__q_LM_q1_joint__;
    (*this)(3,4) = - cos__q_LM_q1_joint__;
    (*this)(4,3) =  cos__q_LM_q1_joint__;
    (*this)(4,4) = - sin__q_LM_q1_joint__;
    (*this)(5,0) = ( 0.075 *  sin__q_LM_q1_joint__);
    (*this)(5,1) = ( 0.075 *  cos__q_LM_q1_joint__);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LM_upperleg_X_fr_LM_hipassembly::Type_fr_LM_upperleg_X_fr_LM_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_LM_upperleg_X_fr_LM_hipassembly& iit::mcorin::MotionTransforms::Type_fr_LM_upperleg_X_fr_LM_hipassembly::update(const state_t& q) {
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LM_q2_joint__;
    (*this)(0,2) =  sin__q_LM_q2_joint__;
    (*this)(1,0) = - sin__q_LM_q2_joint__;
    (*this)(1,2) =  cos__q_LM_q2_joint__;
    (*this)(3,1) = (- 0.077 *  sin__q_LM_q2_joint__);
    (*this)(3,3) =  cos__q_LM_q2_joint__;
    (*this)(3,5) =  sin__q_LM_q2_joint__;
    (*this)(4,1) = (- 0.077 *  cos__q_LM_q2_joint__);
    (*this)(4,3) = - sin__q_LM_q2_joint__;
    (*this)(4,5) =  cos__q_LM_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LM_hipassembly_X_fr_LM_upperleg::Type_fr_LM_hipassembly_X_fr_LM_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_LM_hipassembly_X_fr_LM_upperleg& iit::mcorin::MotionTransforms::Type_fr_LM_hipassembly_X_fr_LM_upperleg::update(const state_t& q) {
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LM_q2_joint__;
    (*this)(0,1) = - sin__q_LM_q2_joint__;
    (*this)(2,0) =  sin__q_LM_q2_joint__;
    (*this)(2,1) =  cos__q_LM_q2_joint__;
    (*this)(3,3) =  cos__q_LM_q2_joint__;
    (*this)(3,4) = - sin__q_LM_q2_joint__;
    (*this)(4,0) = (- 0.077 *  sin__q_LM_q2_joint__);
    (*this)(4,1) = (- 0.077 *  cos__q_LM_q2_joint__);
    (*this)(5,3) =  sin__q_LM_q2_joint__;
    (*this)(5,4) =  cos__q_LM_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LM_lowerleg_X_fr_LM_upperleg::Type_fr_LM_lowerleg_X_fr_LM_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.15;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::MotionTransforms::Type_fr_LM_lowerleg_X_fr_LM_upperleg& iit::mcorin::MotionTransforms::Type_fr_LM_lowerleg_X_fr_LM_upperleg::update(const state_t& q) {
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q3_joint__;
    
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LM_q3_joint__;
    (*this)(0,1) =  sin__q_LM_q3_joint__;
    (*this)(1,0) = - sin__q_LM_q3_joint__;
    (*this)(1,1) =  cos__q_LM_q3_joint__;
    (*this)(3,2) = ( 0.15 *  sin__q_LM_q3_joint__);
    (*this)(3,3) =  cos__q_LM_q3_joint__;
    (*this)(3,4) =  sin__q_LM_q3_joint__;
    (*this)(4,2) = ( 0.15 *  cos__q_LM_q3_joint__);
    (*this)(4,3) = - sin__q_LM_q3_joint__;
    (*this)(4,4) =  cos__q_LM_q3_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LM_upperleg_X_fr_LM_lowerleg::Type_fr_LM_upperleg_X_fr_LM_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.15;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_LM_upperleg_X_fr_LM_lowerleg& iit::mcorin::MotionTransforms::Type_fr_LM_upperleg_X_fr_LM_lowerleg::update(const state_t& q) {
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q3_joint__;
    
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LM_q3_joint__;
    (*this)(0,1) = - sin__q_LM_q3_joint__;
    (*this)(1,0) =  sin__q_LM_q3_joint__;
    (*this)(1,1) =  cos__q_LM_q3_joint__;
    (*this)(3,3) =  cos__q_LM_q3_joint__;
    (*this)(3,4) = - sin__q_LM_q3_joint__;
    (*this)(4,3) =  sin__q_LM_q3_joint__;
    (*this)(4,4) =  cos__q_LM_q3_joint__;
    (*this)(5,0) = ( 0.15 *  sin__q_LM_q3_joint__);
    (*this)(5,1) = ( 0.15 *  cos__q_LM_q3_joint__);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LR_hipassembly_X_fr_trunk::Type_fr_LR_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0.075;
    (*this)(5,1) = 0.125;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_LR_hipassembly_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_LR_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LR_q1_joint__;
    (*this)(0,1) =  cos__q_LR_q1_joint__;
    (*this)(1,0) = - cos__q_LR_q1_joint__;
    (*this)(1,1) = - sin__q_LR_q1_joint__;
    (*this)(3,2) = (( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__));
    (*this)(3,3) = - sin__q_LR_q1_joint__;
    (*this)(3,4) =  cos__q_LR_q1_joint__;
    (*this)(4,2) = (( 0.125 *  sin__q_LR_q1_joint__) + ( 0.075 *  cos__q_LR_q1_joint__));
    (*this)(4,3) = - cos__q_LR_q1_joint__;
    (*this)(4,4) = - sin__q_LR_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LR_hipassembly::Type_fr_trunk_X_fr_LR_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0.075;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0.125;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LR_hipassembly& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_LR_hipassembly::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LR_q1_joint__;
    (*this)(0,1) = - cos__q_LR_q1_joint__;
    (*this)(1,0) =  cos__q_LR_q1_joint__;
    (*this)(1,1) = - sin__q_LR_q1_joint__;
    (*this)(3,3) = - sin__q_LR_q1_joint__;
    (*this)(3,4) = - cos__q_LR_q1_joint__;
    (*this)(4,3) =  cos__q_LR_q1_joint__;
    (*this)(4,4) = - sin__q_LR_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__));
    (*this)(5,1) = (( 0.125 *  sin__q_LR_q1_joint__) + ( 0.075 *  cos__q_LR_q1_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LR_upperleg_X_fr_LR_hipassembly::Type_fr_LR_upperleg_X_fr_LR_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_LR_upperleg_X_fr_LR_hipassembly& iit::mcorin::MotionTransforms::Type_fr_LR_upperleg_X_fr_LR_hipassembly::update(const state_t& q) {
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LR_q2_joint__;
    (*this)(0,2) =  sin__q_LR_q2_joint__;
    (*this)(1,0) = - sin__q_LR_q2_joint__;
    (*this)(1,2) =  cos__q_LR_q2_joint__;
    (*this)(3,1) = (- 0.077 *  sin__q_LR_q2_joint__);
    (*this)(3,3) =  cos__q_LR_q2_joint__;
    (*this)(3,5) =  sin__q_LR_q2_joint__;
    (*this)(4,1) = (- 0.077 *  cos__q_LR_q2_joint__);
    (*this)(4,3) = - sin__q_LR_q2_joint__;
    (*this)(4,5) =  cos__q_LR_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LR_hipassembly_X_fr_LR_upperleg::Type_fr_LR_hipassembly_X_fr_LR_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_LR_hipassembly_X_fr_LR_upperleg& iit::mcorin::MotionTransforms::Type_fr_LR_hipassembly_X_fr_LR_upperleg::update(const state_t& q) {
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LR_q2_joint__;
    (*this)(0,1) = - sin__q_LR_q2_joint__;
    (*this)(2,0) =  sin__q_LR_q2_joint__;
    (*this)(2,1) =  cos__q_LR_q2_joint__;
    (*this)(3,3) =  cos__q_LR_q2_joint__;
    (*this)(3,4) = - sin__q_LR_q2_joint__;
    (*this)(4,0) = (- 0.077 *  sin__q_LR_q2_joint__);
    (*this)(4,1) = (- 0.077 *  cos__q_LR_q2_joint__);
    (*this)(5,3) =  sin__q_LR_q2_joint__;
    (*this)(5,4) =  cos__q_LR_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LR_lowerleg_X_fr_LR_upperleg::Type_fr_LR_lowerleg_X_fr_LR_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.15;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::MotionTransforms::Type_fr_LR_lowerleg_X_fr_LR_upperleg& iit::mcorin::MotionTransforms::Type_fr_LR_lowerleg_X_fr_LR_upperleg::update(const state_t& q) {
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q3_joint__;
    
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LR_q3_joint__;
    (*this)(0,1) =  sin__q_LR_q3_joint__;
    (*this)(1,0) = - sin__q_LR_q3_joint__;
    (*this)(1,1) =  cos__q_LR_q3_joint__;
    (*this)(3,2) = ( 0.15 *  sin__q_LR_q3_joint__);
    (*this)(3,3) =  cos__q_LR_q3_joint__;
    (*this)(3,4) =  sin__q_LR_q3_joint__;
    (*this)(4,2) = ( 0.15 *  cos__q_LR_q3_joint__);
    (*this)(4,3) = - sin__q_LR_q3_joint__;
    (*this)(4,4) =  cos__q_LR_q3_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_LR_upperleg_X_fr_LR_lowerleg::Type_fr_LR_upperleg_X_fr_LR_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.15;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_LR_upperleg_X_fr_LR_lowerleg& iit::mcorin::MotionTransforms::Type_fr_LR_upperleg_X_fr_LR_lowerleg::update(const state_t& q) {
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q3_joint__;
    
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LR_q3_joint__;
    (*this)(0,1) = - sin__q_LR_q3_joint__;
    (*this)(1,0) =  sin__q_LR_q3_joint__;
    (*this)(1,1) =  cos__q_LR_q3_joint__;
    (*this)(3,3) =  cos__q_LR_q3_joint__;
    (*this)(3,4) = - sin__q_LR_q3_joint__;
    (*this)(4,3) =  sin__q_LR_q3_joint__;
    (*this)(4,4) =  cos__q_LR_q3_joint__;
    (*this)(5,0) = ( 0.15 *  sin__q_LR_q3_joint__);
    (*this)(5,1) = ( 0.15 *  cos__q_LR_q3_joint__);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RF_hipassembly_X_fr_trunk::Type_fr_RF_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = - 0.075;
    (*this)(5,1) = - 0.125;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_RF_hipassembly_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_RF_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RF_q1_joint__;
    (*this)(0,1) = - cos__q_RF_q1_joint__;
    (*this)(1,0) =  cos__q_RF_q1_joint__;
    (*this)(1,1) =  sin__q_RF_q1_joint__;
    (*this)(3,2) = (( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__));
    (*this)(3,3) =  sin__q_RF_q1_joint__;
    (*this)(3,4) = - cos__q_RF_q1_joint__;
    (*this)(4,2) = (( 0.125 *  sin__q_RF_q1_joint__) + ( 0.075 *  cos__q_RF_q1_joint__));
    (*this)(4,3) =  cos__q_RF_q1_joint__;
    (*this)(4,4) =  sin__q_RF_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RF_hipassembly::Type_fr_trunk_X_fr_RF_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = - 0.075;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.125;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RF_hipassembly& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RF_hipassembly::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RF_q1_joint__;
    (*this)(0,1) =  cos__q_RF_q1_joint__;
    (*this)(1,0) = - cos__q_RF_q1_joint__;
    (*this)(1,1) =  sin__q_RF_q1_joint__;
    (*this)(3,3) =  sin__q_RF_q1_joint__;
    (*this)(3,4) =  cos__q_RF_q1_joint__;
    (*this)(4,3) = - cos__q_RF_q1_joint__;
    (*this)(4,4) =  sin__q_RF_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__));
    (*this)(5,1) = (( 0.125 *  sin__q_RF_q1_joint__) + ( 0.075 *  cos__q_RF_q1_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RF_upperleg_X_fr_RF_hipassembly::Type_fr_RF_upperleg_X_fr_RF_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_RF_upperleg_X_fr_RF_hipassembly& iit::mcorin::MotionTransforms::Type_fr_RF_upperleg_X_fr_RF_hipassembly::update(const state_t& q) {
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RF_q2_joint__;
    (*this)(0,2) =  sin__q_RF_q2_joint__;
    (*this)(1,0) = - sin__q_RF_q2_joint__;
    (*this)(1,2) =  cos__q_RF_q2_joint__;
    (*this)(3,1) = (- 0.077 *  sin__q_RF_q2_joint__);
    (*this)(3,3) =  cos__q_RF_q2_joint__;
    (*this)(3,5) =  sin__q_RF_q2_joint__;
    (*this)(4,1) = (- 0.077 *  cos__q_RF_q2_joint__);
    (*this)(4,3) = - sin__q_RF_q2_joint__;
    (*this)(4,5) =  cos__q_RF_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RF_hipassembly_X_fr_RF_upperleg::Type_fr_RF_hipassembly_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_RF_hipassembly_X_fr_RF_upperleg& iit::mcorin::MotionTransforms::Type_fr_RF_hipassembly_X_fr_RF_upperleg::update(const state_t& q) {
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RF_q2_joint__;
    (*this)(0,1) = - sin__q_RF_q2_joint__;
    (*this)(2,0) =  sin__q_RF_q2_joint__;
    (*this)(2,1) =  cos__q_RF_q2_joint__;
    (*this)(3,3) =  cos__q_RF_q2_joint__;
    (*this)(3,4) = - sin__q_RF_q2_joint__;
    (*this)(4,0) = (- 0.077 *  sin__q_RF_q2_joint__);
    (*this)(4,1) = (- 0.077 *  cos__q_RF_q2_joint__);
    (*this)(5,3) =  sin__q_RF_q2_joint__;
    (*this)(5,4) =  cos__q_RF_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RF_lowerleg_X_fr_RF_upperleg::Type_fr_RF_lowerleg_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.15;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::MotionTransforms::Type_fr_RF_lowerleg_X_fr_RF_upperleg& iit::mcorin::MotionTransforms::Type_fr_RF_lowerleg_X_fr_RF_upperleg::update(const state_t& q) {
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q3_joint__;
    
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RF_q3_joint__;
    (*this)(0,1) =  sin__q_RF_q3_joint__;
    (*this)(1,0) = - sin__q_RF_q3_joint__;
    (*this)(1,1) =  cos__q_RF_q3_joint__;
    (*this)(3,2) = ( 0.15 *  sin__q_RF_q3_joint__);
    (*this)(3,3) =  cos__q_RF_q3_joint__;
    (*this)(3,4) =  sin__q_RF_q3_joint__;
    (*this)(4,2) = ( 0.15 *  cos__q_RF_q3_joint__);
    (*this)(4,3) = - sin__q_RF_q3_joint__;
    (*this)(4,4) =  cos__q_RF_q3_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RF_upperleg_X_fr_RF_lowerleg::Type_fr_RF_upperleg_X_fr_RF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.15;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_RF_upperleg_X_fr_RF_lowerleg& iit::mcorin::MotionTransforms::Type_fr_RF_upperleg_X_fr_RF_lowerleg::update(const state_t& q) {
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q3_joint__;
    
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RF_q3_joint__;
    (*this)(0,1) = - sin__q_RF_q3_joint__;
    (*this)(1,0) =  sin__q_RF_q3_joint__;
    (*this)(1,1) =  cos__q_RF_q3_joint__;
    (*this)(3,3) =  cos__q_RF_q3_joint__;
    (*this)(3,4) = - sin__q_RF_q3_joint__;
    (*this)(4,3) =  sin__q_RF_q3_joint__;
    (*this)(4,4) =  cos__q_RF_q3_joint__;
    (*this)(5,0) = ( 0.15 *  sin__q_RF_q3_joint__);
    (*this)(5,1) = ( 0.15 *  cos__q_RF_q3_joint__);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RM_hipassembly_X_fr_trunk::Type_fr_RM_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = - 0.075;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::MotionTransforms::Type_fr_RM_hipassembly_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_RM_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RM_q1_joint__;
    (*this)(0,1) = - cos__q_RM_q1_joint__;
    (*this)(1,0) =  cos__q_RM_q1_joint__;
    (*this)(1,1) =  sin__q_RM_q1_joint__;
    (*this)(3,2) = ( 0.075 *  sin__q_RM_q1_joint__);
    (*this)(3,3) =  sin__q_RM_q1_joint__;
    (*this)(3,4) = - cos__q_RM_q1_joint__;
    (*this)(4,2) = ( 0.075 *  cos__q_RM_q1_joint__);
    (*this)(4,3) =  cos__q_RM_q1_joint__;
    (*this)(4,4) =  sin__q_RM_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RM_hipassembly::Type_fr_trunk_X_fr_RM_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = - 0.075;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RM_hipassembly& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RM_hipassembly::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RM_q1_joint__;
    (*this)(0,1) =  cos__q_RM_q1_joint__;
    (*this)(1,0) = - cos__q_RM_q1_joint__;
    (*this)(1,1) =  sin__q_RM_q1_joint__;
    (*this)(3,3) =  sin__q_RM_q1_joint__;
    (*this)(3,4) =  cos__q_RM_q1_joint__;
    (*this)(4,3) = - cos__q_RM_q1_joint__;
    (*this)(4,4) =  sin__q_RM_q1_joint__;
    (*this)(5,0) = ( 0.075 *  sin__q_RM_q1_joint__);
    (*this)(5,1) = ( 0.075 *  cos__q_RM_q1_joint__);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RM_upperleg_X_fr_RM_hipassembly::Type_fr_RM_upperleg_X_fr_RM_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_RM_upperleg_X_fr_RM_hipassembly& iit::mcorin::MotionTransforms::Type_fr_RM_upperleg_X_fr_RM_hipassembly::update(const state_t& q) {
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RM_q2_joint__;
    (*this)(0,2) =  sin__q_RM_q2_joint__;
    (*this)(1,0) = - sin__q_RM_q2_joint__;
    (*this)(1,2) =  cos__q_RM_q2_joint__;
    (*this)(3,1) = (- 0.077 *  sin__q_RM_q2_joint__);
    (*this)(3,3) =  cos__q_RM_q2_joint__;
    (*this)(3,5) =  sin__q_RM_q2_joint__;
    (*this)(4,1) = (- 0.077 *  cos__q_RM_q2_joint__);
    (*this)(4,3) = - sin__q_RM_q2_joint__;
    (*this)(4,5) =  cos__q_RM_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RM_hipassembly_X_fr_RM_upperleg::Type_fr_RM_hipassembly_X_fr_RM_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_RM_hipassembly_X_fr_RM_upperleg& iit::mcorin::MotionTransforms::Type_fr_RM_hipassembly_X_fr_RM_upperleg::update(const state_t& q) {
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RM_q2_joint__;
    (*this)(0,1) = - sin__q_RM_q2_joint__;
    (*this)(2,0) =  sin__q_RM_q2_joint__;
    (*this)(2,1) =  cos__q_RM_q2_joint__;
    (*this)(3,3) =  cos__q_RM_q2_joint__;
    (*this)(3,4) = - sin__q_RM_q2_joint__;
    (*this)(4,0) = (- 0.077 *  sin__q_RM_q2_joint__);
    (*this)(4,1) = (- 0.077 *  cos__q_RM_q2_joint__);
    (*this)(5,3) =  sin__q_RM_q2_joint__;
    (*this)(5,4) =  cos__q_RM_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RM_lowerleg_X_fr_RM_upperleg::Type_fr_RM_lowerleg_X_fr_RM_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.15;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::MotionTransforms::Type_fr_RM_lowerleg_X_fr_RM_upperleg& iit::mcorin::MotionTransforms::Type_fr_RM_lowerleg_X_fr_RM_upperleg::update(const state_t& q) {
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q3_joint__;
    
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RM_q3_joint__;
    (*this)(0,1) =  sin__q_RM_q3_joint__;
    (*this)(1,0) = - sin__q_RM_q3_joint__;
    (*this)(1,1) =  cos__q_RM_q3_joint__;
    (*this)(3,2) = ( 0.15 *  sin__q_RM_q3_joint__);
    (*this)(3,3) =  cos__q_RM_q3_joint__;
    (*this)(3,4) =  sin__q_RM_q3_joint__;
    (*this)(4,2) = ( 0.15 *  cos__q_RM_q3_joint__);
    (*this)(4,3) = - sin__q_RM_q3_joint__;
    (*this)(4,4) =  cos__q_RM_q3_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RM_upperleg_X_fr_RM_lowerleg::Type_fr_RM_upperleg_X_fr_RM_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.15;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_RM_upperleg_X_fr_RM_lowerleg& iit::mcorin::MotionTransforms::Type_fr_RM_upperleg_X_fr_RM_lowerleg::update(const state_t& q) {
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q3_joint__;
    
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RM_q3_joint__;
    (*this)(0,1) = - sin__q_RM_q3_joint__;
    (*this)(1,0) =  sin__q_RM_q3_joint__;
    (*this)(1,1) =  cos__q_RM_q3_joint__;
    (*this)(3,3) =  cos__q_RM_q3_joint__;
    (*this)(3,4) = - sin__q_RM_q3_joint__;
    (*this)(4,3) =  sin__q_RM_q3_joint__;
    (*this)(4,4) =  cos__q_RM_q3_joint__;
    (*this)(5,0) = ( 0.15 *  sin__q_RM_q3_joint__);
    (*this)(5,1) = ( 0.15 *  cos__q_RM_q3_joint__);
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RR_hipassembly_X_fr_trunk::Type_fr_RR_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = - 0.075;
    (*this)(5,1) = 0.125;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_RR_hipassembly_X_fr_trunk& iit::mcorin::MotionTransforms::Type_fr_RR_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RR_q1_joint__;
    (*this)(0,1) = - cos__q_RR_q1_joint__;
    (*this)(1,0) =  cos__q_RR_q1_joint__;
    (*this)(1,1) =  sin__q_RR_q1_joint__;
    (*this)(3,2) = (( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__));
    (*this)(3,3) =  sin__q_RR_q1_joint__;
    (*this)(3,4) = - cos__q_RR_q1_joint__;
    (*this)(4,2) = (( 0.075 *  cos__q_RR_q1_joint__) - ( 0.125 *  sin__q_RR_q1_joint__));
    (*this)(4,3) =  cos__q_RR_q1_joint__;
    (*this)(4,4) =  sin__q_RR_q1_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RR_hipassembly::Type_fr_trunk_X_fr_RR_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = - 0.075;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0.125;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RR_hipassembly& iit::mcorin::MotionTransforms::Type_fr_trunk_X_fr_RR_hipassembly::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RR_q1_joint__;
    (*this)(0,1) =  cos__q_RR_q1_joint__;
    (*this)(1,0) = - cos__q_RR_q1_joint__;
    (*this)(1,1) =  sin__q_RR_q1_joint__;
    (*this)(3,3) =  sin__q_RR_q1_joint__;
    (*this)(3,4) =  cos__q_RR_q1_joint__;
    (*this)(4,3) = - cos__q_RR_q1_joint__;
    (*this)(4,4) =  sin__q_RR_q1_joint__;
    (*this)(5,0) = (( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__));
    (*this)(5,1) = (( 0.075 *  cos__q_RR_q1_joint__) - ( 0.125 *  sin__q_RR_q1_joint__));
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RR_upperleg_X_fr_RR_hipassembly::Type_fr_RR_upperleg_X_fr_RR_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_RR_upperleg_X_fr_RR_hipassembly& iit::mcorin::MotionTransforms::Type_fr_RR_upperleg_X_fr_RR_hipassembly::update(const state_t& q) {
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RR_q2_joint__;
    (*this)(0,2) =  sin__q_RR_q2_joint__;
    (*this)(1,0) = - sin__q_RR_q2_joint__;
    (*this)(1,2) =  cos__q_RR_q2_joint__;
    (*this)(3,1) = (- 0.077 *  sin__q_RR_q2_joint__);
    (*this)(3,3) =  cos__q_RR_q2_joint__;
    (*this)(3,5) =  sin__q_RR_q2_joint__;
    (*this)(4,1) = (- 0.077 *  cos__q_RR_q2_joint__);
    (*this)(4,3) = - sin__q_RR_q2_joint__;
    (*this)(4,5) =  cos__q_RR_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RR_hipassembly_X_fr_RR_upperleg::Type_fr_RR_hipassembly_X_fr_RR_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,5) = 0;
}
const iit::mcorin::MotionTransforms::Type_fr_RR_hipassembly_X_fr_RR_upperleg& iit::mcorin::MotionTransforms::Type_fr_RR_hipassembly_X_fr_RR_upperleg::update(const state_t& q) {
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RR_q2_joint__;
    (*this)(0,1) = - sin__q_RR_q2_joint__;
    (*this)(2,0) =  sin__q_RR_q2_joint__;
    (*this)(2,1) =  cos__q_RR_q2_joint__;
    (*this)(3,3) =  cos__q_RR_q2_joint__;
    (*this)(3,4) = - sin__q_RR_q2_joint__;
    (*this)(4,0) = (- 0.077 *  sin__q_RR_q2_joint__);
    (*this)(4,1) = (- 0.077 *  cos__q_RR_q2_joint__);
    (*this)(5,3) =  sin__q_RR_q2_joint__;
    (*this)(5,4) =  cos__q_RR_q2_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RR_lowerleg_X_fr_RR_upperleg::Type_fr_RR_lowerleg_X_fr_RR_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.15;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::MotionTransforms::Type_fr_RR_lowerleg_X_fr_RR_upperleg& iit::mcorin::MotionTransforms::Type_fr_RR_lowerleg_X_fr_RR_upperleg::update(const state_t& q) {
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q3_joint__;
    
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RR_q3_joint__;
    (*this)(0,1) =  sin__q_RR_q3_joint__;
    (*this)(1,0) = - sin__q_RR_q3_joint__;
    (*this)(1,1) =  cos__q_RR_q3_joint__;
    (*this)(3,2) = ( 0.15 *  sin__q_RR_q3_joint__);
    (*this)(3,3) =  cos__q_RR_q3_joint__;
    (*this)(3,4) =  sin__q_RR_q3_joint__;
    (*this)(4,2) = ( 0.15 *  cos__q_RR_q3_joint__);
    (*this)(4,3) = - sin__q_RR_q3_joint__;
    (*this)(4,4) =  cos__q_RR_q3_joint__;
    return *this;
}
iit::mcorin::MotionTransforms::Type_fr_RR_upperleg_X_fr_RR_lowerleg::Type_fr_RR_upperleg_X_fr_RR_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.15;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::MotionTransforms::Type_fr_RR_upperleg_X_fr_RR_lowerleg& iit::mcorin::MotionTransforms::Type_fr_RR_upperleg_X_fr_RR_lowerleg::update(const state_t& q) {
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q3_joint__;
    
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RR_q3_joint__;
    (*this)(0,1) = - sin__q_RR_q3_joint__;
    (*this)(1,0) =  sin__q_RR_q3_joint__;
    (*this)(1,1) =  cos__q_RR_q3_joint__;
    (*this)(3,3) =  cos__q_RR_q3_joint__;
    (*this)(3,4) = - sin__q_RR_q3_joint__;
    (*this)(4,3) =  sin__q_RR_q3_joint__;
    (*this)(4,4) =  cos__q_RR_q3_joint__;
    (*this)(5,0) = ( 0.15 *  sin__q_RR_q3_joint__);
    (*this)(5,1) = ( 0.15 *  cos__q_RR_q3_joint__);
    return *this;
}

iit::mcorin::ForceTransforms::Type_fr_trunk_X_LF_hipassemblyCOM::Type_fr_trunk_X_LF_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_LF_hipassemblyCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_LF_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LF_q1_joint__;
    (*this)(0,1) = - cos__q_LF_q1_joint__;
    (*this)(0,5) = (( 0.05821 *  cos__q_LF_q1_joint__) +  0.075);
    (*this)(1,0) =  cos__q_LF_q1_joint__;
    (*this)(1,1) = - sin__q_LF_q1_joint__;
    (*this)(1,5) = (( 0.05821 *  sin__q_LF_q1_joint__) -  0.125);
    (*this)(2,3) = (( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__));
    (*this)(2,4) = (((- 0.125 *  sin__q_LF_q1_joint__) + ( 0.075 *  cos__q_LF_q1_joint__)) +  0.05821);
    (*this)(3,3) = - sin__q_LF_q1_joint__;
    (*this)(3,4) = - cos__q_LF_q1_joint__;
    (*this)(4,3) =  cos__q_LF_q1_joint__;
    (*this)(4,4) = - sin__q_LF_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_LF_upperlegCOM::Type_fr_trunk_X_LF_upperlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_LF_upperlegCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_LF_upperlegCOM::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(0,1) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(0,3) = ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  sin__q_LF_q2_joint__);
    (*this)(0,4) = ((( 0.075 + ( 0.077 *  cos__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) + ( 0.1213 *  cos__q_LF_q1_joint__));
    (*this)(0,5) = ((- 0.1213 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__);
    (*this)(1,0) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(1,1) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(1,3) = ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  sin__q_LF_q2_joint__);
    (*this)(1,4) = (((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__) + ( 0.1213 *  sin__q_LF_q1_joint__));
    (*this)(1,5) = (( 0.1213 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__);
    (*this)(2,0) =  sin__q_LF_q2_joint__;
    (*this)(2,1) =  cos__q_LF_q2_joint__;
    (*this)(2,3) = ((( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__);
    (*this)(2,4) = (((- 0.075 *  sin__q_LF_q1_joint__) - ( 0.125 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__);
    (*this)(2,5) = ((((- 0.1213 *  cos__q_LF_q2_joint__) + ( 0.125 *  sin__q_LF_q1_joint__)) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    (*this)(3,3) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(3,4) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(3,5) =  cos__q_LF_q1_joint__;
    (*this)(4,3) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(4,4) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(4,5) =  sin__q_LF_q1_joint__;
    (*this)(5,3) =  sin__q_LF_q2_joint__;
    (*this)(5,4) =  cos__q_LF_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_LF_lowerlegCOM::Type_fr_trunk_X_LF_lowerlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_LF_lowerlegCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_LF_lowerlegCOM::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q3_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,1) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(0,3) = (((( 0.15 *  cos__q_LF_q1_joint__) + ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__)) *  sin__q_LF_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,4) = (((((- 0.075 - ( 0.077 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.15 *  cos__q_LF_q1_joint__) + ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__)) *  cos__q_LF_q3_joint__)) + ( 0.08853 *  cos__q_LF_q1_joint__));
    (*this)(0,5) = (((((- 0.08853 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - ((( 0.08853 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) - (( 0.15 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__));
    (*this)(1,0) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(1,1) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(1,3) = (((( 0.15 *  sin__q_LF_q1_joint__) + ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__)) *  sin__q_LF_q3_joint__) + (((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,4) = ((((( 0.125 - ( 0.077 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.15 *  sin__q_LF_q1_joint__) + ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__)) *  cos__q_LF_q3_joint__)) + ( 0.08853 *  sin__q_LF_q1_joint__));
    (*this)(1,5) = ((((( 0.08853 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.08853 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) + (( 0.15 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__));
    (*this)(2,0) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(2,1) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    (*this)(2,3) = (((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (((( 0.125 *  cos__q_LF_q1_joint__) + ( 0.075 *  sin__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(2,4) = (((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(2,5) = ((((((( 0.08853 *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( 0.08853 *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) - ( 0.15 *  cos__q_LF_q2_joint__)) + ( 0.125 *  sin__q_LF_q1_joint__)) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,4) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,5) =  cos__q_LF_q1_joint__;
    (*this)(4,3) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(4,4) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(4,5) =  sin__q_LF_q1_joint__;
    (*this)(5,3) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(5,4) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_LM_hipassemblyCOM::Type_fr_trunk_X_LM_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_LM_hipassemblyCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_LM_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LM_q1_joint__;
    (*this)(0,1) = - cos__q_LM_q1_joint__;
    (*this)(0,5) = (( 0.05821 *  cos__q_LM_q1_joint__) +  0.075);
    (*this)(1,0) =  cos__q_LM_q1_joint__;
    (*this)(1,1) = - sin__q_LM_q1_joint__;
    (*this)(1,5) = ( 0.05821 *  sin__q_LM_q1_joint__);
    (*this)(2,3) = ( 0.075 *  sin__q_LM_q1_joint__);
    (*this)(2,4) = (( 0.075 *  cos__q_LM_q1_joint__) +  0.05821);
    (*this)(3,3) = - sin__q_LM_q1_joint__;
    (*this)(3,4) = - cos__q_LM_q1_joint__;
    (*this)(4,3) =  cos__q_LM_q1_joint__;
    (*this)(4,4) = - sin__q_LM_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_LM_upperlegCOM::Type_fr_trunk_X_LM_upperlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_LM_upperlegCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_LM_upperlegCOM::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(0,1) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(0,3) = ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  sin__q_LM_q2_joint__);
    (*this)(0,4) = ((( 0.075 + ( 0.077 *  cos__q_LM_q1_joint__)) *  cos__q_LM_q2_joint__) + ( 0.1213 *  cos__q_LM_q1_joint__));
    (*this)(0,5) = ((- 0.1213 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(1,0) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(1,1) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(1,3) = (( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(1,4) = ((( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) + ( 0.1213 *  sin__q_LM_q1_joint__));
    (*this)(1,5) = (( 0.1213 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(2,0) =  sin__q_LM_q2_joint__;
    (*this)(2,1) =  cos__q_LM_q2_joint__;
    (*this)(2,3) = (( 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__);
    (*this)(2,4) = ((- 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(2,5) = (((- 0.1213 *  cos__q_LM_q2_joint__) - ( 0.075 *  cos__q_LM_q1_joint__)) -  0.077);
    (*this)(3,3) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(3,4) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(3,5) =  cos__q_LM_q1_joint__;
    (*this)(4,3) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(4,4) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(4,5) =  sin__q_LM_q1_joint__;
    (*this)(5,3) =  sin__q_LM_q2_joint__;
    (*this)(5,4) =  cos__q_LM_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_LM_lowerlegCOM::Type_fr_trunk_X_LM_lowerlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_LM_lowerlegCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_LM_lowerlegCOM::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q3_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,1) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(0,3) = (((( 0.15 *  cos__q_LM_q1_joint__) + ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__)) *  sin__q_LM_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,4) = (((((- 0.075 - ( 0.077 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.15 *  cos__q_LM_q1_joint__) + ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__)) *  cos__q_LM_q3_joint__)) + ( 0.08853 *  cos__q_LM_q1_joint__));
    (*this)(0,5) = (((((- 0.08853 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.08853 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) - (( 0.15 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__));
    (*this)(1,0) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(1,1) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(1,3) = (((( 0.15 *  sin__q_LM_q1_joint__) + (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) *  sin__q_LM_q3_joint__) + ((( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,4) = (((((- 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.15 *  sin__q_LM_q1_joint__) + (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) *  cos__q_LM_q3_joint__)) + ( 0.08853 *  sin__q_LM_q1_joint__));
    (*this)(1,5) = ((((( 0.08853 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.08853 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) + (( 0.15 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__));
    (*this)(2,0) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(2,1) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    (*this)(2,3) = (((( 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - ((( 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(2,4) = ((((- 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(2,5) = (((((( 0.08853 *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( 0.08853 *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) - ( 0.15 *  cos__q_LM_q2_joint__)) - ( 0.075 *  cos__q_LM_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,4) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,5) =  cos__q_LM_q1_joint__;
    (*this)(4,3) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(4,4) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(4,5) =  sin__q_LM_q1_joint__;
    (*this)(5,3) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(5,4) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_LR_hipassemblyCOM::Type_fr_trunk_X_LR_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_LR_hipassemblyCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_LR_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LR_q1_joint__;
    (*this)(0,1) = - cos__q_LR_q1_joint__;
    (*this)(0,5) = (( 0.05821 *  cos__q_LR_q1_joint__) +  0.075);
    (*this)(1,0) =  cos__q_LR_q1_joint__;
    (*this)(1,1) = - sin__q_LR_q1_joint__;
    (*this)(1,5) = (( 0.05821 *  sin__q_LR_q1_joint__) +  0.125);
    (*this)(2,3) = (( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__));
    (*this)(2,4) = ((( 0.125 *  sin__q_LR_q1_joint__) + ( 0.075 *  cos__q_LR_q1_joint__)) +  0.05821);
    (*this)(3,3) = - sin__q_LR_q1_joint__;
    (*this)(3,4) = - cos__q_LR_q1_joint__;
    (*this)(4,3) =  cos__q_LR_q1_joint__;
    (*this)(4,4) = - sin__q_LR_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_LR_upperlegCOM::Type_fr_trunk_X_LR_upperlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_LR_upperlegCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_LR_upperlegCOM::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(0,1) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(0,3) = ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  sin__q_LR_q2_joint__);
    (*this)(0,4) = ((( 0.075 + ( 0.077 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) + ( 0.1213 *  cos__q_LR_q1_joint__));
    (*this)(0,5) = ((- 0.1213 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__);
    (*this)(1,0) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(1,1) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(1,3) = ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  sin__q_LR_q2_joint__);
    (*this)(1,4) = ((( 0.125 + ( 0.077 *  sin__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) + ( 0.1213 *  sin__q_LR_q1_joint__));
    (*this)(1,5) = (( 0.1213 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__);
    (*this)(2,0) =  sin__q_LR_q2_joint__;
    (*this)(2,1) =  cos__q_LR_q2_joint__;
    (*this)(2,3) = ((( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__);
    (*this)(2,4) = ((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__);
    (*this)(2,5) = ((((- 0.1213 *  cos__q_LR_q2_joint__) - ( 0.125 *  sin__q_LR_q1_joint__)) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    (*this)(3,3) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(3,4) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(3,5) =  cos__q_LR_q1_joint__;
    (*this)(4,3) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(4,4) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(4,5) =  sin__q_LR_q1_joint__;
    (*this)(5,3) =  sin__q_LR_q2_joint__;
    (*this)(5,4) =  cos__q_LR_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_LR_lowerlegCOM::Type_fr_trunk_X_LR_lowerlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_LR_lowerlegCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_LR_lowerlegCOM::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q3_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,1) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(0,3) = (((( 0.15 *  cos__q_LR_q1_joint__) + ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__)) *  sin__q_LR_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,4) = (((((- 0.075 - ( 0.077 *  cos__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.15 *  cos__q_LR_q1_joint__) + ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__)) *  cos__q_LR_q3_joint__)) + ( 0.08853 *  cos__q_LR_q1_joint__));
    (*this)(0,5) = (((((- 0.08853 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - ((( 0.08853 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) - (( 0.15 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__));
    (*this)(1,0) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(1,1) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(1,3) = (((( 0.15 *  sin__q_LR_q1_joint__) + ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__)) *  sin__q_LR_q3_joint__) + ((( 0.125 + ( 0.077 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,4) = (((((- 0.125 - ( 0.077 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.15 *  sin__q_LR_q1_joint__) + ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__)) *  cos__q_LR_q3_joint__)) + ( 0.08853 *  sin__q_LR_q1_joint__));
    (*this)(1,5) = ((((( 0.08853 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.08853 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) + (( 0.15 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__));
    (*this)(2,0) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(2,1) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    (*this)(2,3) = ((((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (((( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(2,4) = ((((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(2,5) = ((((((( 0.08853 *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( 0.08853 *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) - ( 0.15 *  cos__q_LR_q2_joint__)) - ( 0.125 *  sin__q_LR_q1_joint__)) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,4) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,5) =  cos__q_LR_q1_joint__;
    (*this)(4,3) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(4,4) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(4,5) =  sin__q_LR_q1_joint__;
    (*this)(5,3) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(5,4) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_RF_hipassemblyCOM::Type_fr_trunk_X_RF_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_RF_hipassemblyCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_RF_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RF_q1_joint__;
    (*this)(0,1) =  cos__q_RF_q1_joint__;
    (*this)(0,5) = ((- 0.05821 *  cos__q_RF_q1_joint__) -  0.075);
    (*this)(1,0) = - cos__q_RF_q1_joint__;
    (*this)(1,1) =  sin__q_RF_q1_joint__;
    (*this)(1,5) = ((- 0.05821 *  sin__q_RF_q1_joint__) -  0.125);
    (*this)(2,3) = (( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__));
    (*this)(2,4) = ((( 0.125 *  sin__q_RF_q1_joint__) + ( 0.075 *  cos__q_RF_q1_joint__)) +  0.05821);
    (*this)(3,3) =  sin__q_RF_q1_joint__;
    (*this)(3,4) =  cos__q_RF_q1_joint__;
    (*this)(4,3) = - cos__q_RF_q1_joint__;
    (*this)(4,4) =  sin__q_RF_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_RF_upperlegCOM::Type_fr_trunk_X_RF_upperlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_RF_upperlegCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_RF_upperlegCOM::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(0,1) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(0,3) = (((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  sin__q_RF_q2_joint__);
    (*this)(0,4) = (((- 0.075 - ( 0.077 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) - ( 0.1213 *  cos__q_RF_q1_joint__));
    (*this)(0,5) = (( 0.1213 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__);
    (*this)(1,0) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(1,1) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(1,3) = (((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  sin__q_RF_q2_joint__);
    (*this)(1,4) = (((- 0.125 - ( 0.077 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) - ( 0.1213 *  sin__q_RF_q1_joint__));
    (*this)(1,5) = ((- 0.1213 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__);
    (*this)(2,0) =  sin__q_RF_q2_joint__;
    (*this)(2,1) =  cos__q_RF_q2_joint__;
    (*this)(2,3) = ((( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__);
    (*this)(2,4) = ((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__);
    (*this)(2,5) = ((((- 0.1213 *  cos__q_RF_q2_joint__) - ( 0.125 *  sin__q_RF_q1_joint__)) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    (*this)(3,3) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(3,4) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(3,5) = - cos__q_RF_q1_joint__;
    (*this)(4,3) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(4,4) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(4,5) = - sin__q_RF_q1_joint__;
    (*this)(5,3) =  sin__q_RF_q2_joint__;
    (*this)(5,4) =  cos__q_RF_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_RF_lowerlegCOM::Type_fr_trunk_X_RF_lowerlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_RF_lowerlegCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_RF_lowerlegCOM::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q3_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(0,1) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(0,3) = ((((((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(0,4) = ((((( 0.075 + ( 0.077 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q3_joint__)) - ( 0.08853 *  cos__q_RF_q1_joint__));
    (*this)(0,5) = ((((( 0.08853 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + ((( 0.08853 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) + (( 0.15 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__));
    (*this)(1,0) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,1) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(1,3) = ((((((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q3_joint__) + (((- 0.125 - ( 0.077 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,4) = ((((( 0.125 + ( 0.077 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q3_joint__)) - ( 0.08853 *  sin__q_RF_q1_joint__));
    (*this)(1,5) = (((((- 0.08853 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - ((( 0.08853 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) - (( 0.15 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__));
    (*this)(2,0) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(2,1) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    (*this)(2,3) = ((((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(2,4) = ((((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(2,5) = ((((((( 0.08853 *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( 0.08853 *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) - ( 0.15 *  cos__q_RF_q2_joint__)) - ( 0.125 *  sin__q_RF_q1_joint__)) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(3,4) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(3,5) = - cos__q_RF_q1_joint__;
    (*this)(4,3) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,4) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,5) = - sin__q_RF_q1_joint__;
    (*this)(5,3) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(5,4) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_RM_hipassemblyCOM::Type_fr_trunk_X_RM_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_RM_hipassemblyCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_RM_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RM_q1_joint__;
    (*this)(0,1) =  cos__q_RM_q1_joint__;
    (*this)(0,5) = ((- 0.05821 *  cos__q_RM_q1_joint__) -  0.075);
    (*this)(1,0) = - cos__q_RM_q1_joint__;
    (*this)(1,1) =  sin__q_RM_q1_joint__;
    (*this)(1,5) = (- 0.05821 *  sin__q_RM_q1_joint__);
    (*this)(2,3) = ( 0.075 *  sin__q_RM_q1_joint__);
    (*this)(2,4) = (( 0.075 *  cos__q_RM_q1_joint__) +  0.05821);
    (*this)(3,3) =  sin__q_RM_q1_joint__;
    (*this)(3,4) =  cos__q_RM_q1_joint__;
    (*this)(4,3) = - cos__q_RM_q1_joint__;
    (*this)(4,4) =  sin__q_RM_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_RM_upperlegCOM::Type_fr_trunk_X_RM_upperlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_RM_upperlegCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_RM_upperlegCOM::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(0,1) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(0,3) = (((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  sin__q_RM_q2_joint__);
    (*this)(0,4) = (((- 0.075 - ( 0.077 *  cos__q_RM_q1_joint__)) *  cos__q_RM_q2_joint__) - ( 0.1213 *  cos__q_RM_q1_joint__));
    (*this)(0,5) = (( 0.1213 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(1,0) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(1,1) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(1,3) = ((- 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(1,4) = (((- 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) - ( 0.1213 *  sin__q_RM_q1_joint__));
    (*this)(1,5) = ((- 0.1213 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(2,0) =  sin__q_RM_q2_joint__;
    (*this)(2,1) =  cos__q_RM_q2_joint__;
    (*this)(2,3) = (( 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__);
    (*this)(2,4) = ((- 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(2,5) = (((- 0.1213 *  cos__q_RM_q2_joint__) - ( 0.075 *  cos__q_RM_q1_joint__)) -  0.077);
    (*this)(3,3) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(3,4) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(3,5) = - cos__q_RM_q1_joint__;
    (*this)(4,3) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(4,4) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(4,5) = - sin__q_RM_q1_joint__;
    (*this)(5,3) =  sin__q_RM_q2_joint__;
    (*this)(5,4) =  cos__q_RM_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_RM_lowerlegCOM::Type_fr_trunk_X_RM_lowerlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_RM_lowerlegCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_RM_lowerlegCOM::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q3_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(0,1) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(0,3) = ((((((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(0,4) = ((((( 0.075 + ( 0.077 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (((((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__)) *  cos__q_RM_q3_joint__)) - ( 0.08853 *  cos__q_RM_q1_joint__));
    (*this)(0,5) = ((((( 0.08853 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + ((( 0.08853 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) + (( 0.15 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__));
    (*this)(1,0) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,1) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(1,3) = ((((- 0.15 *  sin__q_RM_q1_joint__) - (( 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) *  sin__q_RM_q3_joint__) - ((( 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,4) = ((((( 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (((- 0.15 *  sin__q_RM_q1_joint__) - (( 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) *  cos__q_RM_q3_joint__)) - ( 0.08853 *  sin__q_RM_q1_joint__));
    (*this)(1,5) = (((((- 0.08853 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.08853 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) - (( 0.15 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__));
    (*this)(2,0) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(2,1) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    (*this)(2,3) = (((( 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - ((( 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(2,4) = ((((- 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(2,5) = (((((( 0.08853 *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( 0.08853 *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) - ( 0.15 *  cos__q_RM_q2_joint__)) - ( 0.075 *  cos__q_RM_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(3,4) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(3,5) = - cos__q_RM_q1_joint__;
    (*this)(4,3) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,4) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,5) = - sin__q_RM_q1_joint__;
    (*this)(5,3) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(5,4) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_RR_hipassemblyCOM::Type_fr_trunk_X_RR_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_RR_hipassemblyCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_RR_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RR_q1_joint__;
    (*this)(0,1) =  cos__q_RR_q1_joint__;
    (*this)(0,5) = ((- 0.05821 *  cos__q_RR_q1_joint__) -  0.075);
    (*this)(1,0) = - cos__q_RR_q1_joint__;
    (*this)(1,1) =  sin__q_RR_q1_joint__;
    (*this)(1,5) = ( 0.125 - ( 0.05821 *  sin__q_RR_q1_joint__));
    (*this)(2,3) = (( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__));
    (*this)(2,4) = (((- 0.125 *  sin__q_RR_q1_joint__) + ( 0.075 *  cos__q_RR_q1_joint__)) +  0.05821);
    (*this)(3,3) =  sin__q_RR_q1_joint__;
    (*this)(3,4) =  cos__q_RR_q1_joint__;
    (*this)(4,3) = - cos__q_RR_q1_joint__;
    (*this)(4,4) =  sin__q_RR_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_RR_upperlegCOM::Type_fr_trunk_X_RR_upperlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_RR_upperlegCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_RR_upperlegCOM::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(0,1) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(0,3) = (((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  sin__q_RR_q2_joint__);
    (*this)(0,4) = (((- 0.075 - ( 0.077 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.1213 *  cos__q_RR_q1_joint__));
    (*this)(0,5) = (( 0.1213 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__);
    (*this)(1,0) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(1,1) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(1,3) = (( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__);
    (*this)(1,4) = ((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.1213 *  sin__q_RR_q1_joint__));
    (*this)(1,5) = ((- 0.1213 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__);
    (*this)(2,0) =  sin__q_RR_q2_joint__;
    (*this)(2,1) =  cos__q_RR_q2_joint__;
    (*this)(2,3) = ((( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__);
    (*this)(2,4) = (((- 0.075 *  sin__q_RR_q1_joint__) - ( 0.125 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__);
    (*this)(2,5) = ((((- 0.1213 *  cos__q_RR_q2_joint__) + ( 0.125 *  sin__q_RR_q1_joint__)) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    (*this)(3,3) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(3,4) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(3,5) = - cos__q_RR_q1_joint__;
    (*this)(4,3) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(4,4) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(4,5) = - sin__q_RR_q1_joint__;
    (*this)(5,3) =  sin__q_RR_q2_joint__;
    (*this)(5,4) =  cos__q_RR_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_RR_lowerlegCOM::Type_fr_trunk_X_RR_lowerlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_RR_lowerlegCOM& iit::mcorin::ForceTransforms::Type_fr_trunk_X_RR_lowerlegCOM::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q3_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(0,1) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(0,3) = ((((((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(0,4) = ((((( 0.075 + ( 0.077 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q3_joint__)) - ( 0.08853 *  cos__q_RR_q1_joint__));
    (*this)(0,5) = ((((( 0.08853 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((( 0.08853 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) + (( 0.15 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__));
    (*this)(1,0) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,1) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(1,3) = ((((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q3_joint__) + ((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,4) = (((((( 0.077 *  sin__q_RR_q1_joint__) -  0.125) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q3_joint__)) - ( 0.08853 *  sin__q_RR_q1_joint__));
    (*this)(1,5) = (((((- 0.08853 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - ((( 0.08853 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) - (( 0.15 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__));
    (*this)(2,0) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(2,1) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    (*this)(2,3) = (((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((( 0.125 *  cos__q_RR_q1_joint__) + ( 0.075 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(2,4) = (((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(2,5) = ((((((( 0.08853 *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( 0.08853 *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) - ( 0.15 *  cos__q_RR_q2_joint__)) + ( 0.125 *  sin__q_RR_q1_joint__)) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(3,4) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(3,5) = - cos__q_RR_q1_joint__;
    (*this)(4,3) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,4) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,5) = - sin__q_RR_q1_joint__;
    (*this)(5,3) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(5,4) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LF_upperleg_X_fr_trunk::Type_fr_LF_upperleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_LF_upperleg_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_LF_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(0,1) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(0,2) =  sin__q_LF_q2_joint__;
    (*this)(0,3) = ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  sin__q_LF_q2_joint__);
    (*this)(0,4) = ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  sin__q_LF_q2_joint__);
    (*this)(0,5) = ((( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__);
    (*this)(1,0) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(1,1) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(1,2) =  cos__q_LF_q2_joint__;
    (*this)(1,3) = ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__);
    (*this)(1,4) = ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__);
    (*this)(1,5) = (((- 0.075 *  sin__q_LF_q1_joint__) - ( 0.125 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__);
    (*this)(2,0) =  cos__q_LF_q1_joint__;
    (*this)(2,1) =  sin__q_LF_q1_joint__;
    (*this)(2,5) = ((( 0.125 *  sin__q_LF_q1_joint__) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    (*this)(3,3) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(3,4) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(3,5) =  sin__q_LF_q2_joint__;
    (*this)(4,3) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(4,4) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(4,5) =  cos__q_LF_q2_joint__;
    (*this)(5,3) =  cos__q_LF_q1_joint__;
    (*this)(5,4) =  sin__q_LF_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LF_lowerleg_X_fr_trunk::Type_fr_LF_lowerleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_LF_lowerleg_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_LF_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q3_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,1) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(0,2) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(0,3) = (((( 0.15 *  cos__q_LF_q1_joint__) + ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__)) *  sin__q_LF_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,4) = (((( 0.15 *  sin__q_LF_q1_joint__) + ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__)) *  sin__q_LF_q3_joint__) + (((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,5) = (((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (((( 0.125 *  cos__q_LF_q1_joint__) + ( 0.075 *  sin__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,0) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,1) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,2) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    (*this)(1,3) = ((((- 0.075 - ( 0.077 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.15 *  cos__q_LF_q1_joint__) + ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__)) *  cos__q_LF_q3_joint__));
    (*this)(1,4) = (((( 0.125 - ( 0.077 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.15 *  sin__q_LF_q1_joint__) + ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__)) *  cos__q_LF_q3_joint__));
    (*this)(1,5) = (((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(2,0) =  cos__q_LF_q1_joint__;
    (*this)(2,1) =  sin__q_LF_q1_joint__;
    (*this)(2,3) = ((- 0.15 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__);
    (*this)(2,4) = (( 0.15 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__);
    (*this)(2,5) = ((((- 0.15 *  cos__q_LF_q2_joint__) + ( 0.125 *  sin__q_LF_q1_joint__)) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,4) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(3,5) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(4,3) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(4,4) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(4,5) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    (*this)(5,3) =  cos__q_LF_q1_joint__;
    (*this)(5,4) =  sin__q_LF_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LM_upperleg_X_fr_trunk::Type_fr_LM_upperleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_LM_upperleg_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_LM_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(0,1) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(0,2) =  sin__q_LM_q2_joint__;
    (*this)(0,3) = ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  sin__q_LM_q2_joint__);
    (*this)(0,4) = (( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(0,5) = (( 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__);
    (*this)(1,0) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(1,1) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(1,2) =  cos__q_LM_q2_joint__;
    (*this)(1,3) = ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__);
    (*this)(1,4) = (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__);
    (*this)(1,5) = ((- 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(2,0) =  cos__q_LM_q1_joint__;
    (*this)(2,1) =  sin__q_LM_q1_joint__;
    (*this)(2,5) = ((- 0.075 *  cos__q_LM_q1_joint__) -  0.077);
    (*this)(3,3) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(3,4) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(3,5) =  sin__q_LM_q2_joint__;
    (*this)(4,3) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(4,4) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(4,5) =  cos__q_LM_q2_joint__;
    (*this)(5,3) =  cos__q_LM_q1_joint__;
    (*this)(5,4) =  sin__q_LM_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LM_lowerleg_X_fr_trunk::Type_fr_LM_lowerleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_LM_lowerleg_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_LM_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q3_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,1) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(0,2) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(0,3) = (((( 0.15 *  cos__q_LM_q1_joint__) + ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__)) *  sin__q_LM_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,4) = (((( 0.15 *  sin__q_LM_q1_joint__) + (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) *  sin__q_LM_q3_joint__) + ((( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,5) = (((( 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - ((( 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(1,0) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,1) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,2) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    (*this)(1,3) = ((((- 0.075 - ( 0.077 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.15 *  cos__q_LM_q1_joint__) + ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__)) *  cos__q_LM_q3_joint__));
    (*this)(1,4) = (((( 0.15 *  sin__q_LM_q1_joint__) + (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) *  cos__q_LM_q3_joint__) - ((( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(1,5) = ((((- 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(2,0) =  cos__q_LM_q1_joint__;
    (*this)(2,1) =  sin__q_LM_q1_joint__;
    (*this)(2,3) = ((- 0.15 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(2,4) = (( 0.15 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(2,5) = (((- 0.15 *  cos__q_LM_q2_joint__) - ( 0.075 *  cos__q_LM_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,4) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(3,5) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(4,3) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(4,4) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(4,5) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    (*this)(5,3) =  cos__q_LM_q1_joint__;
    (*this)(5,4) =  sin__q_LM_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LR_upperleg_X_fr_trunk::Type_fr_LR_upperleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_LR_upperleg_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_LR_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(0,1) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(0,2) =  sin__q_LR_q2_joint__;
    (*this)(0,3) = ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  sin__q_LR_q2_joint__);
    (*this)(0,4) = ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  sin__q_LR_q2_joint__);
    (*this)(0,5) = ((( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__);
    (*this)(1,0) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(1,1) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(1,2) =  cos__q_LR_q2_joint__;
    (*this)(1,3) = ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__);
    (*this)(1,4) = ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__);
    (*this)(1,5) = ((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__);
    (*this)(2,0) =  cos__q_LR_q1_joint__;
    (*this)(2,1) =  sin__q_LR_q1_joint__;
    (*this)(2,5) = (((- 0.125 *  sin__q_LR_q1_joint__) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    (*this)(3,3) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(3,4) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(3,5) =  sin__q_LR_q2_joint__;
    (*this)(4,3) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(4,4) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(4,5) =  cos__q_LR_q2_joint__;
    (*this)(5,3) =  cos__q_LR_q1_joint__;
    (*this)(5,4) =  sin__q_LR_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LR_lowerleg_X_fr_trunk::Type_fr_LR_lowerleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_LR_lowerleg_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_LR_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q3_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,1) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(0,2) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(0,3) = (((( 0.15 *  cos__q_LR_q1_joint__) + ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__)) *  sin__q_LR_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,4) = (((( 0.15 *  sin__q_LR_q1_joint__) + ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__)) *  sin__q_LR_q3_joint__) + ((( 0.125 + ( 0.077 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,5) = ((((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (((( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,0) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,1) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,2) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    (*this)(1,3) = ((((- 0.075 - ( 0.077 *  cos__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.15 *  cos__q_LR_q1_joint__) + ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__)) *  cos__q_LR_q3_joint__));
    (*this)(1,4) = ((((- 0.125 - ( 0.077 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.15 *  sin__q_LR_q1_joint__) + ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__)) *  cos__q_LR_q3_joint__));
    (*this)(1,5) = ((((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(2,0) =  cos__q_LR_q1_joint__;
    (*this)(2,1) =  sin__q_LR_q1_joint__;
    (*this)(2,3) = ((- 0.15 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__);
    (*this)(2,4) = (( 0.15 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__);
    (*this)(2,5) = ((((- 0.15 *  cos__q_LR_q2_joint__) - ( 0.125 *  sin__q_LR_q1_joint__)) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,4) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(3,5) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(4,3) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(4,4) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(4,5) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    (*this)(5,3) =  cos__q_LR_q1_joint__;
    (*this)(5,4) =  sin__q_LR_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RF_upperleg_X_fr_trunk::Type_fr_RF_upperleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_RF_upperleg_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_RF_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(0,1) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(0,2) =  sin__q_RF_q2_joint__;
    (*this)(0,3) = (((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  sin__q_RF_q2_joint__);
    (*this)(0,4) = (((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  sin__q_RF_q2_joint__);
    (*this)(0,5) = ((( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__);
    (*this)(1,0) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(1,1) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(1,2) =  cos__q_RF_q2_joint__;
    (*this)(1,3) = (((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__);
    (*this)(1,4) = (((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__);
    (*this)(1,5) = ((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__);
    (*this)(2,0) = - cos__q_RF_q1_joint__;
    (*this)(2,1) = - sin__q_RF_q1_joint__;
    (*this)(2,5) = (((- 0.125 *  sin__q_RF_q1_joint__) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    (*this)(3,3) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(3,4) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(3,5) =  sin__q_RF_q2_joint__;
    (*this)(4,3) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(4,4) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(4,5) =  cos__q_RF_q2_joint__;
    (*this)(5,3) = - cos__q_RF_q1_joint__;
    (*this)(5,4) = - sin__q_RF_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RF_lowerleg_X_fr_trunk::Type_fr_RF_lowerleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_RF_lowerleg_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_RF_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q3_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(0,1) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(0,2) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(0,3) = ((((((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(0,4) = ((((((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q3_joint__) + (((- 0.125 - ( 0.077 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(0,5) = ((((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,0) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,1) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,2) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    (*this)(1,3) = (((( 0.075 + ( 0.077 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q3_joint__));
    (*this)(1,4) = (((( 0.125 + ( 0.077 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q3_joint__));
    (*this)(1,5) = ((((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(2,0) = - cos__q_RF_q1_joint__;
    (*this)(2,1) = - sin__q_RF_q1_joint__;
    (*this)(2,3) = (( 0.15 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__);
    (*this)(2,4) = ((- 0.15 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__);
    (*this)(2,5) = ((((- 0.15 *  cos__q_RF_q2_joint__) - ( 0.125 *  sin__q_RF_q1_joint__)) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(3,4) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(3,5) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(4,3) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,4) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,5) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    (*this)(5,3) = - cos__q_RF_q1_joint__;
    (*this)(5,4) = - sin__q_RF_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RM_upperleg_X_fr_trunk::Type_fr_RM_upperleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_RM_upperleg_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_RM_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(0,1) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(0,2) =  sin__q_RM_q2_joint__;
    (*this)(0,3) = (((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  sin__q_RM_q2_joint__);
    (*this)(0,4) = ((- 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(0,5) = (( 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__);
    (*this)(1,0) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(1,1) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(1,2) =  cos__q_RM_q2_joint__;
    (*this)(1,3) = (((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__);
    (*this)(1,4) = ((- 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__);
    (*this)(1,5) = ((- 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(2,0) = - cos__q_RM_q1_joint__;
    (*this)(2,1) = - sin__q_RM_q1_joint__;
    (*this)(2,5) = ((- 0.075 *  cos__q_RM_q1_joint__) -  0.077);
    (*this)(3,3) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(3,4) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(3,5) =  sin__q_RM_q2_joint__;
    (*this)(4,3) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(4,4) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(4,5) =  cos__q_RM_q2_joint__;
    (*this)(5,3) = - cos__q_RM_q1_joint__;
    (*this)(5,4) = - sin__q_RM_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RM_lowerleg_X_fr_trunk::Type_fr_RM_lowerleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_RM_lowerleg_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_RM_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q3_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(0,1) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(0,2) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(0,3) = ((((((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(0,4) = ((((- 0.15 *  sin__q_RM_q1_joint__) - (( 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) *  sin__q_RM_q3_joint__) - ((( 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(0,5) = (((( 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - ((( 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(1,0) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,1) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,2) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    (*this)(1,3) = (((( 0.075 + ( 0.077 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (((((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__)) *  cos__q_RM_q3_joint__));
    (*this)(1,4) = (((( 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (((- 0.15 *  sin__q_RM_q1_joint__) - (( 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) *  cos__q_RM_q3_joint__));
    (*this)(1,5) = ((((- 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(2,0) = - cos__q_RM_q1_joint__;
    (*this)(2,1) = - sin__q_RM_q1_joint__;
    (*this)(2,3) = (( 0.15 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(2,4) = ((- 0.15 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(2,5) = (((- 0.15 *  cos__q_RM_q2_joint__) - ( 0.075 *  cos__q_RM_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(3,4) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(3,5) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(4,3) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,4) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,5) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    (*this)(5,3) = - cos__q_RM_q1_joint__;
    (*this)(5,4) = - sin__q_RM_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RR_upperleg_X_fr_trunk::Type_fr_RR_upperleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_RR_upperleg_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_RR_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(0,1) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(0,2) =  sin__q_RR_q2_joint__;
    (*this)(0,3) = (((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  sin__q_RR_q2_joint__);
    (*this)(0,4) = (( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__);
    (*this)(0,5) = ((( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__);
    (*this)(1,0) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(1,1) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(1,2) =  cos__q_RR_q2_joint__;
    (*this)(1,3) = (((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__);
    (*this)(1,4) = (( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__);
    (*this)(1,5) = (((- 0.075 *  sin__q_RR_q1_joint__) - ( 0.125 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__);
    (*this)(2,0) = - cos__q_RR_q1_joint__;
    (*this)(2,1) = - sin__q_RR_q1_joint__;
    (*this)(2,5) = ((( 0.125 *  sin__q_RR_q1_joint__) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    (*this)(3,3) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(3,4) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(3,5) =  sin__q_RR_q2_joint__;
    (*this)(4,3) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(4,4) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(4,5) =  cos__q_RR_q2_joint__;
    (*this)(5,3) = - cos__q_RR_q1_joint__;
    (*this)(5,4) = - sin__q_RR_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RR_lowerleg_X_fr_trunk::Type_fr_RR_lowerleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_RR_lowerleg_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_RR_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q3_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(0,1) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(0,2) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(0,3) = ((((((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(0,4) = ((((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q3_joint__) + ((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(0,5) = (((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((( 0.125 *  cos__q_RR_q1_joint__) + ( 0.075 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,0) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,1) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,2) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    (*this)(1,3) = (((( 0.075 + ( 0.077 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q3_joint__));
    (*this)(1,4) = ((((( 0.077 *  sin__q_RR_q1_joint__) -  0.125) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q3_joint__));
    (*this)(1,5) = (((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(2,0) = - cos__q_RR_q1_joint__;
    (*this)(2,1) = - sin__q_RR_q1_joint__;
    (*this)(2,3) = (( 0.15 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__);
    (*this)(2,4) = ((- 0.15 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__);
    (*this)(2,5) = ((((- 0.15 *  cos__q_RR_q2_joint__) + ( 0.125 *  sin__q_RR_q1_joint__)) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(3,4) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(3,5) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(4,3) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,4) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,5) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    (*this)(5,3) = - cos__q_RR_q1_joint__;
    (*this)(5,4) = - sin__q_RR_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_LF_foot::Type_fr_trunk_X_LF_foot()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_LF_foot& iit::mcorin::ForceTransforms::Type_fr_trunk_X_LF_foot::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q3_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,1) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(0,3) = (((( 0.15 *  cos__q_LF_q1_joint__) + ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__)) *  sin__q_LF_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,4) = (((((- 0.075 - ( 0.077 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.15 *  cos__q_LF_q1_joint__) + ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  cos__q_LF_q2_joint__)) *  cos__q_LF_q3_joint__)) + ( 0.17 *  cos__q_LF_q1_joint__));
    (*this)(0,5) = (((((- 0.17 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - ((( 0.17 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) - (( 0.15 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__));
    (*this)(1,0) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(1,1) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(1,3) = (((( 0.15 *  sin__q_LF_q1_joint__) + ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__)) *  sin__q_LF_q3_joint__) + (((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,4) = ((((( 0.125 - ( 0.077 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.15 *  sin__q_LF_q1_joint__) + ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__)) *  cos__q_LF_q3_joint__)) + ( 0.17 *  sin__q_LF_q1_joint__));
    (*this)(1,5) = ((((( 0.17 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.17 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) + (( 0.15 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__));
    (*this)(2,0) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(2,1) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    (*this)(2,3) = (((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (((( 0.125 *  cos__q_LF_q1_joint__) + ( 0.075 *  sin__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(2,4) = (((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((((- 0.125 *  cos__q_LF_q1_joint__) - ( 0.075 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(2,5) = ((((((( 0.17 *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( 0.17 *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) - ( 0.15 *  cos__q_LF_q2_joint__)) + ( 0.125 *  sin__q_LF_q1_joint__)) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,4) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(3,5) =  cos__q_LF_q1_joint__;
    (*this)(4,3) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(4,4) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(4,5) =  sin__q_LF_q1_joint__;
    (*this)(5,3) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(5,4) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LF_q1_joint::Type_fr_trunk_X_fr_LF_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0.075;
    (*this)(1,0) = 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.125;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.125;
    (*this)(2,4) = 0.075;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 1.0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LF_q1_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LF_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LF_q2_joint::Type_fr_trunk_X_fr_LF_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LF_q2_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LF_q2_joint::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LF_q1_joint__;
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(0,4) = (( 0.077 *  cos__q_LF_q1_joint__) +  0.075);
    (*this)(1,0) =  cos__q_LF_q1_joint__;
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(1,4) = (( 0.077 *  sin__q_LF_q1_joint__) -  0.125);
    (*this)(2,3) = (( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__));
    (*this)(2,5) = ((( 0.125 *  sin__q_LF_q1_joint__) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    (*this)(3,3) = - sin__q_LF_q1_joint__;
    (*this)(3,5) =  cos__q_LF_q1_joint__;
    (*this)(4,3) =  cos__q_LF_q1_joint__;
    (*this)(4,5) =  sin__q_LF_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LF_q3_joint::Type_fr_trunk_X_fr_LF_q3_joint()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LF_q3_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LF_q3_joint::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(0,1) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(0,3) = ((( 0.077 *  cos__q_LF_q1_joint__) +  0.075) *  sin__q_LF_q2_joint__);
    (*this)(0,4) = ((( 0.075 + ( 0.077 *  cos__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__) + ( 0.15 *  cos__q_LF_q1_joint__));
    (*this)(0,5) = ((- 0.15 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__);
    (*this)(1,0) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(1,1) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(1,3) = ((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  sin__q_LF_q2_joint__);
    (*this)(1,4) = (((( 0.077 *  sin__q_LF_q1_joint__) -  0.125) *  cos__q_LF_q2_joint__) + ( 0.15 *  sin__q_LF_q1_joint__));
    (*this)(1,5) = (( 0.15 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__);
    (*this)(2,0) =  sin__q_LF_q2_joint__;
    (*this)(2,1) =  cos__q_LF_q2_joint__;
    (*this)(2,3) = ((( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__)) *  cos__q_LF_q2_joint__);
    (*this)(2,4) = (((- 0.075 *  sin__q_LF_q1_joint__) - ( 0.125 *  cos__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__);
    (*this)(2,5) = ((((- 0.15 *  cos__q_LF_q2_joint__) + ( 0.125 *  sin__q_LF_q1_joint__)) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077);
    (*this)(3,3) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(3,4) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(3,5) =  cos__q_LF_q1_joint__;
    (*this)(4,3) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(4,4) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(4,5) =  sin__q_LF_q1_joint__;
    (*this)(5,3) =  sin__q_LF_q2_joint__;
    (*this)(5,4) =  cos__q_LF_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_LM_foot::Type_fr_trunk_X_LM_foot()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_LM_foot& iit::mcorin::ForceTransforms::Type_fr_trunk_X_LM_foot::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q3_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,1) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(0,3) = (((( 0.15 *  cos__q_LM_q1_joint__) + ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__)) *  sin__q_LM_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,4) = (((((- 0.075 - ( 0.077 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.15 *  cos__q_LM_q1_joint__) + ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  cos__q_LM_q2_joint__)) *  cos__q_LM_q3_joint__)) + ( 0.17 *  cos__q_LM_q1_joint__));
    (*this)(0,5) = (((((- 0.17 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.17 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) - (( 0.15 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__));
    (*this)(1,0) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(1,1) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(1,3) = (((( 0.15 *  sin__q_LM_q1_joint__) + (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) *  sin__q_LM_q3_joint__) + ((( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,4) = (((((- 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.15 *  sin__q_LM_q1_joint__) + (( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) *  cos__q_LM_q3_joint__)) + ( 0.17 *  sin__q_LM_q1_joint__));
    (*this)(1,5) = ((((( 0.17 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.17 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) + (( 0.15 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__));
    (*this)(2,0) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(2,1) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    (*this)(2,3) = (((( 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - ((( 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(2,4) = ((((- 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(2,5) = (((((( 0.17 *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( 0.17 *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) - ( 0.15 *  cos__q_LM_q2_joint__)) - ( 0.075 *  cos__q_LM_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,4) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(3,5) =  cos__q_LM_q1_joint__;
    (*this)(4,3) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(4,4) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(4,5) =  sin__q_LM_q1_joint__;
    (*this)(5,3) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(5,4) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LM_q1_joint::Type_fr_trunk_X_fr_LM_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0.075;
    (*this)(1,0) = 1;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0.075;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 1;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LM_q1_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LM_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LM_q2_joint::Type_fr_trunk_X_fr_LM_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LM_q2_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LM_q2_joint::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LM_q1_joint__;
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(0,4) = (( 0.077 *  cos__q_LM_q1_joint__) +  0.075);
    (*this)(1,0) =  cos__q_LM_q1_joint__;
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(1,4) = ( 0.077 *  sin__q_LM_q1_joint__);
    (*this)(2,3) = ( 0.075 *  sin__q_LM_q1_joint__);
    (*this)(2,5) = ((- 0.075 *  cos__q_LM_q1_joint__) -  0.077);
    (*this)(3,3) = - sin__q_LM_q1_joint__;
    (*this)(3,5) =  cos__q_LM_q1_joint__;
    (*this)(4,3) =  cos__q_LM_q1_joint__;
    (*this)(4,5) =  sin__q_LM_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LM_q3_joint::Type_fr_trunk_X_fr_LM_q3_joint()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LM_q3_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LM_q3_joint::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(0,1) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(0,3) = ((( 0.077 *  cos__q_LM_q1_joint__) +  0.075) *  sin__q_LM_q2_joint__);
    (*this)(0,4) = ((( 0.075 + ( 0.077 *  cos__q_LM_q1_joint__)) *  cos__q_LM_q2_joint__) + ( 0.15 *  cos__q_LM_q1_joint__));
    (*this)(0,5) = ((- 0.15 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(1,0) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(1,1) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(1,3) = (( 0.077 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(1,4) = ((( 0.077 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) + ( 0.15 *  sin__q_LM_q1_joint__));
    (*this)(1,5) = (( 0.15 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(2,0) =  sin__q_LM_q2_joint__;
    (*this)(2,1) =  cos__q_LM_q2_joint__;
    (*this)(2,3) = (( 0.075 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__);
    (*this)(2,4) = ((- 0.075 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__);
    (*this)(2,5) = (((- 0.15 *  cos__q_LM_q2_joint__) - ( 0.075 *  cos__q_LM_q1_joint__)) -  0.077);
    (*this)(3,3) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(3,4) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(3,5) =  cos__q_LM_q1_joint__;
    (*this)(4,3) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(4,4) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(4,5) =  sin__q_LM_q1_joint__;
    (*this)(5,3) =  sin__q_LM_q2_joint__;
    (*this)(5,4) =  cos__q_LM_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_LR_foot::Type_fr_trunk_X_LR_foot()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_LR_foot& iit::mcorin::ForceTransforms::Type_fr_trunk_X_LR_foot::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q3_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,1) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(0,3) = (((( 0.15 *  cos__q_LR_q1_joint__) + ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__)) *  sin__q_LR_q3_joint__) + ((( 0.075 + ( 0.077 *  cos__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,4) = (((((- 0.075 - ( 0.077 *  cos__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.15 *  cos__q_LR_q1_joint__) + ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  cos__q_LR_q2_joint__)) *  cos__q_LR_q3_joint__)) + ( 0.17 *  cos__q_LR_q1_joint__));
    (*this)(0,5) = (((((- 0.17 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - ((( 0.17 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) - (( 0.15 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__));
    (*this)(1,0) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(1,1) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(1,3) = (((( 0.15 *  sin__q_LR_q1_joint__) + ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__)) *  sin__q_LR_q3_joint__) + ((( 0.125 + ( 0.077 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,4) = (((((- 0.125 - ( 0.077 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.15 *  sin__q_LR_q1_joint__) + ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  cos__q_LR_q2_joint__)) *  cos__q_LR_q3_joint__)) + ( 0.17 *  sin__q_LR_q1_joint__));
    (*this)(1,5) = ((((( 0.17 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.17 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) + (( 0.15 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__));
    (*this)(2,0) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(2,1) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    (*this)(2,3) = ((((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (((( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(2,4) = ((((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(2,5) = ((((((( 0.17 *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( 0.17 *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) - ( 0.15 *  cos__q_LR_q2_joint__)) - ( 0.125 *  sin__q_LR_q1_joint__)) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,4) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(3,5) =  cos__q_LR_q1_joint__;
    (*this)(4,3) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(4,4) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(4,5) =  sin__q_LR_q1_joint__;
    (*this)(5,3) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(5,4) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LR_q1_joint::Type_fr_trunk_X_fr_LR_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0.075;
    (*this)(1,0) = 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0.125;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.125;
    (*this)(2,4) = 0.075;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = - 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 1.0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LR_q1_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LR_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LR_q2_joint::Type_fr_trunk_X_fr_LR_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LR_q2_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LR_q2_joint::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LR_q1_joint__;
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(0,4) = (( 0.077 *  cos__q_LR_q1_joint__) +  0.075);
    (*this)(1,0) =  cos__q_LR_q1_joint__;
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(1,4) = (( 0.077 *  sin__q_LR_q1_joint__) +  0.125);
    (*this)(2,3) = (( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__));
    (*this)(2,5) = (((- 0.125 *  sin__q_LR_q1_joint__) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    (*this)(3,3) = - sin__q_LR_q1_joint__;
    (*this)(3,5) =  cos__q_LR_q1_joint__;
    (*this)(4,3) =  cos__q_LR_q1_joint__;
    (*this)(4,5) =  sin__q_LR_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LR_q3_joint::Type_fr_trunk_X_fr_LR_q3_joint()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LR_q3_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LR_q3_joint::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(0,1) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(0,3) = ((( 0.077 *  cos__q_LR_q1_joint__) +  0.075) *  sin__q_LR_q2_joint__);
    (*this)(0,4) = ((( 0.075 + ( 0.077 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) + ( 0.15 *  cos__q_LR_q1_joint__));
    (*this)(0,5) = ((- 0.15 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__);
    (*this)(1,0) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(1,1) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(1,3) = ((( 0.077 *  sin__q_LR_q1_joint__) +  0.125) *  sin__q_LR_q2_joint__);
    (*this)(1,4) = ((( 0.125 + ( 0.077 *  sin__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__) + ( 0.15 *  sin__q_LR_q1_joint__));
    (*this)(1,5) = (( 0.15 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__);
    (*this)(2,0) =  sin__q_LR_q2_joint__;
    (*this)(2,1) =  cos__q_LR_q2_joint__;
    (*this)(2,3) = ((( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__)) *  cos__q_LR_q2_joint__);
    (*this)(2,4) = ((( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__);
    (*this)(2,5) = ((((- 0.15 *  cos__q_LR_q2_joint__) - ( 0.125 *  sin__q_LR_q1_joint__)) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077);
    (*this)(3,3) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(3,4) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(3,5) =  cos__q_LR_q1_joint__;
    (*this)(4,3) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(4,4) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(4,5) =  sin__q_LR_q1_joint__;
    (*this)(5,3) =  sin__q_LR_q2_joint__;
    (*this)(5,4) =  cos__q_LR_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_RF_foot::Type_fr_trunk_X_RF_foot()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_RF_foot& iit::mcorin::ForceTransforms::Type_fr_trunk_X_RF_foot::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q3_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(0,1) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(0,3) = ((((((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(0,4) = ((((( 0.075 + ( 0.077 *  cos__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q3_joint__)) - ( 0.17 *  cos__q_RF_q1_joint__));
    (*this)(0,5) = ((((( 0.17 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + ((( 0.17 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) + (( 0.15 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__));
    (*this)(1,0) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,1) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(1,3) = ((((((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q3_joint__) + (((- 0.125 - ( 0.077 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,4) = ((((( 0.125 + ( 0.077 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q3_joint__)) - ( 0.17 *  sin__q_RF_q1_joint__));
    (*this)(1,5) = (((((- 0.17 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - ((( 0.17 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) - (( 0.15 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__));
    (*this)(2,0) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(2,1) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    (*this)(2,3) = ((((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(2,4) = ((((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(2,5) = ((((((( 0.17 *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( 0.17 *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) - ( 0.15 *  cos__q_RF_q2_joint__)) - ( 0.125 *  sin__q_RF_q1_joint__)) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(3,4) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(3,5) = - cos__q_RF_q1_joint__;
    (*this)(4,3) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,4) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,5) = - sin__q_RF_q1_joint__;
    (*this)(5,3) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(5,4) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RF_q1_joint::Type_fr_trunk_X_fr_RF_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = - 0.075;
    (*this)(1,0) = - 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.125;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.125;
    (*this)(2,4) = 0.075;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = - 1.0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RF_q1_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RF_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RF_q2_joint::Type_fr_trunk_X_fr_RF_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RF_q2_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RF_q2_joint::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RF_q1_joint__;
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(0,4) = ((- 0.077 *  cos__q_RF_q1_joint__) -  0.075);
    (*this)(1,0) = - cos__q_RF_q1_joint__;
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(1,4) = ((- 0.077 *  sin__q_RF_q1_joint__) -  0.125);
    (*this)(2,3) = (( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__));
    (*this)(2,5) = (((- 0.125 *  sin__q_RF_q1_joint__) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    (*this)(3,3) =  sin__q_RF_q1_joint__;
    (*this)(3,5) = - cos__q_RF_q1_joint__;
    (*this)(4,3) = - cos__q_RF_q1_joint__;
    (*this)(4,5) = - sin__q_RF_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RF_q3_joint::Type_fr_trunk_X_fr_RF_q3_joint()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RF_q3_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RF_q3_joint::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(0,1) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(0,3) = (((- 0.077 *  cos__q_RF_q1_joint__) -  0.075) *  sin__q_RF_q2_joint__);
    (*this)(0,4) = (((- 0.075 - ( 0.077 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) - ( 0.15 *  cos__q_RF_q1_joint__));
    (*this)(0,5) = (( 0.15 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__);
    (*this)(1,0) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(1,1) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(1,3) = (((- 0.077 *  sin__q_RF_q1_joint__) -  0.125) *  sin__q_RF_q2_joint__);
    (*this)(1,4) = (((- 0.125 - ( 0.077 *  sin__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__) - ( 0.15 *  sin__q_RF_q1_joint__));
    (*this)(1,5) = ((- 0.15 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__);
    (*this)(2,0) =  sin__q_RF_q2_joint__;
    (*this)(2,1) =  cos__q_RF_q2_joint__;
    (*this)(2,3) = ((( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__)) *  cos__q_RF_q2_joint__);
    (*this)(2,4) = ((( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__);
    (*this)(2,5) = ((((- 0.15 *  cos__q_RF_q2_joint__) - ( 0.125 *  sin__q_RF_q1_joint__)) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077);
    (*this)(3,3) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(3,4) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(3,5) = - cos__q_RF_q1_joint__;
    (*this)(4,3) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(4,4) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(4,5) = - sin__q_RF_q1_joint__;
    (*this)(5,3) =  sin__q_RF_q2_joint__;
    (*this)(5,4) =  cos__q_RF_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_RM_foot::Type_fr_trunk_X_RM_foot()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_RM_foot& iit::mcorin::ForceTransforms::Type_fr_trunk_X_RM_foot::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q3_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(0,1) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(0,3) = ((((((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(0,4) = ((((( 0.075 + ( 0.077 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (((((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__)) *  cos__q_RM_q3_joint__)) - ( 0.17 *  cos__q_RM_q1_joint__));
    (*this)(0,5) = ((((( 0.17 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + ((( 0.17 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) + (( 0.15 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__));
    (*this)(1,0) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,1) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(1,3) = ((((- 0.15 *  sin__q_RM_q1_joint__) - (( 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) *  sin__q_RM_q3_joint__) - ((( 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,4) = ((((( 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (((- 0.15 *  sin__q_RM_q1_joint__) - (( 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) *  cos__q_RM_q3_joint__)) - ( 0.17 *  sin__q_RM_q1_joint__));
    (*this)(1,5) = (((((- 0.17 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.17 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) - (( 0.15 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__));
    (*this)(2,0) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(2,1) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    (*this)(2,3) = (((( 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - ((( 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(2,4) = ((((- 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(2,5) = (((((( 0.17 *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( 0.17 *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) - ( 0.15 *  cos__q_RM_q2_joint__)) - ( 0.075 *  cos__q_RM_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(3,4) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(3,5) = - cos__q_RM_q1_joint__;
    (*this)(4,3) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,4) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,5) = - sin__q_RM_q1_joint__;
    (*this)(5,3) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(5,4) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RM_q1_joint::Type_fr_trunk_X_fr_RM_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = - 0.075;
    (*this)(1,0) = - 1;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0.075;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 1;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = - 1;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RM_q1_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RM_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RM_q2_joint::Type_fr_trunk_X_fr_RM_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RM_q2_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RM_q2_joint::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RM_q1_joint__;
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(0,4) = ((- 0.077 *  cos__q_RM_q1_joint__) -  0.075);
    (*this)(1,0) = - cos__q_RM_q1_joint__;
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(1,4) = (- 0.077 *  sin__q_RM_q1_joint__);
    (*this)(2,3) = ( 0.075 *  sin__q_RM_q1_joint__);
    (*this)(2,5) = ((- 0.075 *  cos__q_RM_q1_joint__) -  0.077);
    (*this)(3,3) =  sin__q_RM_q1_joint__;
    (*this)(3,5) = - cos__q_RM_q1_joint__;
    (*this)(4,3) = - cos__q_RM_q1_joint__;
    (*this)(4,5) = - sin__q_RM_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RM_q3_joint::Type_fr_trunk_X_fr_RM_q3_joint()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RM_q3_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RM_q3_joint::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(0,1) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(0,3) = (((- 0.077 *  cos__q_RM_q1_joint__) -  0.075) *  sin__q_RM_q2_joint__);
    (*this)(0,4) = (((- 0.075 - ( 0.077 *  cos__q_RM_q1_joint__)) *  cos__q_RM_q2_joint__) - ( 0.15 *  cos__q_RM_q1_joint__));
    (*this)(0,5) = (( 0.15 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(1,0) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(1,1) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(1,3) = ((- 0.077 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(1,4) = (((- 0.077 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) - ( 0.15 *  sin__q_RM_q1_joint__));
    (*this)(1,5) = ((- 0.15 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(2,0) =  sin__q_RM_q2_joint__;
    (*this)(2,1) =  cos__q_RM_q2_joint__;
    (*this)(2,3) = (( 0.075 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__);
    (*this)(2,4) = ((- 0.075 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__);
    (*this)(2,5) = (((- 0.15 *  cos__q_RM_q2_joint__) - ( 0.075 *  cos__q_RM_q1_joint__)) -  0.077);
    (*this)(3,3) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(3,4) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(3,5) = - cos__q_RM_q1_joint__;
    (*this)(4,3) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(4,4) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(4,5) = - sin__q_RM_q1_joint__;
    (*this)(5,3) =  sin__q_RM_q2_joint__;
    (*this)(5,4) =  cos__q_RM_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_RR_foot::Type_fr_trunk_X_RR_foot()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_RR_foot& iit::mcorin::ForceTransforms::Type_fr_trunk_X_RR_foot::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q3_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(0,1) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(0,3) = ((((((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q3_joint__) + (((- 0.075 - ( 0.077 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(0,4) = ((((( 0.075 + ( 0.077 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q3_joint__)) - ( 0.17 *  cos__q_RR_q1_joint__));
    (*this)(0,5) = ((((( 0.17 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((( 0.17 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) + (( 0.15 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__));
    (*this)(1,0) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,1) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(1,3) = ((((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q3_joint__) + ((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,4) = (((((( 0.077 *  sin__q_RR_q1_joint__) -  0.125) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q3_joint__)) - ( 0.17 *  sin__q_RR_q1_joint__));
    (*this)(1,5) = (((((- 0.17 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - ((( 0.17 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) - (( 0.15 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__));
    (*this)(2,0) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(2,1) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    (*this)(2,3) = (((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((( 0.125 *  cos__q_RR_q1_joint__) + ( 0.075 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(2,4) = (((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((((- 0.125 *  cos__q_RR_q1_joint__) - ( 0.075 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(2,5) = ((((((( 0.17 *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( 0.17 *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) - ( 0.15 *  cos__q_RR_q2_joint__)) + ( 0.125 *  sin__q_RR_q1_joint__)) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    (*this)(3,3) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(3,4) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(3,5) = - cos__q_RR_q1_joint__;
    (*this)(4,3) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,4) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,5) = - sin__q_RR_q1_joint__;
    (*this)(5,3) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(5,4) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RR_q1_joint::Type_fr_trunk_X_fr_RR_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = - 0.075;
    (*this)(1,0) = - 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0.125;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.125;
    (*this)(2,4) = 0.075;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 1.0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = - 1.0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RR_q1_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RR_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RR_q2_joint::Type_fr_trunk_X_fr_RR_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RR_q2_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RR_q2_joint::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RR_q1_joint__;
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(0,4) = ((- 0.077 *  cos__q_RR_q1_joint__) -  0.075);
    (*this)(1,0) = - cos__q_RR_q1_joint__;
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(1,4) = ( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__));
    (*this)(2,3) = (( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__));
    (*this)(2,5) = ((( 0.125 *  sin__q_RR_q1_joint__) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    (*this)(3,3) =  sin__q_RR_q1_joint__;
    (*this)(3,5) = - cos__q_RR_q1_joint__;
    (*this)(4,3) = - cos__q_RR_q1_joint__;
    (*this)(4,5) = - sin__q_RR_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RR_q3_joint::Type_fr_trunk_X_fr_RR_q3_joint()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RR_q3_joint& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RR_q3_joint::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(0,1) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(0,3) = (((- 0.077 *  cos__q_RR_q1_joint__) -  0.075) *  sin__q_RR_q2_joint__);
    (*this)(0,4) = (((- 0.075 - ( 0.077 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  cos__q_RR_q1_joint__));
    (*this)(0,5) = (( 0.15 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__);
    (*this)(1,0) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(1,1) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(1,3) = (( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__);
    (*this)(1,4) = ((( 0.125 - ( 0.077 *  sin__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__) - ( 0.15 *  sin__q_RR_q1_joint__));
    (*this)(1,5) = ((- 0.15 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__);
    (*this)(2,0) =  sin__q_RR_q2_joint__;
    (*this)(2,1) =  cos__q_RR_q2_joint__;
    (*this)(2,3) = ((( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__)) *  cos__q_RR_q2_joint__);
    (*this)(2,4) = (((- 0.075 *  sin__q_RR_q1_joint__) - ( 0.125 *  cos__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__);
    (*this)(2,5) = ((((- 0.15 *  cos__q_RR_q2_joint__) + ( 0.125 *  sin__q_RR_q1_joint__)) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077);
    (*this)(3,3) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(3,4) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(3,5) = - cos__q_RR_q1_joint__;
    (*this)(4,3) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(4,4) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(4,5) = - sin__q_RR_q1_joint__;
    (*this)(5,3) =  sin__q_RR_q2_joint__;
    (*this)(5,4) =  cos__q_RR_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LF_hipassembly_X_fr_trunk::Type_fr_LF_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.075;
    (*this)(2,4) = - 0.125;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_LF_hipassembly_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_LF_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LF_q1_joint__;
    (*this)(0,1) =  cos__q_LF_q1_joint__;
    (*this)(0,5) = (( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__));
    (*this)(1,0) = - cos__q_LF_q1_joint__;
    (*this)(1,1) = - sin__q_LF_q1_joint__;
    (*this)(1,5) = (( 0.075 *  cos__q_LF_q1_joint__) - ( 0.125 *  sin__q_LF_q1_joint__));
    (*this)(3,3) = - sin__q_LF_q1_joint__;
    (*this)(3,4) =  cos__q_LF_q1_joint__;
    (*this)(4,3) = - cos__q_LF_q1_joint__;
    (*this)(4,4) = - sin__q_LF_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LF_hipassembly::Type_fr_trunk_X_fr_LF_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0.075;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.125;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LF_hipassembly& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LF_hipassembly::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LF_q1_joint__;
    (*this)(0,1) = - cos__q_LF_q1_joint__;
    (*this)(1,0) =  cos__q_LF_q1_joint__;
    (*this)(1,1) = - sin__q_LF_q1_joint__;
    (*this)(2,3) = (( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__));
    (*this)(2,4) = (( 0.075 *  cos__q_LF_q1_joint__) - ( 0.125 *  sin__q_LF_q1_joint__));
    (*this)(3,3) = - sin__q_LF_q1_joint__;
    (*this)(3,4) = - cos__q_LF_q1_joint__;
    (*this)(4,3) =  cos__q_LF_q1_joint__;
    (*this)(4,4) = - sin__q_LF_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LF_upperleg_X_fr_LF_hipassembly::Type_fr_LF_upperleg_X_fr_LF_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.077;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_LF_upperleg_X_fr_LF_hipassembly& iit::mcorin::ForceTransforms::Type_fr_LF_upperleg_X_fr_LF_hipassembly::update(const state_t& q) {
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LF_q2_joint__;
    (*this)(0,2) =  sin__q_LF_q2_joint__;
    (*this)(0,4) = (- 0.077 *  sin__q_LF_q2_joint__);
    (*this)(1,0) = - sin__q_LF_q2_joint__;
    (*this)(1,2) =  cos__q_LF_q2_joint__;
    (*this)(1,4) = (- 0.077 *  cos__q_LF_q2_joint__);
    (*this)(3,3) =  cos__q_LF_q2_joint__;
    (*this)(3,5) =  sin__q_LF_q2_joint__;
    (*this)(4,3) = - sin__q_LF_q2_joint__;
    (*this)(4,5) =  cos__q_LF_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LF_hipassembly_X_fr_LF_upperleg::Type_fr_LF_hipassembly_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.077;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_LF_hipassembly_X_fr_LF_upperleg& iit::mcorin::ForceTransforms::Type_fr_LF_hipassembly_X_fr_LF_upperleg::update(const state_t& q) {
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LF_q2_joint__;
    (*this)(0,1) = - sin__q_LF_q2_joint__;
    (*this)(1,3) = (- 0.077 *  sin__q_LF_q2_joint__);
    (*this)(1,4) = (- 0.077 *  cos__q_LF_q2_joint__);
    (*this)(2,0) =  sin__q_LF_q2_joint__;
    (*this)(2,1) =  cos__q_LF_q2_joint__;
    (*this)(3,3) =  cos__q_LF_q2_joint__;
    (*this)(3,4) = - sin__q_LF_q2_joint__;
    (*this)(5,3) =  sin__q_LF_q2_joint__;
    (*this)(5,4) =  cos__q_LF_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LF_lowerleg_X_fr_LF_upperleg::Type_fr_LF_lowerleg_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.15;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_LF_lowerleg_X_fr_LF_upperleg& iit::mcorin::ForceTransforms::Type_fr_LF_lowerleg_X_fr_LF_upperleg::update(const state_t& q) {
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q3_joint__;
    
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LF_q3_joint__;
    (*this)(0,1) =  sin__q_LF_q3_joint__;
    (*this)(0,5) = ( 0.15 *  sin__q_LF_q3_joint__);
    (*this)(1,0) = - sin__q_LF_q3_joint__;
    (*this)(1,1) =  cos__q_LF_q3_joint__;
    (*this)(1,5) = ( 0.15 *  cos__q_LF_q3_joint__);
    (*this)(3,3) =  cos__q_LF_q3_joint__;
    (*this)(3,4) =  sin__q_LF_q3_joint__;
    (*this)(4,3) = - sin__q_LF_q3_joint__;
    (*this)(4,4) =  cos__q_LF_q3_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LF_upperleg_X_fr_LF_lowerleg::Type_fr_LF_upperleg_X_fr_LF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.15;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::ForceTransforms::Type_fr_LF_upperleg_X_fr_LF_lowerleg& iit::mcorin::ForceTransforms::Type_fr_LF_upperleg_X_fr_LF_lowerleg::update(const state_t& q) {
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q3_joint__;
    
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LF_q3_joint__;
    (*this)(0,1) = - sin__q_LF_q3_joint__;
    (*this)(1,0) =  sin__q_LF_q3_joint__;
    (*this)(1,1) =  cos__q_LF_q3_joint__;
    (*this)(2,3) = ( 0.15 *  sin__q_LF_q3_joint__);
    (*this)(2,4) = ( 0.15 *  cos__q_LF_q3_joint__);
    (*this)(3,3) =  cos__q_LF_q3_joint__;
    (*this)(3,4) = - sin__q_LF_q3_joint__;
    (*this)(4,3) =  sin__q_LF_q3_joint__;
    (*this)(4,4) =  cos__q_LF_q3_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LM_hipassembly_X_fr_trunk::Type_fr_LM_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0.075;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_LM_hipassembly_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_LM_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LM_q1_joint__;
    (*this)(0,1) =  cos__q_LM_q1_joint__;
    (*this)(0,5) = ( 0.075 *  sin__q_LM_q1_joint__);
    (*this)(1,0) = - cos__q_LM_q1_joint__;
    (*this)(1,1) = - sin__q_LM_q1_joint__;
    (*this)(1,5) = ( 0.075 *  cos__q_LM_q1_joint__);
    (*this)(3,3) = - sin__q_LM_q1_joint__;
    (*this)(3,4) =  cos__q_LM_q1_joint__;
    (*this)(4,3) = - cos__q_LM_q1_joint__;
    (*this)(4,4) = - sin__q_LM_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LM_hipassembly::Type_fr_trunk_X_fr_LM_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0.075;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LM_hipassembly& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LM_hipassembly::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LM_q1_joint__;
    (*this)(0,1) = - cos__q_LM_q1_joint__;
    (*this)(1,0) =  cos__q_LM_q1_joint__;
    (*this)(1,1) = - sin__q_LM_q1_joint__;
    (*this)(2,3) = ( 0.075 *  sin__q_LM_q1_joint__);
    (*this)(2,4) = ( 0.075 *  cos__q_LM_q1_joint__);
    (*this)(3,3) = - sin__q_LM_q1_joint__;
    (*this)(3,4) = - cos__q_LM_q1_joint__;
    (*this)(4,3) =  cos__q_LM_q1_joint__;
    (*this)(4,4) = - sin__q_LM_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LM_upperleg_X_fr_LM_hipassembly::Type_fr_LM_upperleg_X_fr_LM_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.077;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_LM_upperleg_X_fr_LM_hipassembly& iit::mcorin::ForceTransforms::Type_fr_LM_upperleg_X_fr_LM_hipassembly::update(const state_t& q) {
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LM_q2_joint__;
    (*this)(0,2) =  sin__q_LM_q2_joint__;
    (*this)(0,4) = (- 0.077 *  sin__q_LM_q2_joint__);
    (*this)(1,0) = - sin__q_LM_q2_joint__;
    (*this)(1,2) =  cos__q_LM_q2_joint__;
    (*this)(1,4) = (- 0.077 *  cos__q_LM_q2_joint__);
    (*this)(3,3) =  cos__q_LM_q2_joint__;
    (*this)(3,5) =  sin__q_LM_q2_joint__;
    (*this)(4,3) = - sin__q_LM_q2_joint__;
    (*this)(4,5) =  cos__q_LM_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LM_hipassembly_X_fr_LM_upperleg::Type_fr_LM_hipassembly_X_fr_LM_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.077;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_LM_hipassembly_X_fr_LM_upperleg& iit::mcorin::ForceTransforms::Type_fr_LM_hipassembly_X_fr_LM_upperleg::update(const state_t& q) {
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LM_q2_joint__;
    (*this)(0,1) = - sin__q_LM_q2_joint__;
    (*this)(1,3) = (- 0.077 *  sin__q_LM_q2_joint__);
    (*this)(1,4) = (- 0.077 *  cos__q_LM_q2_joint__);
    (*this)(2,0) =  sin__q_LM_q2_joint__;
    (*this)(2,1) =  cos__q_LM_q2_joint__;
    (*this)(3,3) =  cos__q_LM_q2_joint__;
    (*this)(3,4) = - sin__q_LM_q2_joint__;
    (*this)(5,3) =  sin__q_LM_q2_joint__;
    (*this)(5,4) =  cos__q_LM_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LM_lowerleg_X_fr_LM_upperleg::Type_fr_LM_lowerleg_X_fr_LM_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.15;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_LM_lowerleg_X_fr_LM_upperleg& iit::mcorin::ForceTransforms::Type_fr_LM_lowerleg_X_fr_LM_upperleg::update(const state_t& q) {
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q3_joint__;
    
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LM_q3_joint__;
    (*this)(0,1) =  sin__q_LM_q3_joint__;
    (*this)(0,5) = ( 0.15 *  sin__q_LM_q3_joint__);
    (*this)(1,0) = - sin__q_LM_q3_joint__;
    (*this)(1,1) =  cos__q_LM_q3_joint__;
    (*this)(1,5) = ( 0.15 *  cos__q_LM_q3_joint__);
    (*this)(3,3) =  cos__q_LM_q3_joint__;
    (*this)(3,4) =  sin__q_LM_q3_joint__;
    (*this)(4,3) = - sin__q_LM_q3_joint__;
    (*this)(4,4) =  cos__q_LM_q3_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LM_upperleg_X_fr_LM_lowerleg::Type_fr_LM_upperleg_X_fr_LM_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.15;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::ForceTransforms::Type_fr_LM_upperleg_X_fr_LM_lowerleg& iit::mcorin::ForceTransforms::Type_fr_LM_upperleg_X_fr_LM_lowerleg::update(const state_t& q) {
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q3_joint__;
    
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LM_q3_joint__;
    (*this)(0,1) = - sin__q_LM_q3_joint__;
    (*this)(1,0) =  sin__q_LM_q3_joint__;
    (*this)(1,1) =  cos__q_LM_q3_joint__;
    (*this)(2,3) = ( 0.15 *  sin__q_LM_q3_joint__);
    (*this)(2,4) = ( 0.15 *  cos__q_LM_q3_joint__);
    (*this)(3,3) =  cos__q_LM_q3_joint__;
    (*this)(3,4) = - sin__q_LM_q3_joint__;
    (*this)(4,3) =  sin__q_LM_q3_joint__;
    (*this)(4,4) =  cos__q_LM_q3_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LR_hipassembly_X_fr_trunk::Type_fr_LR_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.075;
    (*this)(2,4) = 0.125;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_LR_hipassembly_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_LR_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LR_q1_joint__;
    (*this)(0,1) =  cos__q_LR_q1_joint__;
    (*this)(0,5) = (( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__));
    (*this)(1,0) = - cos__q_LR_q1_joint__;
    (*this)(1,1) = - sin__q_LR_q1_joint__;
    (*this)(1,5) = (( 0.125 *  sin__q_LR_q1_joint__) + ( 0.075 *  cos__q_LR_q1_joint__));
    (*this)(3,3) = - sin__q_LR_q1_joint__;
    (*this)(3,4) =  cos__q_LR_q1_joint__;
    (*this)(4,3) = - cos__q_LR_q1_joint__;
    (*this)(4,4) = - sin__q_LR_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LR_hipassembly::Type_fr_trunk_X_fr_LR_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0.075;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0.125;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LR_hipassembly& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_LR_hipassembly::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LR_q1_joint__;
    (*this)(0,1) = - cos__q_LR_q1_joint__;
    (*this)(1,0) =  cos__q_LR_q1_joint__;
    (*this)(1,1) = - sin__q_LR_q1_joint__;
    (*this)(2,3) = (( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__));
    (*this)(2,4) = (( 0.125 *  sin__q_LR_q1_joint__) + ( 0.075 *  cos__q_LR_q1_joint__));
    (*this)(3,3) = - sin__q_LR_q1_joint__;
    (*this)(3,4) = - cos__q_LR_q1_joint__;
    (*this)(4,3) =  cos__q_LR_q1_joint__;
    (*this)(4,4) = - sin__q_LR_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LR_upperleg_X_fr_LR_hipassembly::Type_fr_LR_upperleg_X_fr_LR_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.077;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_LR_upperleg_X_fr_LR_hipassembly& iit::mcorin::ForceTransforms::Type_fr_LR_upperleg_X_fr_LR_hipassembly::update(const state_t& q) {
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LR_q2_joint__;
    (*this)(0,2) =  sin__q_LR_q2_joint__;
    (*this)(0,4) = (- 0.077 *  sin__q_LR_q2_joint__);
    (*this)(1,0) = - sin__q_LR_q2_joint__;
    (*this)(1,2) =  cos__q_LR_q2_joint__;
    (*this)(1,4) = (- 0.077 *  cos__q_LR_q2_joint__);
    (*this)(3,3) =  cos__q_LR_q2_joint__;
    (*this)(3,5) =  sin__q_LR_q2_joint__;
    (*this)(4,3) = - sin__q_LR_q2_joint__;
    (*this)(4,5) =  cos__q_LR_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LR_hipassembly_X_fr_LR_upperleg::Type_fr_LR_hipassembly_X_fr_LR_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.077;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_LR_hipassembly_X_fr_LR_upperleg& iit::mcorin::ForceTransforms::Type_fr_LR_hipassembly_X_fr_LR_upperleg::update(const state_t& q) {
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LR_q2_joint__;
    (*this)(0,1) = - sin__q_LR_q2_joint__;
    (*this)(1,3) = (- 0.077 *  sin__q_LR_q2_joint__);
    (*this)(1,4) = (- 0.077 *  cos__q_LR_q2_joint__);
    (*this)(2,0) =  sin__q_LR_q2_joint__;
    (*this)(2,1) =  cos__q_LR_q2_joint__;
    (*this)(3,3) =  cos__q_LR_q2_joint__;
    (*this)(3,4) = - sin__q_LR_q2_joint__;
    (*this)(5,3) =  sin__q_LR_q2_joint__;
    (*this)(5,4) =  cos__q_LR_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LR_lowerleg_X_fr_LR_upperleg::Type_fr_LR_lowerleg_X_fr_LR_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.15;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_LR_lowerleg_X_fr_LR_upperleg& iit::mcorin::ForceTransforms::Type_fr_LR_lowerleg_X_fr_LR_upperleg::update(const state_t& q) {
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q3_joint__;
    
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LR_q3_joint__;
    (*this)(0,1) =  sin__q_LR_q3_joint__;
    (*this)(0,5) = ( 0.15 *  sin__q_LR_q3_joint__);
    (*this)(1,0) = - sin__q_LR_q3_joint__;
    (*this)(1,1) =  cos__q_LR_q3_joint__;
    (*this)(1,5) = ( 0.15 *  cos__q_LR_q3_joint__);
    (*this)(3,3) =  cos__q_LR_q3_joint__;
    (*this)(3,4) =  sin__q_LR_q3_joint__;
    (*this)(4,3) = - sin__q_LR_q3_joint__;
    (*this)(4,4) =  cos__q_LR_q3_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_LR_upperleg_X_fr_LR_lowerleg::Type_fr_LR_upperleg_X_fr_LR_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.15;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::ForceTransforms::Type_fr_LR_upperleg_X_fr_LR_lowerleg& iit::mcorin::ForceTransforms::Type_fr_LR_upperleg_X_fr_LR_lowerleg::update(const state_t& q) {
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q3_joint__;
    
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LR_q3_joint__;
    (*this)(0,1) = - sin__q_LR_q3_joint__;
    (*this)(1,0) =  sin__q_LR_q3_joint__;
    (*this)(1,1) =  cos__q_LR_q3_joint__;
    (*this)(2,3) = ( 0.15 *  sin__q_LR_q3_joint__);
    (*this)(2,4) = ( 0.15 *  cos__q_LR_q3_joint__);
    (*this)(3,3) =  cos__q_LR_q3_joint__;
    (*this)(3,4) = - sin__q_LR_q3_joint__;
    (*this)(4,3) =  sin__q_LR_q3_joint__;
    (*this)(4,4) =  cos__q_LR_q3_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RF_hipassembly_X_fr_trunk::Type_fr_RF_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.075;
    (*this)(2,4) = - 0.125;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_RF_hipassembly_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_RF_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RF_q1_joint__;
    (*this)(0,1) = - cos__q_RF_q1_joint__;
    (*this)(0,5) = (( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__));
    (*this)(1,0) =  cos__q_RF_q1_joint__;
    (*this)(1,1) =  sin__q_RF_q1_joint__;
    (*this)(1,5) = (( 0.125 *  sin__q_RF_q1_joint__) + ( 0.075 *  cos__q_RF_q1_joint__));
    (*this)(3,3) =  sin__q_RF_q1_joint__;
    (*this)(3,4) = - cos__q_RF_q1_joint__;
    (*this)(4,3) =  cos__q_RF_q1_joint__;
    (*this)(4,4) =  sin__q_RF_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RF_hipassembly::Type_fr_trunk_X_fr_RF_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = - 0.075;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.125;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RF_hipassembly& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RF_hipassembly::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RF_q1_joint__;
    (*this)(0,1) =  cos__q_RF_q1_joint__;
    (*this)(1,0) = - cos__q_RF_q1_joint__;
    (*this)(1,1) =  sin__q_RF_q1_joint__;
    (*this)(2,3) = (( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__));
    (*this)(2,4) = (( 0.125 *  sin__q_RF_q1_joint__) + ( 0.075 *  cos__q_RF_q1_joint__));
    (*this)(3,3) =  sin__q_RF_q1_joint__;
    (*this)(3,4) =  cos__q_RF_q1_joint__;
    (*this)(4,3) = - cos__q_RF_q1_joint__;
    (*this)(4,4) =  sin__q_RF_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RF_upperleg_X_fr_RF_hipassembly::Type_fr_RF_upperleg_X_fr_RF_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.077;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_RF_upperleg_X_fr_RF_hipassembly& iit::mcorin::ForceTransforms::Type_fr_RF_upperleg_X_fr_RF_hipassembly::update(const state_t& q) {
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RF_q2_joint__;
    (*this)(0,2) =  sin__q_RF_q2_joint__;
    (*this)(0,4) = (- 0.077 *  sin__q_RF_q2_joint__);
    (*this)(1,0) = - sin__q_RF_q2_joint__;
    (*this)(1,2) =  cos__q_RF_q2_joint__;
    (*this)(1,4) = (- 0.077 *  cos__q_RF_q2_joint__);
    (*this)(3,3) =  cos__q_RF_q2_joint__;
    (*this)(3,5) =  sin__q_RF_q2_joint__;
    (*this)(4,3) = - sin__q_RF_q2_joint__;
    (*this)(4,5) =  cos__q_RF_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RF_hipassembly_X_fr_RF_upperleg::Type_fr_RF_hipassembly_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.077;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_RF_hipassembly_X_fr_RF_upperleg& iit::mcorin::ForceTransforms::Type_fr_RF_hipassembly_X_fr_RF_upperleg::update(const state_t& q) {
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RF_q2_joint__;
    (*this)(0,1) = - sin__q_RF_q2_joint__;
    (*this)(1,3) = (- 0.077 *  sin__q_RF_q2_joint__);
    (*this)(1,4) = (- 0.077 *  cos__q_RF_q2_joint__);
    (*this)(2,0) =  sin__q_RF_q2_joint__;
    (*this)(2,1) =  cos__q_RF_q2_joint__;
    (*this)(3,3) =  cos__q_RF_q2_joint__;
    (*this)(3,4) = - sin__q_RF_q2_joint__;
    (*this)(5,3) =  sin__q_RF_q2_joint__;
    (*this)(5,4) =  cos__q_RF_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RF_lowerleg_X_fr_RF_upperleg::Type_fr_RF_lowerleg_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.15;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_RF_lowerleg_X_fr_RF_upperleg& iit::mcorin::ForceTransforms::Type_fr_RF_lowerleg_X_fr_RF_upperleg::update(const state_t& q) {
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q3_joint__;
    
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RF_q3_joint__;
    (*this)(0,1) =  sin__q_RF_q3_joint__;
    (*this)(0,5) = ( 0.15 *  sin__q_RF_q3_joint__);
    (*this)(1,0) = - sin__q_RF_q3_joint__;
    (*this)(1,1) =  cos__q_RF_q3_joint__;
    (*this)(1,5) = ( 0.15 *  cos__q_RF_q3_joint__);
    (*this)(3,3) =  cos__q_RF_q3_joint__;
    (*this)(3,4) =  sin__q_RF_q3_joint__;
    (*this)(4,3) = - sin__q_RF_q3_joint__;
    (*this)(4,4) =  cos__q_RF_q3_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RF_upperleg_X_fr_RF_lowerleg::Type_fr_RF_upperleg_X_fr_RF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.15;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::ForceTransforms::Type_fr_RF_upperleg_X_fr_RF_lowerleg& iit::mcorin::ForceTransforms::Type_fr_RF_upperleg_X_fr_RF_lowerleg::update(const state_t& q) {
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q3_joint__;
    
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RF_q3_joint__;
    (*this)(0,1) = - sin__q_RF_q3_joint__;
    (*this)(1,0) =  sin__q_RF_q3_joint__;
    (*this)(1,1) =  cos__q_RF_q3_joint__;
    (*this)(2,3) = ( 0.15 *  sin__q_RF_q3_joint__);
    (*this)(2,4) = ( 0.15 *  cos__q_RF_q3_joint__);
    (*this)(3,3) =  cos__q_RF_q3_joint__;
    (*this)(3,4) = - sin__q_RF_q3_joint__;
    (*this)(4,3) =  sin__q_RF_q3_joint__;
    (*this)(4,4) =  cos__q_RF_q3_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RM_hipassembly_X_fr_trunk::Type_fr_RM_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = - 0.075;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_RM_hipassembly_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_RM_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RM_q1_joint__;
    (*this)(0,1) = - cos__q_RM_q1_joint__;
    (*this)(0,5) = ( 0.075 *  sin__q_RM_q1_joint__);
    (*this)(1,0) =  cos__q_RM_q1_joint__;
    (*this)(1,1) =  sin__q_RM_q1_joint__;
    (*this)(1,5) = ( 0.075 *  cos__q_RM_q1_joint__);
    (*this)(3,3) =  sin__q_RM_q1_joint__;
    (*this)(3,4) = - cos__q_RM_q1_joint__;
    (*this)(4,3) =  cos__q_RM_q1_joint__;
    (*this)(4,4) =  sin__q_RM_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RM_hipassembly::Type_fr_trunk_X_fr_RM_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = - 0.075;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RM_hipassembly& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RM_hipassembly::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RM_q1_joint__;
    (*this)(0,1) =  cos__q_RM_q1_joint__;
    (*this)(1,0) = - cos__q_RM_q1_joint__;
    (*this)(1,1) =  sin__q_RM_q1_joint__;
    (*this)(2,3) = ( 0.075 *  sin__q_RM_q1_joint__);
    (*this)(2,4) = ( 0.075 *  cos__q_RM_q1_joint__);
    (*this)(3,3) =  sin__q_RM_q1_joint__;
    (*this)(3,4) =  cos__q_RM_q1_joint__;
    (*this)(4,3) = - cos__q_RM_q1_joint__;
    (*this)(4,4) =  sin__q_RM_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RM_upperleg_X_fr_RM_hipassembly::Type_fr_RM_upperleg_X_fr_RM_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.077;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_RM_upperleg_X_fr_RM_hipassembly& iit::mcorin::ForceTransforms::Type_fr_RM_upperleg_X_fr_RM_hipassembly::update(const state_t& q) {
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RM_q2_joint__;
    (*this)(0,2) =  sin__q_RM_q2_joint__;
    (*this)(0,4) = (- 0.077 *  sin__q_RM_q2_joint__);
    (*this)(1,0) = - sin__q_RM_q2_joint__;
    (*this)(1,2) =  cos__q_RM_q2_joint__;
    (*this)(1,4) = (- 0.077 *  cos__q_RM_q2_joint__);
    (*this)(3,3) =  cos__q_RM_q2_joint__;
    (*this)(3,5) =  sin__q_RM_q2_joint__;
    (*this)(4,3) = - sin__q_RM_q2_joint__;
    (*this)(4,5) =  cos__q_RM_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RM_hipassembly_X_fr_RM_upperleg::Type_fr_RM_hipassembly_X_fr_RM_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.077;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_RM_hipassembly_X_fr_RM_upperleg& iit::mcorin::ForceTransforms::Type_fr_RM_hipassembly_X_fr_RM_upperleg::update(const state_t& q) {
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RM_q2_joint__;
    (*this)(0,1) = - sin__q_RM_q2_joint__;
    (*this)(1,3) = (- 0.077 *  sin__q_RM_q2_joint__);
    (*this)(1,4) = (- 0.077 *  cos__q_RM_q2_joint__);
    (*this)(2,0) =  sin__q_RM_q2_joint__;
    (*this)(2,1) =  cos__q_RM_q2_joint__;
    (*this)(3,3) =  cos__q_RM_q2_joint__;
    (*this)(3,4) = - sin__q_RM_q2_joint__;
    (*this)(5,3) =  sin__q_RM_q2_joint__;
    (*this)(5,4) =  cos__q_RM_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RM_lowerleg_X_fr_RM_upperleg::Type_fr_RM_lowerleg_X_fr_RM_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.15;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_RM_lowerleg_X_fr_RM_upperleg& iit::mcorin::ForceTransforms::Type_fr_RM_lowerleg_X_fr_RM_upperleg::update(const state_t& q) {
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q3_joint__;
    
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RM_q3_joint__;
    (*this)(0,1) =  sin__q_RM_q3_joint__;
    (*this)(0,5) = ( 0.15 *  sin__q_RM_q3_joint__);
    (*this)(1,0) = - sin__q_RM_q3_joint__;
    (*this)(1,1) =  cos__q_RM_q3_joint__;
    (*this)(1,5) = ( 0.15 *  cos__q_RM_q3_joint__);
    (*this)(3,3) =  cos__q_RM_q3_joint__;
    (*this)(3,4) =  sin__q_RM_q3_joint__;
    (*this)(4,3) = - sin__q_RM_q3_joint__;
    (*this)(4,4) =  cos__q_RM_q3_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RM_upperleg_X_fr_RM_lowerleg::Type_fr_RM_upperleg_X_fr_RM_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.15;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::ForceTransforms::Type_fr_RM_upperleg_X_fr_RM_lowerleg& iit::mcorin::ForceTransforms::Type_fr_RM_upperleg_X_fr_RM_lowerleg::update(const state_t& q) {
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q3_joint__;
    
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RM_q3_joint__;
    (*this)(0,1) = - sin__q_RM_q3_joint__;
    (*this)(1,0) =  sin__q_RM_q3_joint__;
    (*this)(1,1) =  cos__q_RM_q3_joint__;
    (*this)(2,3) = ( 0.15 *  sin__q_RM_q3_joint__);
    (*this)(2,4) = ( 0.15 *  cos__q_RM_q3_joint__);
    (*this)(3,3) =  cos__q_RM_q3_joint__;
    (*this)(3,4) = - sin__q_RM_q3_joint__;
    (*this)(4,3) =  sin__q_RM_q3_joint__;
    (*this)(4,4) =  cos__q_RM_q3_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RR_hipassembly_X_fr_trunk::Type_fr_RR_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.075;
    (*this)(2,4) = 0.125;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_RR_hipassembly_X_fr_trunk& iit::mcorin::ForceTransforms::Type_fr_RR_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RR_q1_joint__;
    (*this)(0,1) = - cos__q_RR_q1_joint__;
    (*this)(0,5) = (( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__));
    (*this)(1,0) =  cos__q_RR_q1_joint__;
    (*this)(1,1) =  sin__q_RR_q1_joint__;
    (*this)(1,5) = (( 0.075 *  cos__q_RR_q1_joint__) - ( 0.125 *  sin__q_RR_q1_joint__));
    (*this)(3,3) =  sin__q_RR_q1_joint__;
    (*this)(3,4) = - cos__q_RR_q1_joint__;
    (*this)(4,3) =  cos__q_RR_q1_joint__;
    (*this)(4,4) =  sin__q_RR_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RR_hipassembly::Type_fr_trunk_X_fr_RR_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = - 0.075;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0.125;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RR_hipassembly& iit::mcorin::ForceTransforms::Type_fr_trunk_X_fr_RR_hipassembly::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RR_q1_joint__;
    (*this)(0,1) =  cos__q_RR_q1_joint__;
    (*this)(1,0) = - cos__q_RR_q1_joint__;
    (*this)(1,1) =  sin__q_RR_q1_joint__;
    (*this)(2,3) = (( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__));
    (*this)(2,4) = (( 0.075 *  cos__q_RR_q1_joint__) - ( 0.125 *  sin__q_RR_q1_joint__));
    (*this)(3,3) =  sin__q_RR_q1_joint__;
    (*this)(3,4) =  cos__q_RR_q1_joint__;
    (*this)(4,3) = - cos__q_RR_q1_joint__;
    (*this)(4,4) =  sin__q_RR_q1_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RR_upperleg_X_fr_RR_hipassembly::Type_fr_RR_upperleg_X_fr_RR_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.077;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1.0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_RR_upperleg_X_fr_RR_hipassembly& iit::mcorin::ForceTransforms::Type_fr_RR_upperleg_X_fr_RR_hipassembly::update(const state_t& q) {
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RR_q2_joint__;
    (*this)(0,2) =  sin__q_RR_q2_joint__;
    (*this)(0,4) = (- 0.077 *  sin__q_RR_q2_joint__);
    (*this)(1,0) = - sin__q_RR_q2_joint__;
    (*this)(1,2) =  cos__q_RR_q2_joint__;
    (*this)(1,4) = (- 0.077 *  cos__q_RR_q2_joint__);
    (*this)(3,3) =  cos__q_RR_q2_joint__;
    (*this)(3,5) =  sin__q_RR_q2_joint__;
    (*this)(4,3) = - sin__q_RR_q2_joint__;
    (*this)(4,5) =  cos__q_RR_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RR_hipassembly_X_fr_RR_upperleg::Type_fr_RR_hipassembly_X_fr_RR_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.077;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::mcorin::ForceTransforms::Type_fr_RR_hipassembly_X_fr_RR_upperleg& iit::mcorin::ForceTransforms::Type_fr_RR_hipassembly_X_fr_RR_upperleg::update(const state_t& q) {
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RR_q2_joint__;
    (*this)(0,1) = - sin__q_RR_q2_joint__;
    (*this)(1,3) = (- 0.077 *  sin__q_RR_q2_joint__);
    (*this)(1,4) = (- 0.077 *  cos__q_RR_q2_joint__);
    (*this)(2,0) =  sin__q_RR_q2_joint__;
    (*this)(2,1) =  cos__q_RR_q2_joint__;
    (*this)(3,3) =  cos__q_RR_q2_joint__;
    (*this)(3,4) = - sin__q_RR_q2_joint__;
    (*this)(5,3) =  sin__q_RR_q2_joint__;
    (*this)(5,4) =  cos__q_RR_q2_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RR_lowerleg_X_fr_RR_upperleg::Type_fr_RR_lowerleg_X_fr_RR_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.15;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::mcorin::ForceTransforms::Type_fr_RR_lowerleg_X_fr_RR_upperleg& iit::mcorin::ForceTransforms::Type_fr_RR_lowerleg_X_fr_RR_upperleg::update(const state_t& q) {
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q3_joint__;
    
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RR_q3_joint__;
    (*this)(0,1) =  sin__q_RR_q3_joint__;
    (*this)(0,5) = ( 0.15 *  sin__q_RR_q3_joint__);
    (*this)(1,0) = - sin__q_RR_q3_joint__;
    (*this)(1,1) =  cos__q_RR_q3_joint__;
    (*this)(1,5) = ( 0.15 *  cos__q_RR_q3_joint__);
    (*this)(3,3) =  cos__q_RR_q3_joint__;
    (*this)(3,4) =  sin__q_RR_q3_joint__;
    (*this)(4,3) = - sin__q_RR_q3_joint__;
    (*this)(4,4) =  cos__q_RR_q3_joint__;
    return *this;
}
iit::mcorin::ForceTransforms::Type_fr_RR_upperleg_X_fr_RR_lowerleg::Type_fr_RR_upperleg_X_fr_RR_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.15;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::mcorin::ForceTransforms::Type_fr_RR_upperleg_X_fr_RR_lowerleg& iit::mcorin::ForceTransforms::Type_fr_RR_upperleg_X_fr_RR_lowerleg::update(const state_t& q) {
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q3_joint__;
    
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RR_q3_joint__;
    (*this)(0,1) = - sin__q_RR_q3_joint__;
    (*this)(1,0) =  sin__q_RR_q3_joint__;
    (*this)(1,1) =  cos__q_RR_q3_joint__;
    (*this)(2,3) = ( 0.15 *  sin__q_RR_q3_joint__);
    (*this)(2,4) = ( 0.15 *  cos__q_RR_q3_joint__);
    (*this)(3,3) =  cos__q_RR_q3_joint__;
    (*this)(3,4) = - sin__q_RR_q3_joint__;
    (*this)(4,3) =  sin__q_RR_q3_joint__;
    (*this)(4,4) =  cos__q_RR_q3_joint__;
    return *this;
}

iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LF_hipassemblyCOM::Type_fr_trunk_X_LF_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LF_hipassemblyCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LF_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LF_q1_joint__;
    (*this)(0,1) = - cos__q_LF_q1_joint__;
    (*this)(0,3) = ( 0.125 - ( 0.05821 *  sin__q_LF_q1_joint__));
    (*this)(1,0) =  cos__q_LF_q1_joint__;
    (*this)(1,1) = - sin__q_LF_q1_joint__;
    (*this)(1,3) = (( 0.05821 *  cos__q_LF_q1_joint__) +  0.075);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LF_upperlegCOM::Type_fr_trunk_X_LF_upperlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LF_upperlegCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LF_upperlegCOM::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(0,1) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(0,3) = ((((- 0.1213 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) - ( 0.077 *  sin__q_LF_q1_joint__)) +  0.125);
    (*this)(1,0) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(1,1) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(1,3) = (((( 0.1213 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) + ( 0.077 *  cos__q_LF_q1_joint__)) +  0.075);
    (*this)(2,0) =  sin__q_LF_q2_joint__;
    (*this)(2,1) =  cos__q_LF_q2_joint__;
    (*this)(2,3) = ( 0.1213 *  sin__q_LF_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LF_lowerlegCOM::Type_fr_trunk_X_LF_lowerlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LF_lowerlegCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LF_lowerlegCOM::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q3_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,1) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(0,3) = ((((((( 0.08853 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - ((( 0.08853 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) - (( 0.15 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__)) - ( 0.077 *  sin__q_LF_q1_joint__)) +  0.125);
    (*this)(1,0) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(1,1) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(1,3) = (((((((- 0.08853 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.08853 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) + (( 0.15 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__)) + ( 0.077 *  cos__q_LF_q1_joint__)) +  0.075);
    (*this)(2,0) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(2,1) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    (*this)(2,3) = (((( 0.08853 *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( 0.08853 *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) + ( 0.15 *  sin__q_LF_q2_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LM_hipassemblyCOM::Type_fr_trunk_X_LM_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LM_hipassemblyCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LM_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LM_q1_joint__;
    (*this)(0,1) = - cos__q_LM_q1_joint__;
    (*this)(0,3) = (- 0.05821 *  sin__q_LM_q1_joint__);
    (*this)(1,0) =  cos__q_LM_q1_joint__;
    (*this)(1,1) = - sin__q_LM_q1_joint__;
    (*this)(1,3) = (( 0.05821 *  cos__q_LM_q1_joint__) +  0.075);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LM_upperlegCOM::Type_fr_trunk_X_LM_upperlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LM_upperlegCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LM_upperlegCOM::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(0,1) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(0,3) = (((- 0.1213 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) - ( 0.077 *  sin__q_LM_q1_joint__));
    (*this)(1,0) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(1,1) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(1,3) = (((( 0.1213 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) + ( 0.077 *  cos__q_LM_q1_joint__)) +  0.075);
    (*this)(2,0) =  sin__q_LM_q2_joint__;
    (*this)(2,1) =  cos__q_LM_q2_joint__;
    (*this)(2,3) = ( 0.1213 *  sin__q_LM_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LM_lowerlegCOM::Type_fr_trunk_X_LM_lowerlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LM_lowerlegCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LM_lowerlegCOM::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q3_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,1) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(0,3) = (((((( 0.08853 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.08853 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) - (( 0.15 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) - ( 0.077 *  sin__q_LM_q1_joint__));
    (*this)(1,0) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(1,1) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(1,3) = (((((((- 0.08853 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.08853 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) + (( 0.15 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) + ( 0.077 *  cos__q_LM_q1_joint__)) +  0.075);
    (*this)(2,0) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(2,1) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    (*this)(2,3) = (((( 0.08853 *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( 0.08853 *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) + ( 0.15 *  sin__q_LM_q2_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LR_hipassemblyCOM::Type_fr_trunk_X_LR_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LR_hipassemblyCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LR_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LR_q1_joint__;
    (*this)(0,1) = - cos__q_LR_q1_joint__;
    (*this)(0,3) = ((- 0.05821 *  sin__q_LR_q1_joint__) -  0.125);
    (*this)(1,0) =  cos__q_LR_q1_joint__;
    (*this)(1,1) = - sin__q_LR_q1_joint__;
    (*this)(1,3) = (( 0.05821 *  cos__q_LR_q1_joint__) +  0.075);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LR_upperlegCOM::Type_fr_trunk_X_LR_upperlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LR_upperlegCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LR_upperlegCOM::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(0,1) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(0,3) = ((((- 0.1213 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) - ( 0.077 *  sin__q_LR_q1_joint__)) -  0.125);
    (*this)(1,0) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(1,1) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(1,3) = (((( 0.1213 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) + ( 0.077 *  cos__q_LR_q1_joint__)) +  0.075);
    (*this)(2,0) =  sin__q_LR_q2_joint__;
    (*this)(2,1) =  cos__q_LR_q2_joint__;
    (*this)(2,3) = ( 0.1213 *  sin__q_LR_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LR_lowerlegCOM::Type_fr_trunk_X_LR_lowerlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LR_lowerlegCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LR_lowerlegCOM::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q3_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,1) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(0,3) = ((((((( 0.08853 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - ((( 0.08853 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) - (( 0.15 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__)) - ( 0.077 *  sin__q_LR_q1_joint__)) -  0.125);
    (*this)(1,0) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(1,1) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(1,3) = (((((((- 0.08853 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.08853 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) + (( 0.15 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__)) + ( 0.077 *  cos__q_LR_q1_joint__)) +  0.075);
    (*this)(2,0) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(2,1) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    (*this)(2,3) = (((( 0.08853 *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( 0.08853 *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) + ( 0.15 *  sin__q_LR_q2_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RF_hipassemblyCOM::Type_fr_trunk_X_RF_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RF_hipassemblyCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RF_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RF_q1_joint__;
    (*this)(0,1) =  cos__q_RF_q1_joint__;
    (*this)(0,3) = (( 0.05821 *  sin__q_RF_q1_joint__) +  0.125);
    (*this)(1,0) = - cos__q_RF_q1_joint__;
    (*this)(1,1) =  sin__q_RF_q1_joint__;
    (*this)(1,3) = ((- 0.05821 *  cos__q_RF_q1_joint__) -  0.075);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RF_upperlegCOM::Type_fr_trunk_X_RF_upperlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RF_upperlegCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RF_upperlegCOM::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(0,1) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(0,3) = (((( 0.1213 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) + ( 0.077 *  sin__q_RF_q1_joint__)) +  0.125);
    (*this)(1,0) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(1,1) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(1,3) = ((((- 0.1213 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) - ( 0.077 *  cos__q_RF_q1_joint__)) -  0.075);
    (*this)(2,0) =  sin__q_RF_q2_joint__;
    (*this)(2,1) =  cos__q_RF_q2_joint__;
    (*this)(2,3) = ( 0.1213 *  sin__q_RF_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RF_lowerlegCOM::Type_fr_trunk_X_RF_lowerlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RF_lowerlegCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RF_lowerlegCOM::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q3_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(0,1) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(0,3) = (((((((- 0.08853 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + ((( 0.08853 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) + (( 0.15 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__)) + ( 0.077 *  sin__q_RF_q1_joint__)) +  0.125);
    (*this)(1,0) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,1) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(1,3) = ((((((( 0.08853 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - ((( 0.08853 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) - (( 0.15 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__)) - ( 0.077 *  cos__q_RF_q1_joint__)) -  0.075);
    (*this)(2,0) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(2,1) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    (*this)(2,3) = (((( 0.08853 *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( 0.08853 *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) + ( 0.15 *  sin__q_RF_q2_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RM_hipassemblyCOM::Type_fr_trunk_X_RM_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RM_hipassemblyCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RM_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RM_q1_joint__;
    (*this)(0,1) =  cos__q_RM_q1_joint__;
    (*this)(0,3) = ( 0.05821 *  sin__q_RM_q1_joint__);
    (*this)(1,0) = - cos__q_RM_q1_joint__;
    (*this)(1,1) =  sin__q_RM_q1_joint__;
    (*this)(1,3) = ((- 0.05821 *  cos__q_RM_q1_joint__) -  0.075);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RM_upperlegCOM::Type_fr_trunk_X_RM_upperlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RM_upperlegCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RM_upperlegCOM::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(0,1) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(0,3) = ((( 0.1213 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) + ( 0.077 *  sin__q_RM_q1_joint__));
    (*this)(1,0) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(1,1) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(1,3) = ((((- 0.1213 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) - ( 0.077 *  cos__q_RM_q1_joint__)) -  0.075);
    (*this)(2,0) =  sin__q_RM_q2_joint__;
    (*this)(2,1) =  cos__q_RM_q2_joint__;
    (*this)(2,3) = ( 0.1213 *  sin__q_RM_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RM_lowerlegCOM::Type_fr_trunk_X_RM_lowerlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RM_lowerlegCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RM_lowerlegCOM::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q3_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(0,1) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(0,3) = ((((((- 0.08853 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + ((( 0.08853 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) + (( 0.15 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) + ( 0.077 *  sin__q_RM_q1_joint__));
    (*this)(1,0) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,1) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(1,3) = ((((((( 0.08853 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.08853 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) - (( 0.15 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) - ( 0.077 *  cos__q_RM_q1_joint__)) -  0.075);
    (*this)(2,0) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(2,1) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    (*this)(2,3) = (((( 0.08853 *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( 0.08853 *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) + ( 0.15 *  sin__q_RM_q2_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RR_hipassemblyCOM::Type_fr_trunk_X_RR_hipassemblyCOM()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RR_hipassemblyCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RR_hipassemblyCOM::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RR_q1_joint__;
    (*this)(0,1) =  cos__q_RR_q1_joint__;
    (*this)(0,3) = (( 0.05821 *  sin__q_RR_q1_joint__) -  0.125);
    (*this)(1,0) = - cos__q_RR_q1_joint__;
    (*this)(1,1) =  sin__q_RR_q1_joint__;
    (*this)(1,3) = ((- 0.05821 *  cos__q_RR_q1_joint__) -  0.075);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RR_upperlegCOM::Type_fr_trunk_X_RR_upperlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RR_upperlegCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RR_upperlegCOM::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(0,1) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(0,3) = (((( 0.1213 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) + ( 0.077 *  sin__q_RR_q1_joint__)) -  0.125);
    (*this)(1,0) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(1,1) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(1,3) = ((((- 0.1213 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) - ( 0.077 *  cos__q_RR_q1_joint__)) -  0.075);
    (*this)(2,0) =  sin__q_RR_q2_joint__;
    (*this)(2,1) =  cos__q_RR_q2_joint__;
    (*this)(2,3) = ( 0.1213 *  sin__q_RR_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RR_lowerlegCOM::Type_fr_trunk_X_RR_lowerlegCOM()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RR_lowerlegCOM& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RR_lowerlegCOM::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q3_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(0,1) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(0,3) = (((((((- 0.08853 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((( 0.08853 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) + (( 0.15 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__)) + ( 0.077 *  sin__q_RR_q1_joint__)) -  0.125);
    (*this)(1,0) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,1) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(1,3) = ((((((( 0.08853 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - ((( 0.08853 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) - (( 0.15 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__)) - ( 0.077 *  cos__q_RR_q1_joint__)) -  0.075);
    (*this)(2,0) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(2,1) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    (*this)(2,3) = (((( 0.08853 *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( 0.08853 *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) + ( 0.15 *  sin__q_RR_q2_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LF_upperleg_X_fr_trunk::Type_fr_LF_upperleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LF_upperleg_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_LF_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(0,1) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(0,2) =  sin__q_LF_q2_joint__;
    (*this)(0,3) = (((( 0.125 *  sin__q_LF_q1_joint__) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077) *  cos__q_LF_q2_joint__);
    (*this)(1,0) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(1,1) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(1,2) =  cos__q_LF_q2_joint__;
    (*this)(1,3) = ((((- 0.125 *  sin__q_LF_q1_joint__) + ( 0.075 *  cos__q_LF_q1_joint__)) +  0.077) *  sin__q_LF_q2_joint__);
    (*this)(2,0) =  cos__q_LF_q1_joint__;
    (*this)(2,1) =  sin__q_LF_q1_joint__;
    (*this)(2,3) = ((- 0.075 *  sin__q_LF_q1_joint__) - ( 0.125 *  cos__q_LF_q1_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LF_lowerleg_X_fr_trunk::Type_fr_LF_lowerleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LF_lowerleg_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_LF_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q3_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,1) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(0,2) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(0,3) = ((((( 0.077 + ( 0.075 *  cos__q_LF_q1_joint__)) - ( 0.125 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (((((( 0.125 *  sin__q_LF_q1_joint__) - ( 0.075 *  cos__q_LF_q1_joint__)) -  0.077) *  cos__q_LF_q2_joint__) -  0.15) *  cos__q_LF_q3_joint__));
    (*this)(1,0) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,1) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,2) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    (*this)(1,3) = ((( 0.15 + ((((- 0.125 *  sin__q_LF_q1_joint__) + ( 0.075 *  cos__q_LF_q1_joint__)) +  0.077) *  cos__q_LF_q2_joint__)) *  sin__q_LF_q3_joint__) + (((( 0.077 + ( 0.075 *  cos__q_LF_q1_joint__)) - ( 0.125 *  sin__q_LF_q1_joint__)) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(2,0) =  cos__q_LF_q1_joint__;
    (*this)(2,1) =  sin__q_LF_q1_joint__;
    (*this)(2,3) = ((- 0.075 *  sin__q_LF_q1_joint__) - ( 0.125 *  cos__q_LF_q1_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LM_upperleg_X_fr_trunk::Type_fr_LM_upperleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LM_upperleg_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_LM_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(0,1) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(0,2) =  sin__q_LM_q2_joint__;
    (*this)(0,3) = (((- 0.075 *  cos__q_LM_q1_joint__) -  0.077) *  cos__q_LM_q2_joint__);
    (*this)(1,0) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(1,1) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(1,2) =  cos__q_LM_q2_joint__;
    (*this)(1,3) = ((( 0.075 *  cos__q_LM_q1_joint__) +  0.077) *  sin__q_LM_q2_joint__);
    (*this)(2,0) =  cos__q_LM_q1_joint__;
    (*this)(2,1) =  sin__q_LM_q1_joint__;
    (*this)(2,3) = (- 0.075 *  sin__q_LM_q1_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LM_lowerleg_X_fr_trunk::Type_fr_LM_lowerleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LM_lowerleg_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_LM_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q3_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,1) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(0,2) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(0,3) = (((( 0.077 + ( 0.075 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (((((- 0.075 *  cos__q_LM_q1_joint__) -  0.077) *  cos__q_LM_q2_joint__) -  0.15) *  cos__q_LM_q3_joint__));
    (*this)(1,0) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,1) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,2) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    (*this)(1,3) = ((( 0.15 + ((( 0.075 *  cos__q_LM_q1_joint__) +  0.077) *  cos__q_LM_q2_joint__)) *  sin__q_LM_q3_joint__) + ((( 0.077 + ( 0.075 *  cos__q_LM_q1_joint__)) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(2,0) =  cos__q_LM_q1_joint__;
    (*this)(2,1) =  sin__q_LM_q1_joint__;
    (*this)(2,3) = (- 0.075 *  sin__q_LM_q1_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LR_upperleg_X_fr_trunk::Type_fr_LR_upperleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LR_upperleg_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_LR_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(0,1) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(0,2) =  sin__q_LR_q2_joint__;
    (*this)(0,3) = ((((- 0.125 *  sin__q_LR_q1_joint__) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077) *  cos__q_LR_q2_joint__);
    (*this)(1,0) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(1,1) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(1,2) =  cos__q_LR_q2_joint__;
    (*this)(1,3) = (((( 0.125 *  sin__q_LR_q1_joint__) + ( 0.075 *  cos__q_LR_q1_joint__)) +  0.077) *  sin__q_LR_q2_joint__);
    (*this)(2,0) =  cos__q_LR_q1_joint__;
    (*this)(2,1) =  sin__q_LR_q1_joint__;
    (*this)(2,3) = (( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LR_lowerleg_X_fr_trunk::Type_fr_LR_lowerleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LR_lowerleg_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_LR_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q3_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,1) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(0,2) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(0,3) = ((((( 0.077 + ( 0.075 *  cos__q_LR_q1_joint__)) + ( 0.125 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((((((- 0.125 *  sin__q_LR_q1_joint__) - ( 0.075 *  cos__q_LR_q1_joint__)) -  0.077) *  cos__q_LR_q2_joint__) -  0.15) *  cos__q_LR_q3_joint__));
    (*this)(1,0) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,1) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,2) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    (*this)(1,3) = ((( 0.15 + (((( 0.125 *  sin__q_LR_q1_joint__) + ( 0.075 *  cos__q_LR_q1_joint__)) +  0.077) *  cos__q_LR_q2_joint__)) *  sin__q_LR_q3_joint__) + (((( 0.077 + ( 0.075 *  cos__q_LR_q1_joint__)) + ( 0.125 *  sin__q_LR_q1_joint__)) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(2,0) =  cos__q_LR_q1_joint__;
    (*this)(2,1) =  sin__q_LR_q1_joint__;
    (*this)(2,3) = (( 0.125 *  cos__q_LR_q1_joint__) - ( 0.075 *  sin__q_LR_q1_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RF_upperleg_X_fr_trunk::Type_fr_RF_upperleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RF_upperleg_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_RF_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(0,1) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(0,2) =  sin__q_RF_q2_joint__;
    (*this)(0,3) = ((((- 0.125 *  sin__q_RF_q1_joint__) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077) *  cos__q_RF_q2_joint__);
    (*this)(1,0) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(1,1) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(1,2) =  cos__q_RF_q2_joint__;
    (*this)(1,3) = (((( 0.125 *  sin__q_RF_q1_joint__) + ( 0.075 *  cos__q_RF_q1_joint__)) +  0.077) *  sin__q_RF_q2_joint__);
    (*this)(2,0) = - cos__q_RF_q1_joint__;
    (*this)(2,1) = - sin__q_RF_q1_joint__;
    (*this)(2,3) = (( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RF_lowerleg_X_fr_trunk::Type_fr_RF_lowerleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RF_lowerleg_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_RF_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q3_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(0,1) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(0,2) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(0,3) = ((((( 0.077 + ( 0.075 *  cos__q_RF_q1_joint__)) + ( 0.125 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + ((((((- 0.125 *  sin__q_RF_q1_joint__) - ( 0.075 *  cos__q_RF_q1_joint__)) -  0.077) *  cos__q_RF_q2_joint__) -  0.15) *  cos__q_RF_q3_joint__));
    (*this)(1,0) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,1) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,2) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    (*this)(1,3) = ((( 0.15 + (((( 0.125 *  sin__q_RF_q1_joint__) + ( 0.075 *  cos__q_RF_q1_joint__)) +  0.077) *  cos__q_RF_q2_joint__)) *  sin__q_RF_q3_joint__) + (((( 0.077 + ( 0.075 *  cos__q_RF_q1_joint__)) + ( 0.125 *  sin__q_RF_q1_joint__)) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(2,0) = - cos__q_RF_q1_joint__;
    (*this)(2,1) = - sin__q_RF_q1_joint__;
    (*this)(2,3) = (( 0.125 *  cos__q_RF_q1_joint__) - ( 0.075 *  sin__q_RF_q1_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RM_upperleg_X_fr_trunk::Type_fr_RM_upperleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RM_upperleg_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_RM_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(0,1) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(0,2) =  sin__q_RM_q2_joint__;
    (*this)(0,3) = (((- 0.075 *  cos__q_RM_q1_joint__) -  0.077) *  cos__q_RM_q2_joint__);
    (*this)(1,0) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(1,1) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(1,2) =  cos__q_RM_q2_joint__;
    (*this)(1,3) = ((( 0.075 *  cos__q_RM_q1_joint__) +  0.077) *  sin__q_RM_q2_joint__);
    (*this)(2,0) = - cos__q_RM_q1_joint__;
    (*this)(2,1) = - sin__q_RM_q1_joint__;
    (*this)(2,3) = (- 0.075 *  sin__q_RM_q1_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RM_lowerleg_X_fr_trunk::Type_fr_RM_lowerleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RM_lowerleg_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_RM_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q3_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(0,1) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(0,2) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(0,3) = (((( 0.077 + ( 0.075 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (((((- 0.075 *  cos__q_RM_q1_joint__) -  0.077) *  cos__q_RM_q2_joint__) -  0.15) *  cos__q_RM_q3_joint__));
    (*this)(1,0) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,1) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,2) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    (*this)(1,3) = ((( 0.15 + ((( 0.075 *  cos__q_RM_q1_joint__) +  0.077) *  cos__q_RM_q2_joint__)) *  sin__q_RM_q3_joint__) + ((( 0.077 + ( 0.075 *  cos__q_RM_q1_joint__)) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(2,0) = - cos__q_RM_q1_joint__;
    (*this)(2,1) = - sin__q_RM_q1_joint__;
    (*this)(2,3) = (- 0.075 *  sin__q_RM_q1_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RR_upperleg_X_fr_trunk::Type_fr_RR_upperleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RR_upperleg_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_RR_upperleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(0,1) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(0,2) =  sin__q_RR_q2_joint__;
    (*this)(0,3) = (((( 0.125 *  sin__q_RR_q1_joint__) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077) *  cos__q_RR_q2_joint__);
    (*this)(1,0) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(1,1) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(1,2) =  cos__q_RR_q2_joint__;
    (*this)(1,3) = ((((- 0.125 *  sin__q_RR_q1_joint__) + ( 0.075 *  cos__q_RR_q1_joint__)) +  0.077) *  sin__q_RR_q2_joint__);
    (*this)(2,0) = - cos__q_RR_q1_joint__;
    (*this)(2,1) = - sin__q_RR_q1_joint__;
    (*this)(2,3) = ((- 0.075 *  sin__q_RR_q1_joint__) - ( 0.125 *  cos__q_RR_q1_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RR_lowerleg_X_fr_trunk::Type_fr_RR_lowerleg_X_fr_trunk()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RR_lowerleg_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_RR_lowerleg_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q3_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(0,1) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(0,2) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(0,3) = ((((( 0.077 + ( 0.075 *  cos__q_RR_q1_joint__)) - ( 0.125 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (((((( 0.125 *  sin__q_RR_q1_joint__) - ( 0.075 *  cos__q_RR_q1_joint__)) -  0.077) *  cos__q_RR_q2_joint__) -  0.15) *  cos__q_RR_q3_joint__));
    (*this)(1,0) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,1) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,2) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    (*this)(1,3) = ((( 0.15 + ((((- 0.125 *  sin__q_RR_q1_joint__) + ( 0.075 *  cos__q_RR_q1_joint__)) +  0.077) *  cos__q_RR_q2_joint__)) *  sin__q_RR_q3_joint__) + (((( 0.077 + ( 0.075 *  cos__q_RR_q1_joint__)) - ( 0.125 *  sin__q_RR_q1_joint__)) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(2,0) = - cos__q_RR_q1_joint__;
    (*this)(2,1) = - sin__q_RR_q1_joint__;
    (*this)(2,3) = ((- 0.075 *  sin__q_RR_q1_joint__) - ( 0.125 *  cos__q_RR_q1_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LF_foot::Type_fr_trunk_X_LF_foot()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LF_foot& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LF_foot::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q3_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,1) = ((( sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(0,3) = ((((((( 0.17 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - ((( 0.17 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) - (( 0.15 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__)) - ( 0.077 *  sin__q_LF_q1_joint__)) +  0.125);
    (*this)(1,0) = ((( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    (*this)(1,1) = (((- cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - (( cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(1,3) = (((((((- 0.17 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.17 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) + (( 0.15 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__)) + ( 0.077 *  cos__q_LF_q1_joint__)) +  0.075);
    (*this)(2,0) = (( cos__q_LF_q2_joint__ *  sin__q_LF_q3_joint__) + ( sin__q_LF_q2_joint__ *  cos__q_LF_q3_joint__));
    (*this)(2,1) = (( cos__q_LF_q2_joint__ *  cos__q_LF_q3_joint__) - ( sin__q_LF_q2_joint__ *  sin__q_LF_q3_joint__));
    (*this)(2,3) = (((( 0.17 *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( 0.17 *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) + ( 0.15 *  sin__q_LF_q2_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LF_q1_joint::Type_fr_trunk_X_fr_LF_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.125;
    (*this)(1,0) = 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.075;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LF_q1_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LF_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LF_q2_joint::Type_fr_trunk_X_fr_LF_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LF_q2_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LF_q2_joint::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LF_q1_joint__;
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(0,3) = ( 0.125 - ( 0.077 *  sin__q_LF_q1_joint__));
    (*this)(1,0) =  cos__q_LF_q1_joint__;
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(1,3) = (( 0.077 *  cos__q_LF_q1_joint__) +  0.075);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LF_q3_joint::Type_fr_trunk_X_fr_LF_q3_joint()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LF_q3_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LF_q3_joint::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(0,1) = ( sin__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(0,3) = ((((- 0.15 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) - ( 0.077 *  sin__q_LF_q1_joint__)) +  0.125);
    (*this)(1,0) = ( cos__q_LF_q1_joint__ *  cos__q_LF_q2_joint__);
    (*this)(1,1) = (- cos__q_LF_q1_joint__ *  sin__q_LF_q2_joint__);
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(1,3) = (((( 0.15 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) + ( 0.077 *  cos__q_LF_q1_joint__)) +  0.075);
    (*this)(2,0) =  sin__q_LF_q2_joint__;
    (*this)(2,1) =  cos__q_LF_q2_joint__;
    (*this)(2,3) = ( 0.15 *  sin__q_LF_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LM_foot::Type_fr_trunk_X_LM_foot()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LM_foot& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LM_foot::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q3_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,1) = ((( sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(0,3) = (((((( 0.17 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.17 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) - (( 0.15 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) - ( 0.077 *  sin__q_LM_q1_joint__));
    (*this)(1,0) = ((( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    (*this)(1,1) = (((- cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - (( cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(1,3) = (((((((- 0.17 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.17 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) + (( 0.15 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) + ( 0.077 *  cos__q_LM_q1_joint__)) +  0.075);
    (*this)(2,0) = (( cos__q_LM_q2_joint__ *  sin__q_LM_q3_joint__) + ( sin__q_LM_q2_joint__ *  cos__q_LM_q3_joint__));
    (*this)(2,1) = (( cos__q_LM_q2_joint__ *  cos__q_LM_q3_joint__) - ( sin__q_LM_q2_joint__ *  sin__q_LM_q3_joint__));
    (*this)(2,3) = (((( 0.17 *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( 0.17 *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) + ( 0.15 *  sin__q_LM_q2_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LM_q1_joint::Type_fr_trunk_X_fr_LM_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.075;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LM_q1_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LM_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LM_q2_joint::Type_fr_trunk_X_fr_LM_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LM_q2_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LM_q2_joint::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LM_q1_joint__;
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(0,3) = (- 0.077 *  sin__q_LM_q1_joint__);
    (*this)(1,0) =  cos__q_LM_q1_joint__;
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(1,3) = (( 0.077 *  cos__q_LM_q1_joint__) +  0.075);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LM_q3_joint::Type_fr_trunk_X_fr_LM_q3_joint()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LM_q3_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LM_q3_joint::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(0,1) = ( sin__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(0,3) = (((- 0.15 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) - ( 0.077 *  sin__q_LM_q1_joint__));
    (*this)(1,0) = ( cos__q_LM_q1_joint__ *  cos__q_LM_q2_joint__);
    (*this)(1,1) = (- cos__q_LM_q1_joint__ *  sin__q_LM_q2_joint__);
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(1,3) = (((( 0.15 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) + ( 0.077 *  cos__q_LM_q1_joint__)) +  0.075);
    (*this)(2,0) =  sin__q_LM_q2_joint__;
    (*this)(2,1) =  cos__q_LM_q2_joint__;
    (*this)(2,3) = ( 0.15 *  sin__q_LM_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LR_foot::Type_fr_trunk_X_LR_foot()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LR_foot& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_LR_foot::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q3_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,1) = ((( sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(0,3) = ((((((( 0.17 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - ((( 0.17 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) - (( 0.15 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__)) - ( 0.077 *  sin__q_LR_q1_joint__)) -  0.125);
    (*this)(1,0) = ((( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    (*this)(1,1) = (((- cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - (( cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(1,3) = (((((((- 0.17 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.17 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) + (( 0.15 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__)) + ( 0.077 *  cos__q_LR_q1_joint__)) +  0.075);
    (*this)(2,0) = (( cos__q_LR_q2_joint__ *  sin__q_LR_q3_joint__) + ( sin__q_LR_q2_joint__ *  cos__q_LR_q3_joint__));
    (*this)(2,1) = (( cos__q_LR_q2_joint__ *  cos__q_LR_q3_joint__) - ( sin__q_LR_q2_joint__ *  sin__q_LR_q3_joint__));
    (*this)(2,3) = (((( 0.17 *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( 0.17 *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) + ( 0.15 *  sin__q_LR_q2_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LR_q1_joint::Type_fr_trunk_X_fr_LR_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = - 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.125;
    (*this)(1,0) = 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.075;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LR_q1_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LR_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LR_q2_joint::Type_fr_trunk_X_fr_LR_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LR_q2_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LR_q2_joint::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LR_q1_joint__;
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(0,3) = ((- 0.077 *  sin__q_LR_q1_joint__) -  0.125);
    (*this)(1,0) =  cos__q_LR_q1_joint__;
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(1,3) = (( 0.077 *  cos__q_LR_q1_joint__) +  0.075);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LR_q3_joint::Type_fr_trunk_X_fr_LR_q3_joint()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LR_q3_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LR_q3_joint::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = (- sin__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(0,1) = ( sin__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(0,3) = ((((- 0.15 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) - ( 0.077 *  sin__q_LR_q1_joint__)) -  0.125);
    (*this)(1,0) = ( cos__q_LR_q1_joint__ *  cos__q_LR_q2_joint__);
    (*this)(1,1) = (- cos__q_LR_q1_joint__ *  sin__q_LR_q2_joint__);
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(1,3) = (((( 0.15 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) + ( 0.077 *  cos__q_LR_q1_joint__)) +  0.075);
    (*this)(2,0) =  sin__q_LR_q2_joint__;
    (*this)(2,1) =  cos__q_LR_q2_joint__;
    (*this)(2,3) = ( 0.15 *  sin__q_LR_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RF_foot::Type_fr_trunk_X_RF_foot()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RF_foot& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RF_foot::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q3_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    (*this)(0,1) = (((- sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(0,3) = (((((((- 0.17 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + ((( 0.17 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) + (( 0.15 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__)) + ( 0.077 *  sin__q_RF_q1_joint__)) +  0.125);
    (*this)(1,0) = ((( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - (( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,1) = ((( cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(1,3) = ((((((( 0.17 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - ((( 0.17 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) - (( 0.15 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__)) - ( 0.077 *  cos__q_RF_q1_joint__)) -  0.075);
    (*this)(2,0) = (( cos__q_RF_q2_joint__ *  sin__q_RF_q3_joint__) + ( sin__q_RF_q2_joint__ *  cos__q_RF_q3_joint__));
    (*this)(2,1) = (( cos__q_RF_q2_joint__ *  cos__q_RF_q3_joint__) - ( sin__q_RF_q2_joint__ *  sin__q_RF_q3_joint__));
    (*this)(2,3) = (((( 0.17 *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( 0.17 *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) + ( 0.15 *  sin__q_RF_q2_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RF_q1_joint::Type_fr_trunk_X_fr_RF_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.125;
    (*this)(1,0) = - 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.075;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RF_q1_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RF_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RF_q2_joint::Type_fr_trunk_X_fr_RF_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RF_q2_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RF_q2_joint::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RF_q1_joint__;
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(0,3) = (( 0.077 *  sin__q_RF_q1_joint__) +  0.125);
    (*this)(1,0) = - cos__q_RF_q1_joint__;
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(1,3) = ((- 0.077 *  cos__q_RF_q1_joint__) -  0.075);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RF_q3_joint::Type_fr_trunk_X_fr_RF_q3_joint()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RF_q3_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RF_q3_joint::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(0,1) = (- sin__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(0,3) = (((( 0.15 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) + ( 0.077 *  sin__q_RF_q1_joint__)) +  0.125);
    (*this)(1,0) = (- cos__q_RF_q1_joint__ *  cos__q_RF_q2_joint__);
    (*this)(1,1) = ( cos__q_RF_q1_joint__ *  sin__q_RF_q2_joint__);
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(1,3) = ((((- 0.15 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) - ( 0.077 *  cos__q_RF_q1_joint__)) -  0.075);
    (*this)(2,0) =  sin__q_RF_q2_joint__;
    (*this)(2,1) =  cos__q_RF_q2_joint__;
    (*this)(2,3) = ( 0.15 *  sin__q_RF_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RM_foot::Type_fr_trunk_X_RM_foot()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RM_foot& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RM_foot::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q3_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    (*this)(0,1) = (((- sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(0,3) = ((((((- 0.17 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + ((( 0.17 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) + (( 0.15 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) + ( 0.077 *  sin__q_RM_q1_joint__));
    (*this)(1,0) = ((( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - (( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,1) = ((( cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(1,3) = ((((((( 0.17 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.17 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) - (( 0.15 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) - ( 0.077 *  cos__q_RM_q1_joint__)) -  0.075);
    (*this)(2,0) = (( cos__q_RM_q2_joint__ *  sin__q_RM_q3_joint__) + ( sin__q_RM_q2_joint__ *  cos__q_RM_q3_joint__));
    (*this)(2,1) = (( cos__q_RM_q2_joint__ *  cos__q_RM_q3_joint__) - ( sin__q_RM_q2_joint__ *  sin__q_RM_q3_joint__));
    (*this)(2,3) = (((( 0.17 *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( 0.17 *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) + ( 0.15 *  sin__q_RM_q2_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RM_q1_joint::Type_fr_trunk_X_fr_RM_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 1;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = - 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.075;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RM_q1_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RM_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RM_q2_joint::Type_fr_trunk_X_fr_RM_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RM_q2_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RM_q2_joint::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RM_q1_joint__;
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(0,3) = ( 0.077 *  sin__q_RM_q1_joint__);
    (*this)(1,0) = - cos__q_RM_q1_joint__;
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(1,3) = ((- 0.077 *  cos__q_RM_q1_joint__) -  0.075);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RM_q3_joint::Type_fr_trunk_X_fr_RM_q3_joint()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RM_q3_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RM_q3_joint::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(0,1) = (- sin__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(0,3) = ((( 0.15 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) + ( 0.077 *  sin__q_RM_q1_joint__));
    (*this)(1,0) = (- cos__q_RM_q1_joint__ *  cos__q_RM_q2_joint__);
    (*this)(1,1) = ( cos__q_RM_q1_joint__ *  sin__q_RM_q2_joint__);
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(1,3) = ((((- 0.15 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) - ( 0.077 *  cos__q_RM_q1_joint__)) -  0.075);
    (*this)(2,0) =  sin__q_RM_q2_joint__;
    (*this)(2,1) =  cos__q_RM_q2_joint__;
    (*this)(2,3) = ( 0.15 *  sin__q_RM_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RR_foot::Type_fr_trunk_X_RR_foot()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RR_foot& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_RR_foot::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q3_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ((( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    (*this)(0,1) = (((- sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(0,3) = (((((((- 0.17 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((( 0.17 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) + (( 0.15 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__)) + ( 0.077 *  sin__q_RR_q1_joint__)) -  0.125);
    (*this)(1,0) = ((( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - (( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,1) = ((( cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(1,3) = ((((((( 0.17 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - ((( 0.17 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) - (( 0.15 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__)) - ( 0.077 *  cos__q_RR_q1_joint__)) -  0.075);
    (*this)(2,0) = (( cos__q_RR_q2_joint__ *  sin__q_RR_q3_joint__) + ( sin__q_RR_q2_joint__ *  cos__q_RR_q3_joint__));
    (*this)(2,1) = (( cos__q_RR_q2_joint__ *  cos__q_RR_q3_joint__) - ( sin__q_RR_q2_joint__ *  sin__q_RR_q3_joint__));
    (*this)(2,3) = (((( 0.17 *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( 0.17 *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) + ( 0.15 *  sin__q_RR_q2_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RR_q1_joint::Type_fr_trunk_X_fr_RR_q1_joint()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 1.0;
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.125;
    (*this)(1,0) = - 1.0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.075;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RR_q1_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RR_q1_joint::update(const state_t& q) {
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RR_q2_joint::Type_fr_trunk_X_fr_RR_q2_joint()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RR_q2_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RR_q2_joint::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RR_q1_joint__;
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(0,3) = (( 0.077 *  sin__q_RR_q1_joint__) -  0.125);
    (*this)(1,0) = - cos__q_RR_q1_joint__;
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(1,3) = ((- 0.077 *  cos__q_RR_q1_joint__) -  0.075);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RR_q3_joint::Type_fr_trunk_X_fr_RR_q3_joint()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RR_q3_joint& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RR_q3_joint::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) = ( sin__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(0,1) = (- sin__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(0,3) = (((( 0.15 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) + ( 0.077 *  sin__q_RR_q1_joint__)) -  0.125);
    (*this)(1,0) = (- cos__q_RR_q1_joint__ *  cos__q_RR_q2_joint__);
    (*this)(1,1) = ( cos__q_RR_q1_joint__ *  sin__q_RR_q2_joint__);
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(1,3) = ((((- 0.15 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) - ( 0.077 *  cos__q_RR_q1_joint__)) -  0.075);
    (*this)(2,0) =  sin__q_RR_q2_joint__;
    (*this)(2,1) =  cos__q_RR_q2_joint__;
    (*this)(2,3) = ( 0.15 *  sin__q_RR_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LF_hipassembly_X_fr_trunk::Type_fr_LF_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LF_hipassembly_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_LF_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LF_q1_joint__;
    (*this)(0,1) =  cos__q_LF_q1_joint__;
    (*this)(0,3) = (( 0.125 *  sin__q_LF_q1_joint__) - ( 0.075 *  cos__q_LF_q1_joint__));
    (*this)(1,0) = - cos__q_LF_q1_joint__;
    (*this)(1,1) = - sin__q_LF_q1_joint__;
    (*this)(1,3) = (( 0.075 *  sin__q_LF_q1_joint__) + ( 0.125 *  cos__q_LF_q1_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LF_hipassembly::Type_fr_trunk_X_fr_LF_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.125;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.075;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LF_hipassembly& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LF_hipassembly::update(const state_t& q) {
    static double sin__q_LF_q1_joint__;
    static double cos__q_LF_q1_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( q(LF_Q1_JOINT));
    cos__q_LF_q1_joint__ = std::cos( q(LF_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LF_q1_joint__;
    (*this)(0,1) = - cos__q_LF_q1_joint__;
    (*this)(1,0) =  cos__q_LF_q1_joint__;
    (*this)(1,1) = - sin__q_LF_q1_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LF_upperleg_X_fr_LF_hipassembly::Type_fr_LF_upperleg_X_fr_LF_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LF_upperleg_X_fr_LF_hipassembly& iit::mcorin::HomogeneousTransforms::Type_fr_LF_upperleg_X_fr_LF_hipassembly::update(const state_t& q) {
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LF_q2_joint__;
    (*this)(0,2) =  sin__q_LF_q2_joint__;
    (*this)(0,3) = (- 0.077 *  cos__q_LF_q2_joint__);
    (*this)(1,0) = - sin__q_LF_q2_joint__;
    (*this)(1,2) =  cos__q_LF_q2_joint__;
    (*this)(1,3) = ( 0.077 *  sin__q_LF_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LF_hipassembly_X_fr_LF_upperleg::Type_fr_LF_hipassembly_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.077;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LF_hipassembly_X_fr_LF_upperleg& iit::mcorin::HomogeneousTransforms::Type_fr_LF_hipassembly_X_fr_LF_upperleg::update(const state_t& q) {
    static double sin__q_LF_q2_joint__;
    static double cos__q_LF_q2_joint__;
    
    sin__q_LF_q2_joint__ = std::sin( q(LF_Q2_JOINT));
    cos__q_LF_q2_joint__ = std::cos( q(LF_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LF_q2_joint__;
    (*this)(0,1) = - sin__q_LF_q2_joint__;
    (*this)(2,0) =  sin__q_LF_q2_joint__;
    (*this)(2,1) =  cos__q_LF_q2_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LF_lowerleg_X_fr_LF_upperleg::Type_fr_LF_lowerleg_X_fr_LF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LF_lowerleg_X_fr_LF_upperleg& iit::mcorin::HomogeneousTransforms::Type_fr_LF_lowerleg_X_fr_LF_upperleg::update(const state_t& q) {
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q3_joint__;
    
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LF_q3_joint__;
    (*this)(0,1) =  sin__q_LF_q3_joint__;
    (*this)(0,3) = (- 0.15 *  cos__q_LF_q3_joint__);
    (*this)(1,0) = - sin__q_LF_q3_joint__;
    (*this)(1,1) =  cos__q_LF_q3_joint__;
    (*this)(1,3) = ( 0.15 *  sin__q_LF_q3_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LF_upperleg_X_fr_LF_lowerleg::Type_fr_LF_upperleg_X_fr_LF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.15;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LF_upperleg_X_fr_LF_lowerleg& iit::mcorin::HomogeneousTransforms::Type_fr_LF_upperleg_X_fr_LF_lowerleg::update(const state_t& q) {
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q3_joint__;
    
    sin__q_LF_q3_joint__ = std::sin( q(LF_Q3_JOINT));
    cos__q_LF_q3_joint__ = std::cos( q(LF_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LF_q3_joint__;
    (*this)(0,1) = - sin__q_LF_q3_joint__;
    (*this)(1,0) =  sin__q_LF_q3_joint__;
    (*this)(1,1) =  cos__q_LF_q3_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LM_hipassembly_X_fr_trunk::Type_fr_LM_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LM_hipassembly_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_LM_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LM_q1_joint__;
    (*this)(0,1) =  cos__q_LM_q1_joint__;
    (*this)(0,3) = (- 0.075 *  cos__q_LM_q1_joint__);
    (*this)(1,0) = - cos__q_LM_q1_joint__;
    (*this)(1,1) = - sin__q_LM_q1_joint__;
    (*this)(1,3) = ( 0.075 *  sin__q_LM_q1_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LM_hipassembly::Type_fr_trunk_X_fr_LM_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.075;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LM_hipassembly& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LM_hipassembly::update(const state_t& q) {
    static double sin__q_LM_q1_joint__;
    static double cos__q_LM_q1_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( q(LM_Q1_JOINT));
    cos__q_LM_q1_joint__ = std::cos( q(LM_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LM_q1_joint__;
    (*this)(0,1) = - cos__q_LM_q1_joint__;
    (*this)(1,0) =  cos__q_LM_q1_joint__;
    (*this)(1,1) = - sin__q_LM_q1_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LM_upperleg_X_fr_LM_hipassembly::Type_fr_LM_upperleg_X_fr_LM_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LM_upperleg_X_fr_LM_hipassembly& iit::mcorin::HomogeneousTransforms::Type_fr_LM_upperleg_X_fr_LM_hipassembly::update(const state_t& q) {
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LM_q2_joint__;
    (*this)(0,2) =  sin__q_LM_q2_joint__;
    (*this)(0,3) = (- 0.077 *  cos__q_LM_q2_joint__);
    (*this)(1,0) = - sin__q_LM_q2_joint__;
    (*this)(1,2) =  cos__q_LM_q2_joint__;
    (*this)(1,3) = ( 0.077 *  sin__q_LM_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LM_hipassembly_X_fr_LM_upperleg::Type_fr_LM_hipassembly_X_fr_LM_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.077;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LM_hipassembly_X_fr_LM_upperleg& iit::mcorin::HomogeneousTransforms::Type_fr_LM_hipassembly_X_fr_LM_upperleg::update(const state_t& q) {
    static double sin__q_LM_q2_joint__;
    static double cos__q_LM_q2_joint__;
    
    sin__q_LM_q2_joint__ = std::sin( q(LM_Q2_JOINT));
    cos__q_LM_q2_joint__ = std::cos( q(LM_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LM_q2_joint__;
    (*this)(0,1) = - sin__q_LM_q2_joint__;
    (*this)(2,0) =  sin__q_LM_q2_joint__;
    (*this)(2,1) =  cos__q_LM_q2_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LM_lowerleg_X_fr_LM_upperleg::Type_fr_LM_lowerleg_X_fr_LM_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LM_lowerleg_X_fr_LM_upperleg& iit::mcorin::HomogeneousTransforms::Type_fr_LM_lowerleg_X_fr_LM_upperleg::update(const state_t& q) {
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q3_joint__;
    
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LM_q3_joint__;
    (*this)(0,1) =  sin__q_LM_q3_joint__;
    (*this)(0,3) = (- 0.15 *  cos__q_LM_q3_joint__);
    (*this)(1,0) = - sin__q_LM_q3_joint__;
    (*this)(1,1) =  cos__q_LM_q3_joint__;
    (*this)(1,3) = ( 0.15 *  sin__q_LM_q3_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LM_upperleg_X_fr_LM_lowerleg::Type_fr_LM_upperleg_X_fr_LM_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.15;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LM_upperleg_X_fr_LM_lowerleg& iit::mcorin::HomogeneousTransforms::Type_fr_LM_upperleg_X_fr_LM_lowerleg::update(const state_t& q) {
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q3_joint__;
    
    sin__q_LM_q3_joint__ = std::sin( q(LM_Q3_JOINT));
    cos__q_LM_q3_joint__ = std::cos( q(LM_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LM_q3_joint__;
    (*this)(0,1) = - sin__q_LM_q3_joint__;
    (*this)(1,0) =  sin__q_LM_q3_joint__;
    (*this)(1,1) =  cos__q_LM_q3_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LR_hipassembly_X_fr_trunk::Type_fr_LR_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LR_hipassembly_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_LR_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LR_q1_joint__;
    (*this)(0,1) =  cos__q_LR_q1_joint__;
    (*this)(0,3) = ((- 0.125 *  sin__q_LR_q1_joint__) - ( 0.075 *  cos__q_LR_q1_joint__));
    (*this)(1,0) = - cos__q_LR_q1_joint__;
    (*this)(1,1) = - sin__q_LR_q1_joint__;
    (*this)(1,3) = (( 0.075 *  sin__q_LR_q1_joint__) - ( 0.125 *  cos__q_LR_q1_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LR_hipassembly::Type_fr_trunk_X_fr_LR_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.125;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.075;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LR_hipassembly& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_LR_hipassembly::update(const state_t& q) {
    static double sin__q_LR_q1_joint__;
    static double cos__q_LR_q1_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( q(LR_Q1_JOINT));
    cos__q_LR_q1_joint__ = std::cos( q(LR_Q1_JOINT));
    
    (*this)(0,0) = - sin__q_LR_q1_joint__;
    (*this)(0,1) = - cos__q_LR_q1_joint__;
    (*this)(1,0) =  cos__q_LR_q1_joint__;
    (*this)(1,1) = - sin__q_LR_q1_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LR_upperleg_X_fr_LR_hipassembly::Type_fr_LR_upperleg_X_fr_LR_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LR_upperleg_X_fr_LR_hipassembly& iit::mcorin::HomogeneousTransforms::Type_fr_LR_upperleg_X_fr_LR_hipassembly::update(const state_t& q) {
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LR_q2_joint__;
    (*this)(0,2) =  sin__q_LR_q2_joint__;
    (*this)(0,3) = (- 0.077 *  cos__q_LR_q2_joint__);
    (*this)(1,0) = - sin__q_LR_q2_joint__;
    (*this)(1,2) =  cos__q_LR_q2_joint__;
    (*this)(1,3) = ( 0.077 *  sin__q_LR_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LR_hipassembly_X_fr_LR_upperleg::Type_fr_LR_hipassembly_X_fr_LR_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.077;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LR_hipassembly_X_fr_LR_upperleg& iit::mcorin::HomogeneousTransforms::Type_fr_LR_hipassembly_X_fr_LR_upperleg::update(const state_t& q) {
    static double sin__q_LR_q2_joint__;
    static double cos__q_LR_q2_joint__;
    
    sin__q_LR_q2_joint__ = std::sin( q(LR_Q2_JOINT));
    cos__q_LR_q2_joint__ = std::cos( q(LR_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_LR_q2_joint__;
    (*this)(0,1) = - sin__q_LR_q2_joint__;
    (*this)(2,0) =  sin__q_LR_q2_joint__;
    (*this)(2,1) =  cos__q_LR_q2_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LR_lowerleg_X_fr_LR_upperleg::Type_fr_LR_lowerleg_X_fr_LR_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LR_lowerleg_X_fr_LR_upperleg& iit::mcorin::HomogeneousTransforms::Type_fr_LR_lowerleg_X_fr_LR_upperleg::update(const state_t& q) {
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q3_joint__;
    
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LR_q3_joint__;
    (*this)(0,1) =  sin__q_LR_q3_joint__;
    (*this)(0,3) = (- 0.15 *  cos__q_LR_q3_joint__);
    (*this)(1,0) = - sin__q_LR_q3_joint__;
    (*this)(1,1) =  cos__q_LR_q3_joint__;
    (*this)(1,3) = ( 0.15 *  sin__q_LR_q3_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_LR_upperleg_X_fr_LR_lowerleg::Type_fr_LR_upperleg_X_fr_LR_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.15;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_LR_upperleg_X_fr_LR_lowerleg& iit::mcorin::HomogeneousTransforms::Type_fr_LR_upperleg_X_fr_LR_lowerleg::update(const state_t& q) {
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q3_joint__;
    
    sin__q_LR_q3_joint__ = std::sin( q(LR_Q3_JOINT));
    cos__q_LR_q3_joint__ = std::cos( q(LR_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_LR_q3_joint__;
    (*this)(0,1) = - sin__q_LR_q3_joint__;
    (*this)(1,0) =  sin__q_LR_q3_joint__;
    (*this)(1,1) =  cos__q_LR_q3_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RF_hipassembly_X_fr_trunk::Type_fr_RF_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RF_hipassembly_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_RF_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RF_q1_joint__;
    (*this)(0,1) = - cos__q_RF_q1_joint__;
    (*this)(0,3) = ((- 0.125 *  sin__q_RF_q1_joint__) - ( 0.075 *  cos__q_RF_q1_joint__));
    (*this)(1,0) =  cos__q_RF_q1_joint__;
    (*this)(1,1) =  sin__q_RF_q1_joint__;
    (*this)(1,3) = (( 0.075 *  sin__q_RF_q1_joint__) - ( 0.125 *  cos__q_RF_q1_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RF_hipassembly::Type_fr_trunk_X_fr_RF_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.125;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.075;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RF_hipassembly& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RF_hipassembly::update(const state_t& q) {
    static double sin__q_RF_q1_joint__;
    static double cos__q_RF_q1_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( q(RF_Q1_JOINT));
    cos__q_RF_q1_joint__ = std::cos( q(RF_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RF_q1_joint__;
    (*this)(0,1) =  cos__q_RF_q1_joint__;
    (*this)(1,0) = - cos__q_RF_q1_joint__;
    (*this)(1,1) =  sin__q_RF_q1_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RF_upperleg_X_fr_RF_hipassembly::Type_fr_RF_upperleg_X_fr_RF_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RF_upperleg_X_fr_RF_hipassembly& iit::mcorin::HomogeneousTransforms::Type_fr_RF_upperleg_X_fr_RF_hipassembly::update(const state_t& q) {
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RF_q2_joint__;
    (*this)(0,2) =  sin__q_RF_q2_joint__;
    (*this)(0,3) = (- 0.077 *  cos__q_RF_q2_joint__);
    (*this)(1,0) = - sin__q_RF_q2_joint__;
    (*this)(1,2) =  cos__q_RF_q2_joint__;
    (*this)(1,3) = ( 0.077 *  sin__q_RF_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RF_hipassembly_X_fr_RF_upperleg::Type_fr_RF_hipassembly_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.077;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RF_hipassembly_X_fr_RF_upperleg& iit::mcorin::HomogeneousTransforms::Type_fr_RF_hipassembly_X_fr_RF_upperleg::update(const state_t& q) {
    static double sin__q_RF_q2_joint__;
    static double cos__q_RF_q2_joint__;
    
    sin__q_RF_q2_joint__ = std::sin( q(RF_Q2_JOINT));
    cos__q_RF_q2_joint__ = std::cos( q(RF_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RF_q2_joint__;
    (*this)(0,1) = - sin__q_RF_q2_joint__;
    (*this)(2,0) =  sin__q_RF_q2_joint__;
    (*this)(2,1) =  cos__q_RF_q2_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RF_lowerleg_X_fr_RF_upperleg::Type_fr_RF_lowerleg_X_fr_RF_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RF_lowerleg_X_fr_RF_upperleg& iit::mcorin::HomogeneousTransforms::Type_fr_RF_lowerleg_X_fr_RF_upperleg::update(const state_t& q) {
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q3_joint__;
    
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RF_q3_joint__;
    (*this)(0,1) =  sin__q_RF_q3_joint__;
    (*this)(0,3) = (- 0.15 *  cos__q_RF_q3_joint__);
    (*this)(1,0) = - sin__q_RF_q3_joint__;
    (*this)(1,1) =  cos__q_RF_q3_joint__;
    (*this)(1,3) = ( 0.15 *  sin__q_RF_q3_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RF_upperleg_X_fr_RF_lowerleg::Type_fr_RF_upperleg_X_fr_RF_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.15;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RF_upperleg_X_fr_RF_lowerleg& iit::mcorin::HomogeneousTransforms::Type_fr_RF_upperleg_X_fr_RF_lowerleg::update(const state_t& q) {
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q3_joint__;
    
    sin__q_RF_q3_joint__ = std::sin( q(RF_Q3_JOINT));
    cos__q_RF_q3_joint__ = std::cos( q(RF_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RF_q3_joint__;
    (*this)(0,1) = - sin__q_RF_q3_joint__;
    (*this)(1,0) =  sin__q_RF_q3_joint__;
    (*this)(1,1) =  cos__q_RF_q3_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RM_hipassembly_X_fr_trunk::Type_fr_RM_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RM_hipassembly_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_RM_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RM_q1_joint__;
    (*this)(0,1) = - cos__q_RM_q1_joint__;
    (*this)(0,3) = (- 0.075 *  cos__q_RM_q1_joint__);
    (*this)(1,0) =  cos__q_RM_q1_joint__;
    (*this)(1,1) =  sin__q_RM_q1_joint__;
    (*this)(1,3) = ( 0.075 *  sin__q_RM_q1_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RM_hipassembly::Type_fr_trunk_X_fr_RM_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.075;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RM_hipassembly& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RM_hipassembly::update(const state_t& q) {
    static double sin__q_RM_q1_joint__;
    static double cos__q_RM_q1_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( q(RM_Q1_JOINT));
    cos__q_RM_q1_joint__ = std::cos( q(RM_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RM_q1_joint__;
    (*this)(0,1) =  cos__q_RM_q1_joint__;
    (*this)(1,0) = - cos__q_RM_q1_joint__;
    (*this)(1,1) =  sin__q_RM_q1_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RM_upperleg_X_fr_RM_hipassembly::Type_fr_RM_upperleg_X_fr_RM_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RM_upperleg_X_fr_RM_hipassembly& iit::mcorin::HomogeneousTransforms::Type_fr_RM_upperleg_X_fr_RM_hipassembly::update(const state_t& q) {
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RM_q2_joint__;
    (*this)(0,2) =  sin__q_RM_q2_joint__;
    (*this)(0,3) = (- 0.077 *  cos__q_RM_q2_joint__);
    (*this)(1,0) = - sin__q_RM_q2_joint__;
    (*this)(1,2) =  cos__q_RM_q2_joint__;
    (*this)(1,3) = ( 0.077 *  sin__q_RM_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RM_hipassembly_X_fr_RM_upperleg::Type_fr_RM_hipassembly_X_fr_RM_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.077;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RM_hipassembly_X_fr_RM_upperleg& iit::mcorin::HomogeneousTransforms::Type_fr_RM_hipassembly_X_fr_RM_upperleg::update(const state_t& q) {
    static double sin__q_RM_q2_joint__;
    static double cos__q_RM_q2_joint__;
    
    sin__q_RM_q2_joint__ = std::sin( q(RM_Q2_JOINT));
    cos__q_RM_q2_joint__ = std::cos( q(RM_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RM_q2_joint__;
    (*this)(0,1) = - sin__q_RM_q2_joint__;
    (*this)(2,0) =  sin__q_RM_q2_joint__;
    (*this)(2,1) =  cos__q_RM_q2_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RM_lowerleg_X_fr_RM_upperleg::Type_fr_RM_lowerleg_X_fr_RM_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RM_lowerleg_X_fr_RM_upperleg& iit::mcorin::HomogeneousTransforms::Type_fr_RM_lowerleg_X_fr_RM_upperleg::update(const state_t& q) {
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q3_joint__;
    
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RM_q3_joint__;
    (*this)(0,1) =  sin__q_RM_q3_joint__;
    (*this)(0,3) = (- 0.15 *  cos__q_RM_q3_joint__);
    (*this)(1,0) = - sin__q_RM_q3_joint__;
    (*this)(1,1) =  cos__q_RM_q3_joint__;
    (*this)(1,3) = ( 0.15 *  sin__q_RM_q3_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RM_upperleg_X_fr_RM_lowerleg::Type_fr_RM_upperleg_X_fr_RM_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.15;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RM_upperleg_X_fr_RM_lowerleg& iit::mcorin::HomogeneousTransforms::Type_fr_RM_upperleg_X_fr_RM_lowerleg::update(const state_t& q) {
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q3_joint__;
    
    sin__q_RM_q3_joint__ = std::sin( q(RM_Q3_JOINT));
    cos__q_RM_q3_joint__ = std::cos( q(RM_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RM_q3_joint__;
    (*this)(0,1) = - sin__q_RM_q3_joint__;
    (*this)(1,0) =  sin__q_RM_q3_joint__;
    (*this)(1,1) =  cos__q_RM_q3_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RR_hipassembly_X_fr_trunk::Type_fr_RR_hipassembly_X_fr_trunk()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RR_hipassembly_X_fr_trunk& iit::mcorin::HomogeneousTransforms::Type_fr_RR_hipassembly_X_fr_trunk::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RR_q1_joint__;
    (*this)(0,1) = - cos__q_RR_q1_joint__;
    (*this)(0,3) = (( 0.125 *  sin__q_RR_q1_joint__) - ( 0.075 *  cos__q_RR_q1_joint__));
    (*this)(1,0) =  cos__q_RR_q1_joint__;
    (*this)(1,1) =  sin__q_RR_q1_joint__;
    (*this)(1,3) = (( 0.075 *  sin__q_RR_q1_joint__) + ( 0.125 *  cos__q_RR_q1_joint__));
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RR_hipassembly::Type_fr_trunk_X_fr_RR_hipassembly()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = - 0.125;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.075;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RR_hipassembly& iit::mcorin::HomogeneousTransforms::Type_fr_trunk_X_fr_RR_hipassembly::update(const state_t& q) {
    static double sin__q_RR_q1_joint__;
    static double cos__q_RR_q1_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( q(RR_Q1_JOINT));
    cos__q_RR_q1_joint__ = std::cos( q(RR_Q1_JOINT));
    
    (*this)(0,0) =  sin__q_RR_q1_joint__;
    (*this)(0,1) =  cos__q_RR_q1_joint__;
    (*this)(1,0) = - cos__q_RR_q1_joint__;
    (*this)(1,1) =  sin__q_RR_q1_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RR_upperleg_X_fr_RR_hipassembly::Type_fr_RR_upperleg_X_fr_RR_hipassembly()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RR_upperleg_X_fr_RR_hipassembly& iit::mcorin::HomogeneousTransforms::Type_fr_RR_upperleg_X_fr_RR_hipassembly::update(const state_t& q) {
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RR_q2_joint__;
    (*this)(0,2) =  sin__q_RR_q2_joint__;
    (*this)(0,3) = (- 0.077 *  cos__q_RR_q2_joint__);
    (*this)(1,0) = - sin__q_RR_q2_joint__;
    (*this)(1,2) =  cos__q_RR_q2_joint__;
    (*this)(1,3) = ( 0.077 *  sin__q_RR_q2_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RR_hipassembly_X_fr_RR_upperleg::Type_fr_RR_hipassembly_X_fr_RR_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.077;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RR_hipassembly_X_fr_RR_upperleg& iit::mcorin::HomogeneousTransforms::Type_fr_RR_hipassembly_X_fr_RR_upperleg::update(const state_t& q) {
    static double sin__q_RR_q2_joint__;
    static double cos__q_RR_q2_joint__;
    
    sin__q_RR_q2_joint__ = std::sin( q(RR_Q2_JOINT));
    cos__q_RR_q2_joint__ = std::cos( q(RR_Q2_JOINT));
    
    (*this)(0,0) =  cos__q_RR_q2_joint__;
    (*this)(0,1) = - sin__q_RR_q2_joint__;
    (*this)(2,0) =  sin__q_RR_q2_joint__;
    (*this)(2,1) =  cos__q_RR_q2_joint__;
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RR_lowerleg_X_fr_RR_upperleg::Type_fr_RR_lowerleg_X_fr_RR_upperleg()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RR_lowerleg_X_fr_RR_upperleg& iit::mcorin::HomogeneousTransforms::Type_fr_RR_lowerleg_X_fr_RR_upperleg::update(const state_t& q) {
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q3_joint__;
    
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RR_q3_joint__;
    (*this)(0,1) =  sin__q_RR_q3_joint__;
    (*this)(0,3) = (- 0.15 *  cos__q_RR_q3_joint__);
    (*this)(1,0) = - sin__q_RR_q3_joint__;
    (*this)(1,1) =  cos__q_RR_q3_joint__;
    (*this)(1,3) = ( 0.15 *  sin__q_RR_q3_joint__);
    return *this;
}
iit::mcorin::HomogeneousTransforms::Type_fr_RR_upperleg_X_fr_RR_lowerleg::Type_fr_RR_upperleg_X_fr_RR_lowerleg()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.15;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::mcorin::HomogeneousTransforms::Type_fr_RR_upperleg_X_fr_RR_lowerleg& iit::mcorin::HomogeneousTransforms::Type_fr_RR_upperleg_X_fr_RR_lowerleg::update(const state_t& q) {
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q3_joint__;
    
    sin__q_RR_q3_joint__ = std::sin( q(RR_Q3_JOINT));
    cos__q_RR_q3_joint__ = std::cos( q(RR_Q3_JOINT));
    
    (*this)(0,0) =  cos__q_RR_q3_joint__;
    (*this)(0,1) = - sin__q_RR_q3_joint__;
    (*this)(1,0) =  sin__q_RR_q3_joint__;
    (*this)(1,1) =  cos__q_RR_q3_joint__;
    return *this;
}


#include "jacobians.h"


iit::mcorin::Jacobians::Jacobians
    ()
     : 
    fr_trunk_J_LF_foot(), 
    fr_trunk_J_LM_foot(), 
    fr_trunk_J_LR_foot(), 
    fr_trunk_J_RF_foot(), 
    fr_trunk_J_RM_foot(), 
    fr_trunk_J_RR_foot()
{
    updateParameters();
}


void iit::mcorin::Jacobians::updateParameters() {
}


iit::mcorin::Jacobians::Type_fr_trunk_J_LF_foot::Type_fr_trunk_J_LF_foot()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(5,0) = 0;
}

const iit::mcorin::Jacobians::Type_fr_trunk_J_LF_foot& iit::mcorin::Jacobians::Type_fr_trunk_J_LF_foot::update(const JointState& jState) {
    static double sin__q_LF_q1_joint__;
    static double sin__q_LF_q2_joint__;
    static double sin__q_LF_q3_joint__;
    static double cos__q_LF_q1_joint__;
    static double cos__q_LF_q2_joint__;
    static double cos__q_LF_q3_joint__;
    
    sin__q_LF_q1_joint__ = std::sin( jState(LF_Q1_JOINT));
    sin__q_LF_q2_joint__ = std::sin( jState(LF_Q2_JOINT));
    sin__q_LF_q3_joint__ = std::sin( jState(LF_Q3_JOINT));
    cos__q_LF_q1_joint__ = std::cos( jState(LF_Q1_JOINT));
    cos__q_LF_q2_joint__ = std::cos( jState(LF_Q2_JOINT));
    cos__q_LF_q3_joint__ = std::cos( jState(LF_Q3_JOINT));
    
    (*this)(0,1) =  cos__q_LF_q1_joint__;
    (*this)(0,2) =  cos__q_LF_q1_joint__;
    (*this)(1,1) =  sin__q_LF_q1_joint__;
    (*this)(1,2) =  sin__q_LF_q1_joint__;
    (*this)(3,0) = (((((( 0.17 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - ((( 0.17 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) - (( 0.15 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__)) - ( 0.077 *  cos__q_LF_q1_joint__));
    (*this)(3,1) = ((((( 0.17 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.17 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) + (( 0.15 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__));
    (*this)(3,2) = (((( 0.17 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + ((( 0.17 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(4,0) = (((((( 0.17 *  sin__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - ((( 0.17 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) - (( 0.15 *  sin__q_LF_q1_joint__) *  cos__q_LF_q2_joint__)) - ( 0.077 *  sin__q_LF_q1_joint__));
    (*this)(4,1) = (((((- 0.17 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - ((( 0.17 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) - (( 0.15 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__));
    (*this)(4,2) = ((((- 0.17 *  cos__q_LF_q1_joint__) *  cos__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) - ((( 0.17 *  cos__q_LF_q1_joint__) *  sin__q_LF_q2_joint__) *  cos__q_LF_q3_joint__));
    (*this)(5,1) = ((((- 0.17 *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__) + (( 0.17 *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__)) + ( 0.15 *  cos__q_LF_q2_joint__));
    (*this)(5,2) = ((( 0.17 *  cos__q_LF_q2_joint__) *  cos__q_LF_q3_joint__) - (( 0.17 *  sin__q_LF_q2_joint__) *  sin__q_LF_q3_joint__));
    return *this;
}
iit::mcorin::Jacobians::Type_fr_trunk_J_LM_foot::Type_fr_trunk_J_LM_foot()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(5,0) = 0;
}

const iit::mcorin::Jacobians::Type_fr_trunk_J_LM_foot& iit::mcorin::Jacobians::Type_fr_trunk_J_LM_foot::update(const JointState& jState) {
    static double sin__q_LM_q1_joint__;
    static double sin__q_LM_q2_joint__;
    static double sin__q_LM_q3_joint__;
    static double cos__q_LM_q1_joint__;
    static double cos__q_LM_q2_joint__;
    static double cos__q_LM_q3_joint__;
    
    sin__q_LM_q1_joint__ = std::sin( jState(LM_Q1_JOINT));
    sin__q_LM_q2_joint__ = std::sin( jState(LM_Q2_JOINT));
    sin__q_LM_q3_joint__ = std::sin( jState(LM_Q3_JOINT));
    cos__q_LM_q1_joint__ = std::cos( jState(LM_Q1_JOINT));
    cos__q_LM_q2_joint__ = std::cos( jState(LM_Q2_JOINT));
    cos__q_LM_q3_joint__ = std::cos( jState(LM_Q3_JOINT));
    
    (*this)(0,1) =  cos__q_LM_q1_joint__;
    (*this)(0,2) =  cos__q_LM_q1_joint__;
    (*this)(1,1) =  sin__q_LM_q1_joint__;
    (*this)(1,2) =  sin__q_LM_q1_joint__;
    (*this)(3,0) = (((((( 0.17 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.17 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) - (( 0.15 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) - ( 0.077 *  cos__q_LM_q1_joint__));
    (*this)(3,1) = ((((( 0.17 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.17 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) + (( 0.15 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__));
    (*this)(3,2) = (((( 0.17 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + ((( 0.17 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(4,0) = (((((( 0.17 *  sin__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.17 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) - (( 0.15 *  sin__q_LM_q1_joint__) *  cos__q_LM_q2_joint__)) - ( 0.077 *  sin__q_LM_q1_joint__));
    (*this)(4,1) = (((((- 0.17 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.17 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) - (( 0.15 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__));
    (*this)(4,2) = ((((- 0.17 *  cos__q_LM_q1_joint__) *  cos__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) - ((( 0.17 *  cos__q_LM_q1_joint__) *  sin__q_LM_q2_joint__) *  cos__q_LM_q3_joint__));
    (*this)(5,1) = ((((- 0.17 *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__) + (( 0.17 *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__)) + ( 0.15 *  cos__q_LM_q2_joint__));
    (*this)(5,2) = ((( 0.17 *  cos__q_LM_q2_joint__) *  cos__q_LM_q3_joint__) - (( 0.17 *  sin__q_LM_q2_joint__) *  sin__q_LM_q3_joint__));
    return *this;
}
iit::mcorin::Jacobians::Type_fr_trunk_J_LR_foot::Type_fr_trunk_J_LR_foot()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(5,0) = 0;
}

const iit::mcorin::Jacobians::Type_fr_trunk_J_LR_foot& iit::mcorin::Jacobians::Type_fr_trunk_J_LR_foot::update(const JointState& jState) {
    static double sin__q_LR_q1_joint__;
    static double sin__q_LR_q2_joint__;
    static double sin__q_LR_q3_joint__;
    static double cos__q_LR_q1_joint__;
    static double cos__q_LR_q2_joint__;
    static double cos__q_LR_q3_joint__;
    
    sin__q_LR_q1_joint__ = std::sin( jState(LR_Q1_JOINT));
    sin__q_LR_q2_joint__ = std::sin( jState(LR_Q2_JOINT));
    sin__q_LR_q3_joint__ = std::sin( jState(LR_Q3_JOINT));
    cos__q_LR_q1_joint__ = std::cos( jState(LR_Q1_JOINT));
    cos__q_LR_q2_joint__ = std::cos( jState(LR_Q2_JOINT));
    cos__q_LR_q3_joint__ = std::cos( jState(LR_Q3_JOINT));
    
    (*this)(0,1) =  cos__q_LR_q1_joint__;
    (*this)(0,2) =  cos__q_LR_q1_joint__;
    (*this)(1,1) =  sin__q_LR_q1_joint__;
    (*this)(1,2) =  sin__q_LR_q1_joint__;
    (*this)(3,0) = (((((( 0.17 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - ((( 0.17 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) - (( 0.15 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__)) - ( 0.077 *  cos__q_LR_q1_joint__));
    (*this)(3,1) = ((((( 0.17 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.17 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) + (( 0.15 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__));
    (*this)(3,2) = (((( 0.17 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + ((( 0.17 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(4,0) = (((((( 0.17 *  sin__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - ((( 0.17 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) - (( 0.15 *  sin__q_LR_q1_joint__) *  cos__q_LR_q2_joint__)) - ( 0.077 *  sin__q_LR_q1_joint__));
    (*this)(4,1) = (((((- 0.17 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - ((( 0.17 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) - (( 0.15 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__));
    (*this)(4,2) = ((((- 0.17 *  cos__q_LR_q1_joint__) *  cos__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) - ((( 0.17 *  cos__q_LR_q1_joint__) *  sin__q_LR_q2_joint__) *  cos__q_LR_q3_joint__));
    (*this)(5,1) = ((((- 0.17 *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__) + (( 0.17 *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__)) + ( 0.15 *  cos__q_LR_q2_joint__));
    (*this)(5,2) = ((( 0.17 *  cos__q_LR_q2_joint__) *  cos__q_LR_q3_joint__) - (( 0.17 *  sin__q_LR_q2_joint__) *  sin__q_LR_q3_joint__));
    return *this;
}
iit::mcorin::Jacobians::Type_fr_trunk_J_RF_foot::Type_fr_trunk_J_RF_foot()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(5,0) = 0;
}

const iit::mcorin::Jacobians::Type_fr_trunk_J_RF_foot& iit::mcorin::Jacobians::Type_fr_trunk_J_RF_foot::update(const JointState& jState) {
    static double sin__q_RF_q1_joint__;
    static double sin__q_RF_q2_joint__;
    static double sin__q_RF_q3_joint__;
    static double cos__q_RF_q1_joint__;
    static double cos__q_RF_q2_joint__;
    static double cos__q_RF_q3_joint__;
    
    sin__q_RF_q1_joint__ = std::sin( jState(RF_Q1_JOINT));
    sin__q_RF_q2_joint__ = std::sin( jState(RF_Q2_JOINT));
    sin__q_RF_q3_joint__ = std::sin( jState(RF_Q3_JOINT));
    cos__q_RF_q1_joint__ = std::cos( jState(RF_Q1_JOINT));
    cos__q_RF_q2_joint__ = std::cos( jState(RF_Q2_JOINT));
    cos__q_RF_q3_joint__ = std::cos( jState(RF_Q3_JOINT));
    
    (*this)(0,1) = - cos__q_RF_q1_joint__;
    (*this)(0,2) = - cos__q_RF_q1_joint__;
    (*this)(1,1) = - sin__q_RF_q1_joint__;
    (*this)(1,2) = - sin__q_RF_q1_joint__;
    (*this)(3,0) = ((((((- 0.17 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + ((( 0.17 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) + (( 0.15 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__)) + ( 0.077 *  cos__q_RF_q1_joint__));
    (*this)(3,1) = (((((- 0.17 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - ((( 0.17 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) - (( 0.15 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__));
    (*this)(3,2) = ((((- 0.17 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) - ((( 0.17 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(4,0) = ((((((- 0.17 *  sin__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + ((( 0.17 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) + (( 0.15 *  sin__q_RF_q1_joint__) *  cos__q_RF_q2_joint__)) + ( 0.077 *  sin__q_RF_q1_joint__));
    (*this)(4,1) = ((((( 0.17 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + ((( 0.17 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) + (( 0.15 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__));
    (*this)(4,2) = (((( 0.17 *  cos__q_RF_q1_joint__) *  cos__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + ((( 0.17 *  cos__q_RF_q1_joint__) *  sin__q_RF_q2_joint__) *  cos__q_RF_q3_joint__));
    (*this)(5,1) = ((((- 0.17 *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__) + (( 0.17 *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__)) + ( 0.15 *  cos__q_RF_q2_joint__));
    (*this)(5,2) = ((( 0.17 *  cos__q_RF_q2_joint__) *  cos__q_RF_q3_joint__) - (( 0.17 *  sin__q_RF_q2_joint__) *  sin__q_RF_q3_joint__));
    return *this;
}
iit::mcorin::Jacobians::Type_fr_trunk_J_RM_foot::Type_fr_trunk_J_RM_foot()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(5,0) = 0;
}

const iit::mcorin::Jacobians::Type_fr_trunk_J_RM_foot& iit::mcorin::Jacobians::Type_fr_trunk_J_RM_foot::update(const JointState& jState) {
    static double sin__q_RM_q1_joint__;
    static double sin__q_RM_q2_joint__;
    static double sin__q_RM_q3_joint__;
    static double cos__q_RM_q1_joint__;
    static double cos__q_RM_q2_joint__;
    static double cos__q_RM_q3_joint__;
    
    sin__q_RM_q1_joint__ = std::sin( jState(RM_Q1_JOINT));
    sin__q_RM_q2_joint__ = std::sin( jState(RM_Q2_JOINT));
    sin__q_RM_q3_joint__ = std::sin( jState(RM_Q3_JOINT));
    cos__q_RM_q1_joint__ = std::cos( jState(RM_Q1_JOINT));
    cos__q_RM_q2_joint__ = std::cos( jState(RM_Q2_JOINT));
    cos__q_RM_q3_joint__ = std::cos( jState(RM_Q3_JOINT));
    
    (*this)(0,1) = - cos__q_RM_q1_joint__;
    (*this)(0,2) = - cos__q_RM_q1_joint__;
    (*this)(1,1) = - sin__q_RM_q1_joint__;
    (*this)(1,2) = - sin__q_RM_q1_joint__;
    (*this)(3,0) = ((((((- 0.17 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + ((( 0.17 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) + (( 0.15 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) + ( 0.077 *  cos__q_RM_q1_joint__));
    (*this)(3,1) = (((((- 0.17 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.17 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) - (( 0.15 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__));
    (*this)(3,2) = ((((- 0.17 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) - ((( 0.17 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(4,0) = ((((((- 0.17 *  sin__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + ((( 0.17 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) + (( 0.15 *  sin__q_RM_q1_joint__) *  cos__q_RM_q2_joint__)) + ( 0.077 *  sin__q_RM_q1_joint__));
    (*this)(4,1) = ((((( 0.17 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + ((( 0.17 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) + (( 0.15 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__));
    (*this)(4,2) = (((( 0.17 *  cos__q_RM_q1_joint__) *  cos__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + ((( 0.17 *  cos__q_RM_q1_joint__) *  sin__q_RM_q2_joint__) *  cos__q_RM_q3_joint__));
    (*this)(5,1) = ((((- 0.17 *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__) + (( 0.17 *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__)) + ( 0.15 *  cos__q_RM_q2_joint__));
    (*this)(5,2) = ((( 0.17 *  cos__q_RM_q2_joint__) *  cos__q_RM_q3_joint__) - (( 0.17 *  sin__q_RM_q2_joint__) *  sin__q_RM_q3_joint__));
    return *this;
}
iit::mcorin::Jacobians::Type_fr_trunk_J_RR_foot::Type_fr_trunk_J_RR_foot()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(5,0) = 0;
}

const iit::mcorin::Jacobians::Type_fr_trunk_J_RR_foot& iit::mcorin::Jacobians::Type_fr_trunk_J_RR_foot::update(const JointState& jState) {
    static double sin__q_RR_q1_joint__;
    static double sin__q_RR_q2_joint__;
    static double sin__q_RR_q3_joint__;
    static double cos__q_RR_q1_joint__;
    static double cos__q_RR_q2_joint__;
    static double cos__q_RR_q3_joint__;
    
    sin__q_RR_q1_joint__ = std::sin( jState(RR_Q1_JOINT));
    sin__q_RR_q2_joint__ = std::sin( jState(RR_Q2_JOINT));
    sin__q_RR_q3_joint__ = std::sin( jState(RR_Q3_JOINT));
    cos__q_RR_q1_joint__ = std::cos( jState(RR_Q1_JOINT));
    cos__q_RR_q2_joint__ = std::cos( jState(RR_Q2_JOINT));
    cos__q_RR_q3_joint__ = std::cos( jState(RR_Q3_JOINT));
    
    (*this)(0,1) = - cos__q_RR_q1_joint__;
    (*this)(0,2) = - cos__q_RR_q1_joint__;
    (*this)(1,1) = - sin__q_RR_q1_joint__;
    (*this)(1,2) = - sin__q_RR_q1_joint__;
    (*this)(3,0) = ((((((- 0.17 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((( 0.17 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) + (( 0.15 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__)) + ( 0.077 *  cos__q_RR_q1_joint__));
    (*this)(3,1) = (((((- 0.17 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - ((( 0.17 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) - (( 0.15 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__));
    (*this)(3,2) = ((((- 0.17 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) - ((( 0.17 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(4,0) = ((((((- 0.17 *  sin__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((( 0.17 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) + (( 0.15 *  sin__q_RR_q1_joint__) *  cos__q_RR_q2_joint__)) + ( 0.077 *  sin__q_RR_q1_joint__));
    (*this)(4,1) = ((((( 0.17 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((( 0.17 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) + (( 0.15 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__));
    (*this)(4,2) = (((( 0.17 *  cos__q_RR_q1_joint__) *  cos__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + ((( 0.17 *  cos__q_RR_q1_joint__) *  sin__q_RR_q2_joint__) *  cos__q_RR_q3_joint__));
    (*this)(5,1) = ((((- 0.17 *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__) + (( 0.17 *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__)) + ( 0.15 *  cos__q_RR_q2_joint__));
    (*this)(5,2) = ((( 0.17 *  cos__q_RR_q2_joint__) *  cos__q_RR_q3_joint__) - (( 0.17 *  sin__q_RR_q2_joint__) *  sin__q_RR_q3_joint__));
    return *this;
}

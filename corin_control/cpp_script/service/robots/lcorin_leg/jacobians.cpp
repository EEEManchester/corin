#include "jacobians.h"


iit::LCORIN_LEG::Jacobians::Jacobians
    ()
     : 
    fr_base_J_ee()
{
    updateParameters();
}


void iit::LCORIN_LEG::Jacobians::updateParameters() {
}


iit::LCORIN_LEG::Jacobians::Type_fr_base_J_ee::Type_fr_base_J_ee()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(5,0) = 0;
}

const iit::LCORIN_LEG::Jacobians::Type_fr_base_J_ee& iit::LCORIN_LEG::Jacobians::Type_fr_base_J_ee::update(const JointState& jState) {
    static double sin__q_q1__;
    static double sin__q_q2__;
    static double sin__q_q3__;
    static double cos__q_q1__;
    static double cos__q_q2__;
    static double cos__q_q3__;
    
    sin__q_q1__ = std::sin( jState(Q1));
    sin__q_q2__ = std::sin( jState(Q2));
    sin__q_q3__ = std::sin( jState(Q3));
    cos__q_q1__ = std::cos( jState(Q1));
    cos__q_q2__ = std::cos( jState(Q2));
    cos__q_q3__ = std::cos( jState(Q3));
    
    (*this)(0,1) =  sin__q_q1__;
    (*this)(0,2) =  sin__q_q1__;
    (*this)(1,1) = - cos__q_q1__;
    (*this)(1,2) = - cos__q_q1__;
    (*this)(3,0) = (((((( 0.17 *  sin__q_q1__) *  sin__q_q2__) *  sin__q_q3__) - ((( 0.17 *  sin__q_q1__) *  cos__q_q2__) *  cos__q_q3__)) - (( 0.15 *  sin__q_q1__) *  cos__q_q2__)) - ( 0.077 *  sin__q_q1__));
    (*this)(3,1) = (((((- 0.17 *  cos__q_q1__) *  cos__q_q2__) *  sin__q_q3__) - ((( 0.17 *  cos__q_q1__) *  sin__q_q2__) *  cos__q_q3__)) - (( 0.15 *  cos__q_q1__) *  sin__q_q2__));
    (*this)(3,2) = ((((- 0.17 *  cos__q_q1__) *  cos__q_q2__) *  sin__q_q3__) - ((( 0.17 *  cos__q_q1__) *  sin__q_q2__) *  cos__q_q3__));
    (*this)(4,0) = ((((((- 0.17 *  cos__q_q1__) *  sin__q_q2__) *  sin__q_q3__) + ((( 0.17 *  cos__q_q1__) *  cos__q_q2__) *  cos__q_q3__)) + (( 0.15 *  cos__q_q1__) *  cos__q_q2__)) + ( 0.077 *  cos__q_q1__));
    (*this)(4,1) = (((((- 0.17 *  sin__q_q1__) *  cos__q_q2__) *  sin__q_q3__) - ((( 0.17 *  sin__q_q1__) *  sin__q_q2__) *  cos__q_q3__)) - (( 0.15 *  sin__q_q1__) *  sin__q_q2__));
    (*this)(4,2) = ((((- 0.17 *  sin__q_q1__) *  cos__q_q2__) *  sin__q_q3__) - ((( 0.17 *  sin__q_q1__) *  sin__q_q2__) *  cos__q_q3__));
    (*this)(5,1) = ((((- 0.17 *  sin__q_q2__) *  sin__q_q3__) + (( 0.17 *  cos__q_q2__) *  cos__q_q3__)) + ( 0.15 *  cos__q_q2__));
    (*this)(5,2) = ((( 0.17 *  cos__q_q2__) *  cos__q_q3__) - (( 0.17 *  sin__q_q2__) *  sin__q_q3__));
    return *this;
}

#include "jacobians.h"


iit::SLM::Jacobians::Jacobians
    ()
     : 
    fr_base_J_ee()
{
    updateParameters();
}


void iit::SLM::Jacobians::updateParameters() {
}


iit::SLM::Jacobians::Type_fr_base_J_ee::Type_fr_base_J_ee()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = - 1.0;
    (*this)(2,0) = 0;
    (*this)(4,0) = 0;
}

const iit::SLM::Jacobians::Type_fr_base_J_ee& iit::SLM::Jacobians::Type_fr_base_J_ee::update(const JointState& jState) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( jState(Q1));
    cos__q_q1__ = std::cos( jState(Q1));
    
    (*this)(3,0) = (- 0.15 *  sin__q_q1__);
    (*this)(5,0) = ( 0.15 *  cos__q_q1__);
    return *this;
}

#include "transforms.h"

// Constructors

iit::SLM::MotionTransforms::MotionTransforms
    ()
     :
    fr_base_X_fr_l1(),
    fr_base_X_ee(),
    fr_base_X_fr_q1(),
    fr_l1_X_fr_base()
{
    updateParameters();
}
void iit::SLM::MotionTransforms::updateParameters() {
}

iit::SLM::ForceTransforms::ForceTransforms
    ()
     :
    fr_base_X_fr_l1(),
    fr_base_X_ee(),
    fr_base_X_fr_q1(),
    fr_l1_X_fr_base()
{
    updateParameters();
}
void iit::SLM::ForceTransforms::updateParameters() {
}

iit::SLM::HomogeneousTransforms::HomogeneousTransforms
    ()
     :
    fr_base_X_fr_l1(),
    fr_base_X_ee(),
    fr_base_X_fr_q1(),
    fr_l1_X_fr_base()
{
    updateParameters();
}
void iit::SLM::HomogeneousTransforms::updateParameters() {
}

iit::SLM::MotionTransforms::Type_fr_base_X_fr_l1::Type_fr_base_X_fr_l1()
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
const iit::SLM::MotionTransforms::Type_fr_base_X_fr_l1& iit::SLM::MotionTransforms::Type_fr_base_X_fr_l1::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,1) = - sin__q_q1__;
    (*this)(2,0) =  sin__q_q1__;
    (*this)(2,1) =  cos__q_q1__;
    (*this)(3,3) =  cos__q_q1__;
    (*this)(3,4) = - sin__q_q1__;
    (*this)(5,3) =  sin__q_q1__;
    (*this)(5,4) =  cos__q_q1__;
    return *this;
}
iit::SLM::MotionTransforms::Type_fr_base_X_ee::Type_fr_base_X_ee()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1.0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = - 0.15;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,5) = 0;
}
const iit::SLM::MotionTransforms::Type_fr_base_X_ee& iit::SLM::MotionTransforms::Type_fr_base_X_ee::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,1) = - sin__q_q1__;
    (*this)(2,0) =  sin__q_q1__;
    (*this)(2,1) =  cos__q_q1__;
    (*this)(3,2) = ( 0.15 *  sin__q_q1__);
    (*this)(3,3) =  cos__q_q1__;
    (*this)(3,4) = - sin__q_q1__;
    (*this)(5,2) = (- 0.15 *  cos__q_q1__);
    (*this)(5,3) =  sin__q_q1__;
    (*this)(5,4) =  cos__q_q1__;
    return *this;
}
iit::SLM::MotionTransforms::Type_fr_base_X_fr_q1::Type_fr_base_X_fr_q1()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
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
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
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
    (*this)(5,3) = 0;
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
const iit::SLM::MotionTransforms::Type_fr_base_X_fr_q1& iit::SLM::MotionTransforms::Type_fr_base_X_fr_q1::update(const state_t& q) {
    return *this;
}
iit::SLM::MotionTransforms::Type_fr_l1_X_fr_base::Type_fr_l1_X_fr_base()
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
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
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
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
const iit::SLM::MotionTransforms::Type_fr_l1_X_fr_base& iit::SLM::MotionTransforms::Type_fr_l1_X_fr_base::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,2) =  sin__q_q1__;
    (*this)(1,0) = - sin__q_q1__;
    (*this)(1,2) =  cos__q_q1__;
    (*this)(3,3) =  cos__q_q1__;
    (*this)(3,5) =  sin__q_q1__;
    (*this)(4,3) = - sin__q_q1__;
    (*this)(4,5) =  cos__q_q1__;
    return *this;
}

iit::SLM::ForceTransforms::Type_fr_base_X_fr_l1::Type_fr_base_X_fr_l1()
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
const iit::SLM::ForceTransforms::Type_fr_base_X_fr_l1& iit::SLM::ForceTransforms::Type_fr_base_X_fr_l1::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,1) = - sin__q_q1__;
    (*this)(2,0) =  sin__q_q1__;
    (*this)(2,1) =  cos__q_q1__;
    (*this)(3,3) =  cos__q_q1__;
    (*this)(3,4) = - sin__q_q1__;
    (*this)(5,3) =  sin__q_q1__;
    (*this)(5,4) =  cos__q_q1__;
    return *this;
}
iit::SLM::ForceTransforms::Type_fr_base_X_ee::Type_fr_base_X_ee()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = - 0.15;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
const iit::SLM::ForceTransforms::Type_fr_base_X_ee& iit::SLM::ForceTransforms::Type_fr_base_X_ee::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,1) = - sin__q_q1__;
    (*this)(0,5) = ( 0.15 *  sin__q_q1__);
    (*this)(2,0) =  sin__q_q1__;
    (*this)(2,1) =  cos__q_q1__;
    (*this)(2,5) = (- 0.15 *  cos__q_q1__);
    (*this)(3,3) =  cos__q_q1__;
    (*this)(3,4) = - sin__q_q1__;
    (*this)(5,3) =  sin__q_q1__;
    (*this)(5,4) =  cos__q_q1__;
    return *this;
}
iit::SLM::ForceTransforms::Type_fr_base_X_fr_q1::Type_fr_base_X_fr_q1()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
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
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
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
    (*this)(5,3) = 0;
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
const iit::SLM::ForceTransforms::Type_fr_base_X_fr_q1& iit::SLM::ForceTransforms::Type_fr_base_X_fr_q1::update(const state_t& q) {
    return *this;
}
iit::SLM::ForceTransforms::Type_fr_l1_X_fr_base::Type_fr_l1_X_fr_base()
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
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
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
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
const iit::SLM::ForceTransforms::Type_fr_l1_X_fr_base& iit::SLM::ForceTransforms::Type_fr_l1_X_fr_base::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,2) =  sin__q_q1__;
    (*this)(1,0) = - sin__q_q1__;
    (*this)(1,2) =  cos__q_q1__;
    (*this)(3,3) =  cos__q_q1__;
    (*this)(3,5) =  sin__q_q1__;
    (*this)(4,3) = - sin__q_q1__;
    (*this)(4,5) =  cos__q_q1__;
    return *this;
}

iit::SLM::HomogeneousTransforms::Type_fr_base_X_fr_l1::Type_fr_base_X_fr_l1()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
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
const iit::SLM::HomogeneousTransforms::Type_fr_base_X_fr_l1& iit::SLM::HomogeneousTransforms::Type_fr_base_X_fr_l1::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,1) = - sin__q_q1__;
    (*this)(2,0) =  sin__q_q1__;
    (*this)(2,1) =  cos__q_q1__;
    return *this;
}
iit::SLM::HomogeneousTransforms::Type_fr_base_X_ee::Type_fr_base_X_ee()
{
    (*this)(0,2) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::SLM::HomogeneousTransforms::Type_fr_base_X_ee& iit::SLM::HomogeneousTransforms::Type_fr_base_X_ee::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,1) = - sin__q_q1__;
    (*this)(0,3) = ( 0.15 *  cos__q_q1__);
    (*this)(2,0) =  sin__q_q1__;
    (*this)(2,1) =  cos__q_q1__;
    (*this)(2,3) = ( 0.15 *  sin__q_q1__);
    return *this;
}
iit::SLM::HomogeneousTransforms::Type_fr_base_X_fr_q1::Type_fr_base_X_fr_q1()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::SLM::HomogeneousTransforms::Type_fr_base_X_fr_q1& iit::SLM::HomogeneousTransforms::Type_fr_base_X_fr_q1::update(const state_t& q) {
    return *this;
}
iit::SLM::HomogeneousTransforms::Type_fr_l1_X_fr_base::Type_fr_l1_X_fr_base()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::SLM::HomogeneousTransforms::Type_fr_l1_X_fr_base& iit::SLM::HomogeneousTransforms::Type_fr_l1_X_fr_base::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,2) =  sin__q_q1__;
    (*this)(1,0) = - sin__q_q1__;
    (*this)(1,2) =  cos__q_q1__;
    return *this;
}


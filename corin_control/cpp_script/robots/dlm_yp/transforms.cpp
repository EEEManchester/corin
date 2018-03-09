#include "transforms.h"

// Constructors

iit::DLM_YP::MotionTransforms::MotionTransforms
    ()
     :
    fr_base_X_fr_l2(),
    fr_base_X_ee(),
    fr_base_X_fr_q1(),
    fr_base_X_fr_q2(),
    fr_l1_X_fr_base(),
    fr_base_X_fr_l1(),
    fr_l2_X_fr_l1(),
    fr_l1_X_fr_l2()
{
    updateParameters();
}
void iit::DLM_YP::MotionTransforms::updateParameters() {
}

iit::DLM_YP::ForceTransforms::ForceTransforms
    ()
     :
    fr_base_X_fr_l2(),
    fr_base_X_ee(),
    fr_base_X_fr_q1(),
    fr_base_X_fr_q2(),
    fr_l1_X_fr_base(),
    fr_base_X_fr_l1(),
    fr_l2_X_fr_l1(),
    fr_l1_X_fr_l2()
{
    updateParameters();
}
void iit::DLM_YP::ForceTransforms::updateParameters() {
}

iit::DLM_YP::HomogeneousTransforms::HomogeneousTransforms
    ()
     :
    fr_base_X_fr_l2(),
    fr_base_X_ee(),
    fr_base_X_fr_q1(),
    fr_base_X_fr_q2(),
    fr_l1_X_fr_base(),
    fr_base_X_fr_l1(),
    fr_l2_X_fr_l1(),
    fr_l1_X_fr_l2()
{
    updateParameters();
}
void iit::DLM_YP::HomogeneousTransforms::updateParameters() {
}

iit::DLM_YP::MotionTransforms::Type_fr_base_X_fr_l2::Type_fr_base_X_fr_l2()
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
    (*this)(3,2) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,5) = 0;
}
const iit::DLM_YP::MotionTransforms::Type_fr_base_X_fr_l2& iit::DLM_YP::MotionTransforms::Type_fr_base_X_fr_l2::update(const state_t& q) {
    static double sin__q_q2__;
    static double sin__q_q1__;
    static double cos__q_q1__;
    static double cos__q_q2__;
    
    sin__q_q2__ = std::sin( q(Q2));
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    cos__q_q2__ = std::cos( q(Q2));
    
    (*this)(0,0) = ( cos__q_q1__ *  cos__q_q2__);
    (*this)(0,1) = (- cos__q_q1__ *  sin__q_q2__);
    (*this)(0,2) =  sin__q_q1__;
    (*this)(1,0) = ( sin__q_q1__ *  cos__q_q2__);
    (*this)(1,1) = (- sin__q_q1__ *  sin__q_q2__);
    (*this)(1,2) = - cos__q_q1__;
    (*this)(2,0) =  sin__q_q2__;
    (*this)(2,1) =  cos__q_q2__;
    (*this)(3,0) = (( 0.077 *  sin__q_q1__) *  sin__q_q2__);
    (*this)(3,1) = (( 0.077 *  sin__q_q1__) *  cos__q_q2__);
    (*this)(3,3) = ( cos__q_q1__ *  cos__q_q2__);
    (*this)(3,4) = (- cos__q_q1__ *  sin__q_q2__);
    (*this)(3,5) =  sin__q_q1__;
    (*this)(4,0) = ((- 0.077 *  cos__q_q1__) *  sin__q_q2__);
    (*this)(4,1) = ((- 0.077 *  cos__q_q1__) *  cos__q_q2__);
    (*this)(4,3) = ( sin__q_q1__ *  cos__q_q2__);
    (*this)(4,4) = (- sin__q_q1__ *  sin__q_q2__);
    (*this)(4,5) = - cos__q_q1__;
    (*this)(5,3) =  sin__q_q2__;
    (*this)(5,4) =  cos__q_q2__;
    return *this;
}
iit::DLM_YP::MotionTransforms::Type_fr_base_X_ee::Type_fr_base_X_ee()
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
const iit::DLM_YP::MotionTransforms::Type_fr_base_X_ee& iit::DLM_YP::MotionTransforms::Type_fr_base_X_ee::update(const state_t& q) {
    static double sin__q_q2__;
    static double sin__q_q1__;
    static double cos__q_q1__;
    static double cos__q_q2__;
    
    sin__q_q2__ = std::sin( q(Q2));
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    cos__q_q2__ = std::cos( q(Q2));
    
    (*this)(0,0) = ( cos__q_q1__ *  cos__q_q2__);
    (*this)(0,1) = (- cos__q_q1__ *  sin__q_q2__);
    (*this)(0,2) =  sin__q_q1__;
    (*this)(1,0) = ( sin__q_q1__ *  cos__q_q2__);
    (*this)(1,1) = (- sin__q_q1__ *  sin__q_q2__);
    (*this)(1,2) = - cos__q_q1__;
    (*this)(2,0) =  sin__q_q2__;
    (*this)(2,1) =  cos__q_q2__;
    (*this)(3,0) = (( 0.077 *  sin__q_q1__) *  sin__q_q2__);
    (*this)(3,1) = ((( 0.077 *  sin__q_q1__) *  cos__q_q2__) + ( 0.15 *  sin__q_q1__));
    (*this)(3,2) = (( 0.15 *  cos__q_q1__) *  sin__q_q2__);
    (*this)(3,3) = ( cos__q_q1__ *  cos__q_q2__);
    (*this)(3,4) = (- cos__q_q1__ *  sin__q_q2__);
    (*this)(3,5) =  sin__q_q1__;
    (*this)(4,0) = ((- 0.077 *  cos__q_q1__) *  sin__q_q2__);
    (*this)(4,1) = (((- 0.077 *  cos__q_q1__) *  cos__q_q2__) - ( 0.15 *  cos__q_q1__));
    (*this)(4,2) = (( 0.15 *  sin__q_q1__) *  sin__q_q2__);
    (*this)(4,3) = ( sin__q_q1__ *  cos__q_q2__);
    (*this)(4,4) = (- sin__q_q1__ *  sin__q_q2__);
    (*this)(4,5) = - cos__q_q1__;
    (*this)(5,2) = ((- 0.15 *  cos__q_q2__) -  0.077);
    (*this)(5,3) =  sin__q_q2__;
    (*this)(5,4) =  cos__q_q2__;
    return *this;
}
iit::DLM_YP::MotionTransforms::Type_fr_base_X_fr_q1::Type_fr_base_X_fr_q1()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
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
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::DLM_YP::MotionTransforms::Type_fr_base_X_fr_q1& iit::DLM_YP::MotionTransforms::Type_fr_base_X_fr_q1::update(const state_t& q) {
    return *this;
}
iit::DLM_YP::MotionTransforms::Type_fr_base_X_fr_q2::Type_fr_base_X_fr_q2()
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
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = - 0.077;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::DLM_YP::MotionTransforms::Type_fr_base_X_fr_q2& iit::DLM_YP::MotionTransforms::Type_fr_base_X_fr_q2::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,2) =  sin__q_q1__;
    (*this)(1,0) =  sin__q_q1__;
    (*this)(1,2) = - cos__q_q1__;
    (*this)(3,1) = ( 0.077 *  sin__q_q1__);
    (*this)(3,3) =  cos__q_q1__;
    (*this)(3,5) =  sin__q_q1__;
    (*this)(4,1) = (- 0.077 *  cos__q_q1__);
    (*this)(4,3) =  sin__q_q1__;
    (*this)(4,5) = - cos__q_q1__;
    return *this;
}
iit::DLM_YP::MotionTransforms::Type_fr_l1_X_fr_base::Type_fr_l1_X_fr_base()
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
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::DLM_YP::MotionTransforms::Type_fr_l1_X_fr_base& iit::DLM_YP::MotionTransforms::Type_fr_l1_X_fr_base::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,1) =  sin__q_q1__;
    (*this)(1,0) = - sin__q_q1__;
    (*this)(1,1) =  cos__q_q1__;
    (*this)(3,3) =  cos__q_q1__;
    (*this)(3,4) =  sin__q_q1__;
    (*this)(4,3) = - sin__q_q1__;
    (*this)(4,4) =  cos__q_q1__;
    return *this;
}
iit::DLM_YP::MotionTransforms::Type_fr_base_X_fr_l1::Type_fr_base_X_fr_l1()
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
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::DLM_YP::MotionTransforms::Type_fr_base_X_fr_l1& iit::DLM_YP::MotionTransforms::Type_fr_base_X_fr_l1::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,1) = - sin__q_q1__;
    (*this)(1,0) =  sin__q_q1__;
    (*this)(1,1) =  cos__q_q1__;
    (*this)(3,3) =  cos__q_q1__;
    (*this)(3,4) = - sin__q_q1__;
    (*this)(4,3) =  sin__q_q1__;
    (*this)(4,4) =  cos__q_q1__;
    return *this;
}
iit::DLM_YP::MotionTransforms::Type_fr_l2_X_fr_l1::Type_fr_l2_X_fr_l1()
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
const iit::DLM_YP::MotionTransforms::Type_fr_l2_X_fr_l1& iit::DLM_YP::MotionTransforms::Type_fr_l2_X_fr_l1::update(const state_t& q) {
    static double sin__q_q2__;
    static double cos__q_q2__;
    
    sin__q_q2__ = std::sin( q(Q2));
    cos__q_q2__ = std::cos( q(Q2));
    
    (*this)(0,0) =  cos__q_q2__;
    (*this)(0,2) =  sin__q_q2__;
    (*this)(1,0) = - sin__q_q2__;
    (*this)(1,2) =  cos__q_q2__;
    (*this)(3,1) = (- 0.077 *  sin__q_q2__);
    (*this)(3,3) =  cos__q_q2__;
    (*this)(3,5) =  sin__q_q2__;
    (*this)(4,1) = (- 0.077 *  cos__q_q2__);
    (*this)(4,3) = - sin__q_q2__;
    (*this)(4,5) =  cos__q_q2__;
    return *this;
}
iit::DLM_YP::MotionTransforms::Type_fr_l1_X_fr_l2::Type_fr_l1_X_fr_l2()
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
const iit::DLM_YP::MotionTransforms::Type_fr_l1_X_fr_l2& iit::DLM_YP::MotionTransforms::Type_fr_l1_X_fr_l2::update(const state_t& q) {
    static double sin__q_q2__;
    static double cos__q_q2__;
    
    sin__q_q2__ = std::sin( q(Q2));
    cos__q_q2__ = std::cos( q(Q2));
    
    (*this)(0,0) =  cos__q_q2__;
    (*this)(0,1) = - sin__q_q2__;
    (*this)(2,0) =  sin__q_q2__;
    (*this)(2,1) =  cos__q_q2__;
    (*this)(3,3) =  cos__q_q2__;
    (*this)(3,4) = - sin__q_q2__;
    (*this)(4,0) = (- 0.077 *  sin__q_q2__);
    (*this)(4,1) = (- 0.077 *  cos__q_q2__);
    (*this)(5,3) =  sin__q_q2__;
    (*this)(5,4) =  cos__q_q2__;
    return *this;
}

iit::DLM_YP::ForceTransforms::Type_fr_base_X_fr_l2::Type_fr_base_X_fr_l2()
{
    (*this)(0,5) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = - 0.077;
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
const iit::DLM_YP::ForceTransforms::Type_fr_base_X_fr_l2& iit::DLM_YP::ForceTransforms::Type_fr_base_X_fr_l2::update(const state_t& q) {
    static double sin__q_q2__;
    static double sin__q_q1__;
    static double cos__q_q1__;
    static double cos__q_q2__;
    
    sin__q_q2__ = std::sin( q(Q2));
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    cos__q_q2__ = std::cos( q(Q2));
    
    (*this)(0,0) = ( cos__q_q1__ *  cos__q_q2__);
    (*this)(0,1) = (- cos__q_q1__ *  sin__q_q2__);
    (*this)(0,2) =  sin__q_q1__;
    (*this)(0,3) = (( 0.077 *  sin__q_q1__) *  sin__q_q2__);
    (*this)(0,4) = (( 0.077 *  sin__q_q1__) *  cos__q_q2__);
    (*this)(1,0) = ( sin__q_q1__ *  cos__q_q2__);
    (*this)(1,1) = (- sin__q_q1__ *  sin__q_q2__);
    (*this)(1,2) = - cos__q_q1__;
    (*this)(1,3) = ((- 0.077 *  cos__q_q1__) *  sin__q_q2__);
    (*this)(1,4) = ((- 0.077 *  cos__q_q1__) *  cos__q_q2__);
    (*this)(2,0) =  sin__q_q2__;
    (*this)(2,1) =  cos__q_q2__;
    (*this)(3,3) = ( cos__q_q1__ *  cos__q_q2__);
    (*this)(3,4) = (- cos__q_q1__ *  sin__q_q2__);
    (*this)(3,5) =  sin__q_q1__;
    (*this)(4,3) = ( sin__q_q1__ *  cos__q_q2__);
    (*this)(4,4) = (- sin__q_q1__ *  sin__q_q2__);
    (*this)(4,5) = - cos__q_q1__;
    (*this)(5,3) =  sin__q_q2__;
    (*this)(5,4) =  cos__q_q2__;
    return *this;
}
iit::DLM_YP::ForceTransforms::Type_fr_base_X_ee::Type_fr_base_X_ee()
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
const iit::DLM_YP::ForceTransforms::Type_fr_base_X_ee& iit::DLM_YP::ForceTransforms::Type_fr_base_X_ee::update(const state_t& q) {
    static double sin__q_q2__;
    static double sin__q_q1__;
    static double cos__q_q1__;
    static double cos__q_q2__;
    
    sin__q_q2__ = std::sin( q(Q2));
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    cos__q_q2__ = std::cos( q(Q2));
    
    (*this)(0,0) = ( cos__q_q1__ *  cos__q_q2__);
    (*this)(0,1) = (- cos__q_q1__ *  sin__q_q2__);
    (*this)(0,2) =  sin__q_q1__;
    (*this)(0,3) = (( 0.077 *  sin__q_q1__) *  sin__q_q2__);
    (*this)(0,4) = ((( 0.077 *  sin__q_q1__) *  cos__q_q2__) + ( 0.15 *  sin__q_q1__));
    (*this)(0,5) = (( 0.15 *  cos__q_q1__) *  sin__q_q2__);
    (*this)(1,0) = ( sin__q_q1__ *  cos__q_q2__);
    (*this)(1,1) = (- sin__q_q1__ *  sin__q_q2__);
    (*this)(1,2) = - cos__q_q1__;
    (*this)(1,3) = ((- 0.077 *  cos__q_q1__) *  sin__q_q2__);
    (*this)(1,4) = (((- 0.077 *  cos__q_q1__) *  cos__q_q2__) - ( 0.15 *  cos__q_q1__));
    (*this)(1,5) = (( 0.15 *  sin__q_q1__) *  sin__q_q2__);
    (*this)(2,0) =  sin__q_q2__;
    (*this)(2,1) =  cos__q_q2__;
    (*this)(2,5) = ((- 0.15 *  cos__q_q2__) -  0.077);
    (*this)(3,3) = ( cos__q_q1__ *  cos__q_q2__);
    (*this)(3,4) = (- cos__q_q1__ *  sin__q_q2__);
    (*this)(3,5) =  sin__q_q1__;
    (*this)(4,3) = ( sin__q_q1__ *  cos__q_q2__);
    (*this)(4,4) = (- sin__q_q1__ *  sin__q_q2__);
    (*this)(4,5) = - cos__q_q1__;
    (*this)(5,3) =  sin__q_q2__;
    (*this)(5,4) =  cos__q_q2__;
    return *this;
}
iit::DLM_YP::ForceTransforms::Type_fr_base_X_fr_q1::Type_fr_base_X_fr_q1()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
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
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
const iit::DLM_YP::ForceTransforms::Type_fr_base_X_fr_q1& iit::DLM_YP::ForceTransforms::Type_fr_base_X_fr_q1::update(const state_t& q) {
    return *this;
}
iit::DLM_YP::ForceTransforms::Type_fr_base_X_fr_q2::Type_fr_base_X_fr_q2()
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
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
const iit::DLM_YP::ForceTransforms::Type_fr_base_X_fr_q2& iit::DLM_YP::ForceTransforms::Type_fr_base_X_fr_q2::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,2) =  sin__q_q1__;
    (*this)(0,4) = ( 0.077 *  sin__q_q1__);
    (*this)(1,0) =  sin__q_q1__;
    (*this)(1,2) = - cos__q_q1__;
    (*this)(1,4) = (- 0.077 *  cos__q_q1__);
    (*this)(3,3) =  cos__q_q1__;
    (*this)(3,5) =  sin__q_q1__;
    (*this)(4,3) =  sin__q_q1__;
    (*this)(4,5) = - cos__q_q1__;
    return *this;
}
iit::DLM_YP::ForceTransforms::Type_fr_l1_X_fr_base::Type_fr_l1_X_fr_base()
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
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::DLM_YP::ForceTransforms::Type_fr_l1_X_fr_base& iit::DLM_YP::ForceTransforms::Type_fr_l1_X_fr_base::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,1) =  sin__q_q1__;
    (*this)(1,0) = - sin__q_q1__;
    (*this)(1,1) =  cos__q_q1__;
    (*this)(3,3) =  cos__q_q1__;
    (*this)(3,4) =  sin__q_q1__;
    (*this)(4,3) = - sin__q_q1__;
    (*this)(4,4) =  cos__q_q1__;
    return *this;
}
iit::DLM_YP::ForceTransforms::Type_fr_base_X_fr_l1::Type_fr_base_X_fr_l1()
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
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
const iit::DLM_YP::ForceTransforms::Type_fr_base_X_fr_l1& iit::DLM_YP::ForceTransforms::Type_fr_base_X_fr_l1::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,1) = - sin__q_q1__;
    (*this)(1,0) =  sin__q_q1__;
    (*this)(1,1) =  cos__q_q1__;
    (*this)(3,3) =  cos__q_q1__;
    (*this)(3,4) = - sin__q_q1__;
    (*this)(4,3) =  sin__q_q1__;
    (*this)(4,4) =  cos__q_q1__;
    return *this;
}
iit::DLM_YP::ForceTransforms::Type_fr_l2_X_fr_l1::Type_fr_l2_X_fr_l1()
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
const iit::DLM_YP::ForceTransforms::Type_fr_l2_X_fr_l1& iit::DLM_YP::ForceTransforms::Type_fr_l2_X_fr_l1::update(const state_t& q) {
    static double sin__q_q2__;
    static double cos__q_q2__;
    
    sin__q_q2__ = std::sin( q(Q2));
    cos__q_q2__ = std::cos( q(Q2));
    
    (*this)(0,0) =  cos__q_q2__;
    (*this)(0,2) =  sin__q_q2__;
    (*this)(0,4) = (- 0.077 *  sin__q_q2__);
    (*this)(1,0) = - sin__q_q2__;
    (*this)(1,2) =  cos__q_q2__;
    (*this)(1,4) = (- 0.077 *  cos__q_q2__);
    (*this)(3,3) =  cos__q_q2__;
    (*this)(3,5) =  sin__q_q2__;
    (*this)(4,3) = - sin__q_q2__;
    (*this)(4,5) =  cos__q_q2__;
    return *this;
}
iit::DLM_YP::ForceTransforms::Type_fr_l1_X_fr_l2::Type_fr_l1_X_fr_l2()
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
const iit::DLM_YP::ForceTransforms::Type_fr_l1_X_fr_l2& iit::DLM_YP::ForceTransforms::Type_fr_l1_X_fr_l2::update(const state_t& q) {
    static double sin__q_q2__;
    static double cos__q_q2__;
    
    sin__q_q2__ = std::sin( q(Q2));
    cos__q_q2__ = std::cos( q(Q2));
    
    (*this)(0,0) =  cos__q_q2__;
    (*this)(0,1) = - sin__q_q2__;
    (*this)(1,3) = (- 0.077 *  sin__q_q2__);
    (*this)(1,4) = (- 0.077 *  cos__q_q2__);
    (*this)(2,0) =  sin__q_q2__;
    (*this)(2,1) =  cos__q_q2__;
    (*this)(3,3) =  cos__q_q2__;
    (*this)(3,4) = - sin__q_q2__;
    (*this)(5,3) =  sin__q_q2__;
    (*this)(5,4) =  cos__q_q2__;
    return *this;
}

iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_fr_l2::Type_fr_base_X_fr_l2()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_fr_l2& iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_fr_l2::update(const state_t& q) {
    static double sin__q_q2__;
    static double sin__q_q1__;
    static double cos__q_q1__;
    static double cos__q_q2__;
    
    sin__q_q2__ = std::sin( q(Q2));
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    cos__q_q2__ = std::cos( q(Q2));
    
    (*this)(0,0) = ( cos__q_q1__ *  cos__q_q2__);
    (*this)(0,1) = (- cos__q_q1__ *  sin__q_q2__);
    (*this)(0,2) =  sin__q_q1__;
    (*this)(0,3) = ( 0.077 *  cos__q_q1__);
    (*this)(1,0) = ( sin__q_q1__ *  cos__q_q2__);
    (*this)(1,1) = (- sin__q_q1__ *  sin__q_q2__);
    (*this)(1,2) = - cos__q_q1__;
    (*this)(1,3) = ( 0.077 *  sin__q_q1__);
    (*this)(2,0) =  sin__q_q2__;
    (*this)(2,1) =  cos__q_q2__;
    return *this;
}
iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_ee::Type_fr_base_X_ee()
{
    (*this)(2,2) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
const iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_ee& iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_ee::update(const state_t& q) {
    static double sin__q_q2__;
    static double sin__q_q1__;
    static double cos__q_q1__;
    static double cos__q_q2__;
    
    sin__q_q2__ = std::sin( q(Q2));
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    cos__q_q2__ = std::cos( q(Q2));
    
    (*this)(0,0) = ( cos__q_q1__ *  cos__q_q2__);
    (*this)(0,1) = (- cos__q_q1__ *  sin__q_q2__);
    (*this)(0,2) =  sin__q_q1__;
    (*this)(0,3) = ((( 0.15 *  cos__q_q1__) *  cos__q_q2__) + ( 0.077 *  cos__q_q1__));
    (*this)(1,0) = ( sin__q_q1__ *  cos__q_q2__);
    (*this)(1,1) = (- sin__q_q1__ *  sin__q_q2__);
    (*this)(1,2) = - cos__q_q1__;
    (*this)(1,3) = ((( 0.15 *  sin__q_q1__) *  cos__q_q2__) + ( 0.077 *  sin__q_q1__));
    (*this)(2,0) =  sin__q_q2__;
    (*this)(2,1) =  cos__q_q2__;
    (*this)(2,3) = ( 0.15 *  sin__q_q2__);
    return *this;
}
iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_fr_q1::Type_fr_base_X_fr_q1()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
const iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_fr_q1& iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_fr_q1::update(const state_t& q) {
    return *this;
}
iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_fr_q2::Type_fr_base_X_fr_q2()
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
const iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_fr_q2& iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_fr_q2::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,2) =  sin__q_q1__;
    (*this)(0,3) = ( 0.077 *  cos__q_q1__);
    (*this)(1,0) =  sin__q_q1__;
    (*this)(1,2) = - cos__q_q1__;
    (*this)(1,3) = ( 0.077 *  sin__q_q1__);
    return *this;
}
iit::DLM_YP::HomogeneousTransforms::Type_fr_l1_X_fr_base::Type_fr_l1_X_fr_base()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
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
const iit::DLM_YP::HomogeneousTransforms::Type_fr_l1_X_fr_base& iit::DLM_YP::HomogeneousTransforms::Type_fr_l1_X_fr_base::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,1) =  sin__q_q1__;
    (*this)(1,0) = - sin__q_q1__;
    (*this)(1,1) =  cos__q_q1__;
    return *this;
}
iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_fr_l1::Type_fr_base_X_fr_l1()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
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
const iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_fr_l1& iit::DLM_YP::HomogeneousTransforms::Type_fr_base_X_fr_l1::update(const state_t& q) {
    static double sin__q_q1__;
    static double cos__q_q1__;
    
    sin__q_q1__ = std::sin( q(Q1));
    cos__q_q1__ = std::cos( q(Q1));
    
    (*this)(0,0) =  cos__q_q1__;
    (*this)(0,1) = - sin__q_q1__;
    (*this)(1,0) =  sin__q_q1__;
    (*this)(1,1) =  cos__q_q1__;
    return *this;
}
iit::DLM_YP::HomogeneousTransforms::Type_fr_l2_X_fr_l1::Type_fr_l2_X_fr_l1()
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
const iit::DLM_YP::HomogeneousTransforms::Type_fr_l2_X_fr_l1& iit::DLM_YP::HomogeneousTransforms::Type_fr_l2_X_fr_l1::update(const state_t& q) {
    static double sin__q_q2__;
    static double cos__q_q2__;
    
    sin__q_q2__ = std::sin( q(Q2));
    cos__q_q2__ = std::cos( q(Q2));
    
    (*this)(0,0) =  cos__q_q2__;
    (*this)(0,2) =  sin__q_q2__;
    (*this)(0,3) = (- 0.077 *  cos__q_q2__);
    (*this)(1,0) = - sin__q_q2__;
    (*this)(1,2) =  cos__q_q2__;
    (*this)(1,3) = ( 0.077 *  sin__q_q2__);
    return *this;
}
iit::DLM_YP::HomogeneousTransforms::Type_fr_l1_X_fr_l2::Type_fr_l1_X_fr_l2()
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
const iit::DLM_YP::HomogeneousTransforms::Type_fr_l1_X_fr_l2& iit::DLM_YP::HomogeneousTransforms::Type_fr_l1_X_fr_l2::update(const state_t& q) {
    static double sin__q_q2__;
    static double cos__q_q2__;
    
    sin__q_q2__ = std::sin( q(Q2));
    cos__q_q2__ = std::cos( q(Q2));
    
    (*this)(0,0) =  cos__q_q2__;
    (*this)(0,1) = - sin__q_q2__;
    (*this)(2,0) =  sin__q_q2__;
    (*this)(2,1) =  cos__q_q2__;
    return *this;
}


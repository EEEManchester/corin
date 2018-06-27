#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
iit::mcorin::dyn::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    LF_lowerleg_Ic(linkInertias.getTensor_LF_lowerleg()),
    LM_lowerleg_Ic(linkInertias.getTensor_LM_lowerleg()),
    LR_lowerleg_Ic(linkInertias.getTensor_LR_lowerleg()),
    RF_lowerleg_Ic(linkInertias.getTensor_RF_lowerleg()),
    RM_lowerleg_Ic(linkInertias.getTensor_RM_lowerleg()),
    RR_lowerleg_Ic(linkInertias.getTensor_RR_lowerleg())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
#define Fcol(j) (block<6,1>(0,(j)+6))
#define F(i,j) DATA((i),(j)+6)
const iit::mcorin::dyn::JSIM& iit::mcorin::dyn::JSIM::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_RR_upperleg_X_fr_RR_lowerleg(state);
    frcTransf -> fr_RM_upperleg_X_fr_RM_lowerleg(state);
    frcTransf -> fr_LR_upperleg_X_fr_LR_lowerleg(state);
    frcTransf -> fr_LM_upperleg_X_fr_LM_lowerleg(state);
    frcTransf -> fr_RR_hipassembly_X_fr_RR_upperleg(state);
    frcTransf -> fr_RM_hipassembly_X_fr_RM_upperleg(state);
    frcTransf -> fr_LR_hipassembly_X_fr_LR_upperleg(state);
    frcTransf -> fr_LM_hipassembly_X_fr_LM_upperleg(state);
    frcTransf -> fr_trunk_X_fr_RR_hipassembly(state);
    frcTransf -> fr_trunk_X_fr_RM_hipassembly(state);
    frcTransf -> fr_trunk_X_fr_LR_hipassembly(state);
    frcTransf -> fr_trunk_X_fr_LM_hipassembly(state);
    frcTransf -> fr_RF_upperleg_X_fr_RF_lowerleg(state);
    frcTransf -> fr_LF_upperleg_X_fr_LF_lowerleg(state);
    frcTransf -> fr_RF_hipassembly_X_fr_RF_upperleg(state);
    frcTransf -> fr_LF_hipassembly_X_fr_LF_upperleg(state);
    frcTransf -> fr_trunk_X_fr_RF_hipassembly(state);
    frcTransf -> fr_trunk_X_fr_LF_hipassembly(state);

    // Initializes the composite inertia tensors
    trunk_Ic = linkInertias.getTensor_trunk();
    LF_hipassembly_Ic = linkInertias.getTensor_LF_hipassembly();
    LF_upperleg_Ic = linkInertias.getTensor_LF_upperleg();
    LM_hipassembly_Ic = linkInertias.getTensor_LM_hipassembly();
    LM_upperleg_Ic = linkInertias.getTensor_LM_upperleg();
    LR_hipassembly_Ic = linkInertias.getTensor_LR_hipassembly();
    LR_upperleg_Ic = linkInertias.getTensor_LR_upperleg();
    RF_hipassembly_Ic = linkInertias.getTensor_RF_hipassembly();
    RF_upperleg_Ic = linkInertias.getTensor_RF_upperleg();
    RM_hipassembly_Ic = linkInertias.getTensor_RM_hipassembly();
    RM_upperleg_Ic = linkInertias.getTensor_RM_upperleg();
    RR_hipassembly_Ic = linkInertias.getTensor_RR_hipassembly();
    RR_upperleg_Ic = linkInertias.getTensor_RR_upperleg();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link RR_lowerleg:
    iit::rbd::transformInertia(RR_lowerleg_Ic, frcTransf -> fr_RR_upperleg_X_fr_RR_lowerleg, Ic_spare);
    RR_upperleg_Ic += Ic_spare;

    Fcol(RR_Q3_JOINT) = RR_lowerleg_Ic.col(AZ);
    DATA(RR_Q3_JOINT+6, RR_Q3_JOINT+6) = Fcol(RR_Q3_JOINT)(AZ);

    Fcol(RR_Q3_JOINT) = frcTransf -> fr_RR_upperleg_X_fr_RR_lowerleg * Fcol(RR_Q3_JOINT);
    DATA(RR_Q3_JOINT+6, RR_Q2_JOINT+6) = F(AZ,RR_Q3_JOINT);
    DATA(RR_Q2_JOINT+6, RR_Q3_JOINT+6) = DATA(RR_Q3_JOINT+6, RR_Q2_JOINT+6);
    Fcol(RR_Q3_JOINT) = frcTransf -> fr_RR_hipassembly_X_fr_RR_upperleg * Fcol(RR_Q3_JOINT);
    DATA(RR_Q3_JOINT+6, RR_Q1_JOINT+6) = F(AZ,RR_Q3_JOINT);
    DATA(RR_Q1_JOINT+6, RR_Q3_JOINT+6) = DATA(RR_Q3_JOINT+6, RR_Q1_JOINT+6);
    Fcol(RR_Q3_JOINT) = frcTransf -> fr_trunk_X_fr_RR_hipassembly * Fcol(RR_Q3_JOINT);

    // Link RM_lowerleg:
    iit::rbd::transformInertia(RM_lowerleg_Ic, frcTransf -> fr_RM_upperleg_X_fr_RM_lowerleg, Ic_spare);
    RM_upperleg_Ic += Ic_spare;

    Fcol(RM_Q3_JOINT) = RM_lowerleg_Ic.col(AZ);
    DATA(RM_Q3_JOINT+6, RM_Q3_JOINT+6) = Fcol(RM_Q3_JOINT)(AZ);

    Fcol(RM_Q3_JOINT) = frcTransf -> fr_RM_upperleg_X_fr_RM_lowerleg * Fcol(RM_Q3_JOINT);
    DATA(RM_Q3_JOINT+6, RM_Q2_JOINT+6) = F(AZ,RM_Q3_JOINT);
    DATA(RM_Q2_JOINT+6, RM_Q3_JOINT+6) = DATA(RM_Q3_JOINT+6, RM_Q2_JOINT+6);
    Fcol(RM_Q3_JOINT) = frcTransf -> fr_RM_hipassembly_X_fr_RM_upperleg * Fcol(RM_Q3_JOINT);
    DATA(RM_Q3_JOINT+6, RM_Q1_JOINT+6) = F(AZ,RM_Q3_JOINT);
    DATA(RM_Q1_JOINT+6, RM_Q3_JOINT+6) = DATA(RM_Q3_JOINT+6, RM_Q1_JOINT+6);
    Fcol(RM_Q3_JOINT) = frcTransf -> fr_trunk_X_fr_RM_hipassembly * Fcol(RM_Q3_JOINT);

    // Link LR_lowerleg:
    iit::rbd::transformInertia(LR_lowerleg_Ic, frcTransf -> fr_LR_upperleg_X_fr_LR_lowerleg, Ic_spare);
    LR_upperleg_Ic += Ic_spare;

    Fcol(LR_Q3_JOINT) = LR_lowerleg_Ic.col(AZ);
    DATA(LR_Q3_JOINT+6, LR_Q3_JOINT+6) = Fcol(LR_Q3_JOINT)(AZ);

    Fcol(LR_Q3_JOINT) = frcTransf -> fr_LR_upperleg_X_fr_LR_lowerleg * Fcol(LR_Q3_JOINT);
    DATA(LR_Q3_JOINT+6, LR_Q2_JOINT+6) = F(AZ,LR_Q3_JOINT);
    DATA(LR_Q2_JOINT+6, LR_Q3_JOINT+6) = DATA(LR_Q3_JOINT+6, LR_Q2_JOINT+6);
    Fcol(LR_Q3_JOINT) = frcTransf -> fr_LR_hipassembly_X_fr_LR_upperleg * Fcol(LR_Q3_JOINT);
    DATA(LR_Q3_JOINT+6, LR_Q1_JOINT+6) = F(AZ,LR_Q3_JOINT);
    DATA(LR_Q1_JOINT+6, LR_Q3_JOINT+6) = DATA(LR_Q3_JOINT+6, LR_Q1_JOINT+6);
    Fcol(LR_Q3_JOINT) = frcTransf -> fr_trunk_X_fr_LR_hipassembly * Fcol(LR_Q3_JOINT);

    // Link LM_lowerleg:
    iit::rbd::transformInertia(LM_lowerleg_Ic, frcTransf -> fr_LM_upperleg_X_fr_LM_lowerleg, Ic_spare);
    LM_upperleg_Ic += Ic_spare;

    Fcol(LM_Q3_JOINT) = LM_lowerleg_Ic.col(AZ);
    DATA(LM_Q3_JOINT+6, LM_Q3_JOINT+6) = Fcol(LM_Q3_JOINT)(AZ);

    Fcol(LM_Q3_JOINT) = frcTransf -> fr_LM_upperleg_X_fr_LM_lowerleg * Fcol(LM_Q3_JOINT);
    DATA(LM_Q3_JOINT+6, LM_Q2_JOINT+6) = F(AZ,LM_Q3_JOINT);
    DATA(LM_Q2_JOINT+6, LM_Q3_JOINT+6) = DATA(LM_Q3_JOINT+6, LM_Q2_JOINT+6);
    Fcol(LM_Q3_JOINT) = frcTransf -> fr_LM_hipassembly_X_fr_LM_upperleg * Fcol(LM_Q3_JOINT);
    DATA(LM_Q3_JOINT+6, LM_Q1_JOINT+6) = F(AZ,LM_Q3_JOINT);
    DATA(LM_Q1_JOINT+6, LM_Q3_JOINT+6) = DATA(LM_Q3_JOINT+6, LM_Q1_JOINT+6);
    Fcol(LM_Q3_JOINT) = frcTransf -> fr_trunk_X_fr_LM_hipassembly * Fcol(LM_Q3_JOINT);

    // Link RR_upperleg:
    iit::rbd::transformInertia(RR_upperleg_Ic, frcTransf -> fr_RR_hipassembly_X_fr_RR_upperleg, Ic_spare);
    RR_hipassembly_Ic += Ic_spare;

    Fcol(RR_Q2_JOINT) = RR_upperleg_Ic.col(AZ);
    DATA(RR_Q2_JOINT+6, RR_Q2_JOINT+6) = Fcol(RR_Q2_JOINT)(AZ);

    Fcol(RR_Q2_JOINT) = frcTransf -> fr_RR_hipassembly_X_fr_RR_upperleg * Fcol(RR_Q2_JOINT);
    DATA(RR_Q2_JOINT+6, RR_Q1_JOINT+6) = F(AZ,RR_Q2_JOINT);
    DATA(RR_Q1_JOINT+6, RR_Q2_JOINT+6) = DATA(RR_Q2_JOINT+6, RR_Q1_JOINT+6);
    Fcol(RR_Q2_JOINT) = frcTransf -> fr_trunk_X_fr_RR_hipassembly * Fcol(RR_Q2_JOINT);

    // Link RM_upperleg:
    iit::rbd::transformInertia(RM_upperleg_Ic, frcTransf -> fr_RM_hipassembly_X_fr_RM_upperleg, Ic_spare);
    RM_hipassembly_Ic += Ic_spare;

    Fcol(RM_Q2_JOINT) = RM_upperleg_Ic.col(AZ);
    DATA(RM_Q2_JOINT+6, RM_Q2_JOINT+6) = Fcol(RM_Q2_JOINT)(AZ);

    Fcol(RM_Q2_JOINT) = frcTransf -> fr_RM_hipassembly_X_fr_RM_upperleg * Fcol(RM_Q2_JOINT);
    DATA(RM_Q2_JOINT+6, RM_Q1_JOINT+6) = F(AZ,RM_Q2_JOINT);
    DATA(RM_Q1_JOINT+6, RM_Q2_JOINT+6) = DATA(RM_Q2_JOINT+6, RM_Q1_JOINT+6);
    Fcol(RM_Q2_JOINT) = frcTransf -> fr_trunk_X_fr_RM_hipassembly * Fcol(RM_Q2_JOINT);

    // Link LR_upperleg:
    iit::rbd::transformInertia(LR_upperleg_Ic, frcTransf -> fr_LR_hipassembly_X_fr_LR_upperleg, Ic_spare);
    LR_hipassembly_Ic += Ic_spare;

    Fcol(LR_Q2_JOINT) = LR_upperleg_Ic.col(AZ);
    DATA(LR_Q2_JOINT+6, LR_Q2_JOINT+6) = Fcol(LR_Q2_JOINT)(AZ);

    Fcol(LR_Q2_JOINT) = frcTransf -> fr_LR_hipassembly_X_fr_LR_upperleg * Fcol(LR_Q2_JOINT);
    DATA(LR_Q2_JOINT+6, LR_Q1_JOINT+6) = F(AZ,LR_Q2_JOINT);
    DATA(LR_Q1_JOINT+6, LR_Q2_JOINT+6) = DATA(LR_Q2_JOINT+6, LR_Q1_JOINT+6);
    Fcol(LR_Q2_JOINT) = frcTransf -> fr_trunk_X_fr_LR_hipassembly * Fcol(LR_Q2_JOINT);

    // Link LM_upperleg:
    iit::rbd::transformInertia(LM_upperleg_Ic, frcTransf -> fr_LM_hipassembly_X_fr_LM_upperleg, Ic_spare);
    LM_hipassembly_Ic += Ic_spare;

    Fcol(LM_Q2_JOINT) = LM_upperleg_Ic.col(AZ);
    DATA(LM_Q2_JOINT+6, LM_Q2_JOINT+6) = Fcol(LM_Q2_JOINT)(AZ);

    Fcol(LM_Q2_JOINT) = frcTransf -> fr_LM_hipassembly_X_fr_LM_upperleg * Fcol(LM_Q2_JOINT);
    DATA(LM_Q2_JOINT+6, LM_Q1_JOINT+6) = F(AZ,LM_Q2_JOINT);
    DATA(LM_Q1_JOINT+6, LM_Q2_JOINT+6) = DATA(LM_Q2_JOINT+6, LM_Q1_JOINT+6);
    Fcol(LM_Q2_JOINT) = frcTransf -> fr_trunk_X_fr_LM_hipassembly * Fcol(LM_Q2_JOINT);

    // Link RR_hipassembly:
    iit::rbd::transformInertia(RR_hipassembly_Ic, frcTransf -> fr_trunk_X_fr_RR_hipassembly, Ic_spare);
    trunk_Ic += Ic_spare;

    Fcol(RR_Q1_JOINT) = RR_hipassembly_Ic.col(AZ);
    DATA(RR_Q1_JOINT+6, RR_Q1_JOINT+6) = Fcol(RR_Q1_JOINT)(AZ);

    Fcol(RR_Q1_JOINT) = frcTransf -> fr_trunk_X_fr_RR_hipassembly * Fcol(RR_Q1_JOINT);

    // Link RM_hipassembly:
    iit::rbd::transformInertia(RM_hipassembly_Ic, frcTransf -> fr_trunk_X_fr_RM_hipassembly, Ic_spare);
    trunk_Ic += Ic_spare;

    Fcol(RM_Q1_JOINT) = RM_hipassembly_Ic.col(AZ);
    DATA(RM_Q1_JOINT+6, RM_Q1_JOINT+6) = Fcol(RM_Q1_JOINT)(AZ);

    Fcol(RM_Q1_JOINT) = frcTransf -> fr_trunk_X_fr_RM_hipassembly * Fcol(RM_Q1_JOINT);

    // Link LR_hipassembly:
    iit::rbd::transformInertia(LR_hipassembly_Ic, frcTransf -> fr_trunk_X_fr_LR_hipassembly, Ic_spare);
    trunk_Ic += Ic_spare;

    Fcol(LR_Q1_JOINT) = LR_hipassembly_Ic.col(AZ);
    DATA(LR_Q1_JOINT+6, LR_Q1_JOINT+6) = Fcol(LR_Q1_JOINT)(AZ);

    Fcol(LR_Q1_JOINT) = frcTransf -> fr_trunk_X_fr_LR_hipassembly * Fcol(LR_Q1_JOINT);

    // Link LM_hipassembly:
    iit::rbd::transformInertia(LM_hipassembly_Ic, frcTransf -> fr_trunk_X_fr_LM_hipassembly, Ic_spare);
    trunk_Ic += Ic_spare;

    Fcol(LM_Q1_JOINT) = LM_hipassembly_Ic.col(AZ);
    DATA(LM_Q1_JOINT+6, LM_Q1_JOINT+6) = Fcol(LM_Q1_JOINT)(AZ);

    Fcol(LM_Q1_JOINT) = frcTransf -> fr_trunk_X_fr_LM_hipassembly * Fcol(LM_Q1_JOINT);

    // Link RF_lowerleg:
    iit::rbd::transformInertia(RF_lowerleg_Ic, frcTransf -> fr_RF_upperleg_X_fr_RF_lowerleg, Ic_spare);
    RF_upperleg_Ic += Ic_spare;

    Fcol(RF_Q3_JOINT) = RF_lowerleg_Ic.col(AZ);
    DATA(RF_Q3_JOINT+6, RF_Q3_JOINT+6) = Fcol(RF_Q3_JOINT)(AZ);

    Fcol(RF_Q3_JOINT) = frcTransf -> fr_RF_upperleg_X_fr_RF_lowerleg * Fcol(RF_Q3_JOINT);
    DATA(RF_Q3_JOINT+6, RF_Q2_JOINT+6) = F(AZ,RF_Q3_JOINT);
    DATA(RF_Q2_JOINT+6, RF_Q3_JOINT+6) = DATA(RF_Q3_JOINT+6, RF_Q2_JOINT+6);
    Fcol(RF_Q3_JOINT) = frcTransf -> fr_RF_hipassembly_X_fr_RF_upperleg * Fcol(RF_Q3_JOINT);
    DATA(RF_Q3_JOINT+6, RF_Q1_JOINT+6) = F(AZ,RF_Q3_JOINT);
    DATA(RF_Q1_JOINT+6, RF_Q3_JOINT+6) = DATA(RF_Q3_JOINT+6, RF_Q1_JOINT+6);
    Fcol(RF_Q3_JOINT) = frcTransf -> fr_trunk_X_fr_RF_hipassembly * Fcol(RF_Q3_JOINT);

    // Link LF_lowerleg:
    iit::rbd::transformInertia(LF_lowerleg_Ic, frcTransf -> fr_LF_upperleg_X_fr_LF_lowerleg, Ic_spare);
    LF_upperleg_Ic += Ic_spare;

    Fcol(LF_Q3_JOINT) = LF_lowerleg_Ic.col(AZ);
    DATA(LF_Q3_JOINT+6, LF_Q3_JOINT+6) = Fcol(LF_Q3_JOINT)(AZ);

    Fcol(LF_Q3_JOINT) = frcTransf -> fr_LF_upperleg_X_fr_LF_lowerleg * Fcol(LF_Q3_JOINT);
    DATA(LF_Q3_JOINT+6, LF_Q2_JOINT+6) = F(AZ,LF_Q3_JOINT);
    DATA(LF_Q2_JOINT+6, LF_Q3_JOINT+6) = DATA(LF_Q3_JOINT+6, LF_Q2_JOINT+6);
    Fcol(LF_Q3_JOINT) = frcTransf -> fr_LF_hipassembly_X_fr_LF_upperleg * Fcol(LF_Q3_JOINT);
    DATA(LF_Q3_JOINT+6, LF_Q1_JOINT+6) = F(AZ,LF_Q3_JOINT);
    DATA(LF_Q1_JOINT+6, LF_Q3_JOINT+6) = DATA(LF_Q3_JOINT+6, LF_Q1_JOINT+6);
    Fcol(LF_Q3_JOINT) = frcTransf -> fr_trunk_X_fr_LF_hipassembly * Fcol(LF_Q3_JOINT);

    // Link RF_upperleg:
    iit::rbd::transformInertia(RF_upperleg_Ic, frcTransf -> fr_RF_hipassembly_X_fr_RF_upperleg, Ic_spare);
    RF_hipassembly_Ic += Ic_spare;

    Fcol(RF_Q2_JOINT) = RF_upperleg_Ic.col(AZ);
    DATA(RF_Q2_JOINT+6, RF_Q2_JOINT+6) = Fcol(RF_Q2_JOINT)(AZ);

    Fcol(RF_Q2_JOINT) = frcTransf -> fr_RF_hipassembly_X_fr_RF_upperleg * Fcol(RF_Q2_JOINT);
    DATA(RF_Q2_JOINT+6, RF_Q1_JOINT+6) = F(AZ,RF_Q2_JOINT);
    DATA(RF_Q1_JOINT+6, RF_Q2_JOINT+6) = DATA(RF_Q2_JOINT+6, RF_Q1_JOINT+6);
    Fcol(RF_Q2_JOINT) = frcTransf -> fr_trunk_X_fr_RF_hipassembly * Fcol(RF_Q2_JOINT);

    // Link LF_upperleg:
    iit::rbd::transformInertia(LF_upperleg_Ic, frcTransf -> fr_LF_hipassembly_X_fr_LF_upperleg, Ic_spare);
    LF_hipassembly_Ic += Ic_spare;

    Fcol(LF_Q2_JOINT) = LF_upperleg_Ic.col(AZ);
    DATA(LF_Q2_JOINT+6, LF_Q2_JOINT+6) = Fcol(LF_Q2_JOINT)(AZ);

    Fcol(LF_Q2_JOINT) = frcTransf -> fr_LF_hipassembly_X_fr_LF_upperleg * Fcol(LF_Q2_JOINT);
    DATA(LF_Q2_JOINT+6, LF_Q1_JOINT+6) = F(AZ,LF_Q2_JOINT);
    DATA(LF_Q1_JOINT+6, LF_Q2_JOINT+6) = DATA(LF_Q2_JOINT+6, LF_Q1_JOINT+6);
    Fcol(LF_Q2_JOINT) = frcTransf -> fr_trunk_X_fr_LF_hipassembly * Fcol(LF_Q2_JOINT);

    // Link RF_hipassembly:
    iit::rbd::transformInertia(RF_hipassembly_Ic, frcTransf -> fr_trunk_X_fr_RF_hipassembly, Ic_spare);
    trunk_Ic += Ic_spare;

    Fcol(RF_Q1_JOINT) = RF_hipassembly_Ic.col(AZ);
    DATA(RF_Q1_JOINT+6, RF_Q1_JOINT+6) = Fcol(RF_Q1_JOINT)(AZ);

    Fcol(RF_Q1_JOINT) = frcTransf -> fr_trunk_X_fr_RF_hipassembly * Fcol(RF_Q1_JOINT);

    // Link LF_hipassembly:
    iit::rbd::transformInertia(LF_hipassembly_Ic, frcTransf -> fr_trunk_X_fr_LF_hipassembly, Ic_spare);
    trunk_Ic += Ic_spare;

    Fcol(LF_Q1_JOINT) = LF_hipassembly_Ic.col(AZ);
    DATA(LF_Q1_JOINT+6, LF_Q1_JOINT+6) = Fcol(LF_Q1_JOINT)(AZ);

    Fcol(LF_Q1_JOINT) = frcTransf -> fr_trunk_X_fr_LF_hipassembly * Fcol(LF_Q1_JOINT);

    // Copies the upper-right block into the lower-left block, after transposing
    block<18, 6>(6,0) = (block<6, 18>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    block<6,6>(0,0) = trunk_Ic;
    return *this;
}

#undef DATA
#undef F

void iit::mcorin::dyn::JSIM::computeL() {
    L = this -> triangularView<Eigen::Lower>();
    // Joint RR_q3_joint, index 5 :
    L(5, 5) = std::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    
    // Joint RR_q2_joint, index 4 :
    L(4, 4) = std::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    
    // Joint RR_q1_joint, index 3 :
    L(3, 3) = std::sqrt(L(3, 3));
    
    // Joint RM_q3_joint, index 5 :
    L(5, 5) = std::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    
    // Joint RM_q2_joint, index 4 :
    L(4, 4) = std::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    
    // Joint RM_q1_joint, index 3 :
    L(3, 3) = std::sqrt(L(3, 3));
    
    // Joint RF_q3_joint, index 2 :
    L(2, 2) = std::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint RF_q2_joint, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint RF_q1_joint, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
    // Joint LR_q3_joint, index 5 :
    L(5, 5) = std::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    
    // Joint LR_q2_joint, index 4 :
    L(4, 4) = std::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    
    // Joint LR_q1_joint, index 3 :
    L(3, 3) = std::sqrt(L(3, 3));
    
    // Joint LM_q3_joint, index 5 :
    L(5, 5) = std::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    
    // Joint LM_q2_joint, index 4 :
    L(4, 4) = std::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    
    // Joint LM_q1_joint, index 3 :
    L(3, 3) = std::sqrt(L(3, 3));
    
    // Joint LF_q3_joint, index 2 :
    L(2, 2) = std::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint LF_q2_joint, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint LF_q1_joint, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
}

void iit::mcorin::dyn::JSIM::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =  + (Linv(3, 3) * Linv(3, 3));
    inverse(4, 4) =  + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(5, 5) =  + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(3, 3) =  + (Linv(3, 3) * Linv(3, 3));
    inverse(4, 4) =  + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(5, 5) =  + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =  + (Linv(3, 3) * Linv(3, 3));
    inverse(4, 4) =  + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(5, 5) =  + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(3, 3) =  + (Linv(3, 3) * Linv(3, 3));
    inverse(4, 4) =  + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(5, 5) =  + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
}

void iit::mcorin::dyn::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
}

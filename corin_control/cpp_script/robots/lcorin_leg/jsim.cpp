#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
iit::LCORIN_LEG::dyn::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    l3_Ic(linkInertias.getTensor_l3())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
const iit::LCORIN_LEG::dyn::JSIM& iit::LCORIN_LEG::dyn::JSIM::update(const JointState& state) {
    static iit::rbd::ForceVector F;

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_l2_X_fr_l3(state);
    frcTransf -> fr_l1_X_fr_l2(state);

    // Initializes the composite inertia tensors
    l1_Ic = linkInertias.getTensor_l1();
    l2_Ic = linkInertias.getTensor_l2();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link l3:
    iit::rbd::transformInertia(l3_Ic, frcTransf -> fr_l2_X_fr_l3, Ic_spare);
    l2_Ic += Ic_spare;

    F = l3_Ic.col(AZ);
    DATA(Q3, Q3) = F(AZ);

    F = frcTransf -> fr_l2_X_fr_l3 * F;
    DATA(Q3, Q2) = F(AZ);
    DATA(Q2, Q3) = DATA(Q3, Q2);
    F = frcTransf -> fr_l1_X_fr_l2 * F;
    DATA(Q3, Q1) = F(AZ);
    DATA(Q1, Q3) = DATA(Q3, Q1);

    // Link l2:
    iit::rbd::transformInertia(l2_Ic, frcTransf -> fr_l1_X_fr_l2, Ic_spare);
    l1_Ic += Ic_spare;

    F = l2_Ic.col(AZ);
    DATA(Q2, Q2) = F(AZ);

    F = frcTransf -> fr_l1_X_fr_l2 * F;
    DATA(Q2, Q1) = F(AZ);
    DATA(Q1, Q2) = DATA(Q2, Q1);

    // Link l1:

    F = l1_Ic.col(AZ);
    DATA(Q1, Q1) = F(AZ);


    return *this;
}

#undef DATA
#undef F

void iit::LCORIN_LEG::dyn::JSIM::computeL() {
    L = this -> triangularView<Eigen::Lower>();
    // Joint q3, index 3 :
    L(3, 3) = std::sqrt(L(3, 3));
    L(3, 2) = L(3, 2) / L(3, 3);
    L(3, 1) = L(3, 1) / L(3, 3);
    L(2, 2) = L(2, 2) - L(3, 2) * L(3, 2);
    L(2, 1) = L(2, 1) - L(3, 2) * L(3, 1);
    L(1, 1) = L(1, 1) - L(3, 1) * L(3, 1);
    
    // Joint q2, index 2 :
    L(2, 2) = std::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    
    // Joint q1, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    
}

void iit::LCORIN_LEG::dyn::JSIM::computeInverse() {
    computeLInverse();

    inverse(1, 1) =  + (Linv(1, 1) * Linv(1, 1));
    inverse(2, 2) =  + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(3, 3) =  + (Linv(3, 1) * Linv(3, 1)) + (Linv(3, 2) * Linv(3, 2)) + (Linv(3, 3) * Linv(3, 3));
    inverse(3, 2) =  + (Linv(3, 1) * Linv(2, 1)) + (Linv(3, 2) * Linv(2, 2));
    inverse(2, 3) = inverse(3, 2);
    inverse(3, 1) =  + (Linv(3, 1) * Linv(1, 1));
    inverse(1, 3) = inverse(3, 1);
}

void iit::LCORIN_LEG::dyn::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(3, 2) = - Linv(2, 2) * ((Linv(3, 3) * L(3, 2)) + 0);
    Linv(3, 1) = - Linv(1, 1) * ((Linv(3, 2) * L(2, 1)) + (Linv(3, 3) * L(3, 1)) + 0);
}

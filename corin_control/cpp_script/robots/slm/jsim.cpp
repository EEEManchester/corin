#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
iit::SLM::dyn::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    l1_Ic(linkInertias.getTensor_l1())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
const iit::SLM::dyn::JSIM& iit::SLM::dyn::JSIM::update(const JointState& state) {
    static iit::rbd::ForceVector F;

    // Precomputes only once the coordinate transforms:

    // Initializes the composite inertia tensors

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link l1:

    F = l1_Ic.col(AZ);
    DATA(Q1, Q1) = F(AZ);


    return *this;
}

#undef DATA
#undef F

void iit::SLM::dyn::JSIM::computeL() {
    L = this -> triangularView<Eigen::Lower>();
    // Joint q1, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    
}

void iit::SLM::dyn::JSIM::computeInverse() {
    computeLInverse();

    inverse(1, 1) =  + (Linv(1, 1) * Linv(1, 1));
}

void iit::SLM::dyn::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(1, 1) = 1 / L(1, 1);
}

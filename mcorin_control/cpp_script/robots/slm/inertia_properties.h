#ifndef IIT_ROBOT_SLM_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_SLM_INERTIA_PROPERTIES_H_

#include <Eigen/Dense>
#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"

namespace iit {
namespace SLM {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot SLM.
 */
namespace dyn {

typedef iit::rbd::InertiaMatrixDense InertiaMatrix;

class InertiaProperties {
    public:
        InertiaProperties();
        ~InertiaProperties();
        const InertiaMatrix& getTensor_l1() const;
        double getMass_l1() const;
        const iit::rbd::Vector3d& getCOM_l1() const;
        double getTotalMass() const;

    private:

        InertiaMatrix tensor_l1;
        iit::rbd::Vector3d com_l1;
};


inline InertiaProperties::~InertiaProperties() {}

inline const InertiaMatrix& InertiaProperties::getTensor_l1() const {
    return this->tensor_l1;
}
inline double InertiaProperties::getMass_l1() const {
    return this->tensor_l1.getMass();
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_l1() const {
    return this->com_l1;
}

inline double InertiaProperties::getTotalMass() const {
    return 0.1554;
}

}
}
}

#endif

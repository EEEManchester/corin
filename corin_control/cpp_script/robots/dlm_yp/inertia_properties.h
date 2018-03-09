#ifndef IIT_ROBOT_DLM_YP_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_DLM_YP_INERTIA_PROPERTIES_H_

#include <Eigen/Dense>
#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"

namespace iit {
namespace DLM_YP {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot DLM_YP.
 */
namespace dyn {

typedef iit::rbd::InertiaMatrixDense InertiaMatrix;

class InertiaProperties {
    public:
        InertiaProperties();
        ~InertiaProperties();
        const InertiaMatrix& getTensor_l1() const;
        const InertiaMatrix& getTensor_l2() const;
        double getMass_l1() const;
        double getMass_l2() const;
        const iit::rbd::Vector3d& getCOM_l1() const;
        const iit::rbd::Vector3d& getCOM_l2() const;
        double getTotalMass() const;

    private:

        InertiaMatrix tensor_l1;
        InertiaMatrix tensor_l2;
        iit::rbd::Vector3d com_l1;
        iit::rbd::Vector3d com_l2;
};


inline InertiaProperties::~InertiaProperties() {}

inline const InertiaMatrix& InertiaProperties::getTensor_l1() const {
    return this->tensor_l1;
}
inline const InertiaMatrix& InertiaProperties::getTensor_l2() const {
    return this->tensor_l2;
}
inline double InertiaProperties::getMass_l1() const {
    return this->tensor_l1.getMass();
}
inline double InertiaProperties::getMass_l2() const {
    return this->tensor_l2.getMass();
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_l1() const {
    return this->com_l1;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_l2() const {
    return this->com_l2;
}

inline double InertiaProperties::getTotalMass() const {
    return 0.18 + 0.1554;
}

}
}
}

#endif

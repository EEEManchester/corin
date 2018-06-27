#ifndef IIT_ROBOT_MCORIN_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_MCORIN_INERTIA_PROPERTIES_H_

#include <Eigen/Dense>
#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"

namespace iit {
namespace mcorin {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot mcorin.
 */
namespace dyn {

typedef iit::rbd::InertiaMatrixDense InertiaMatrix;

class InertiaProperties {
    public:
        InertiaProperties();
        ~InertiaProperties();
        const InertiaMatrix& getTensor_trunk() const;
        const InertiaMatrix& getTensor_LF_hipassembly() const;
        const InertiaMatrix& getTensor_LF_upperleg() const;
        const InertiaMatrix& getTensor_LF_lowerleg() const;
        const InertiaMatrix& getTensor_LM_hipassembly() const;
        const InertiaMatrix& getTensor_LM_upperleg() const;
        const InertiaMatrix& getTensor_LM_lowerleg() const;
        const InertiaMatrix& getTensor_LR_hipassembly() const;
        const InertiaMatrix& getTensor_LR_upperleg() const;
        const InertiaMatrix& getTensor_LR_lowerleg() const;
        const InertiaMatrix& getTensor_RF_hipassembly() const;
        const InertiaMatrix& getTensor_RF_upperleg() const;
        const InertiaMatrix& getTensor_RF_lowerleg() const;
        const InertiaMatrix& getTensor_RM_hipassembly() const;
        const InertiaMatrix& getTensor_RM_upperleg() const;
        const InertiaMatrix& getTensor_RM_lowerleg() const;
        const InertiaMatrix& getTensor_RR_hipassembly() const;
        const InertiaMatrix& getTensor_RR_upperleg() const;
        const InertiaMatrix& getTensor_RR_lowerleg() const;
        double getMass_trunk() const;
        double getMass_LF_hipassembly() const;
        double getMass_LF_upperleg() const;
        double getMass_LF_lowerleg() const;
        double getMass_LM_hipassembly() const;
        double getMass_LM_upperleg() const;
        double getMass_LM_lowerleg() const;
        double getMass_LR_hipassembly() const;
        double getMass_LR_upperleg() const;
        double getMass_LR_lowerleg() const;
        double getMass_RF_hipassembly() const;
        double getMass_RF_upperleg() const;
        double getMass_RF_lowerleg() const;
        double getMass_RM_hipassembly() const;
        double getMass_RM_upperleg() const;
        double getMass_RM_lowerleg() const;
        double getMass_RR_hipassembly() const;
        double getMass_RR_upperleg() const;
        double getMass_RR_lowerleg() const;
        const iit::rbd::Vector3d& getCOM_trunk() const;
        const iit::rbd::Vector3d& getCOM_LF_hipassembly() const;
        const iit::rbd::Vector3d& getCOM_LF_upperleg() const;
        const iit::rbd::Vector3d& getCOM_LF_lowerleg() const;
        const iit::rbd::Vector3d& getCOM_LM_hipassembly() const;
        const iit::rbd::Vector3d& getCOM_LM_upperleg() const;
        const iit::rbd::Vector3d& getCOM_LM_lowerleg() const;
        const iit::rbd::Vector3d& getCOM_LR_hipassembly() const;
        const iit::rbd::Vector3d& getCOM_LR_upperleg() const;
        const iit::rbd::Vector3d& getCOM_LR_lowerleg() const;
        const iit::rbd::Vector3d& getCOM_RF_hipassembly() const;
        const iit::rbd::Vector3d& getCOM_RF_upperleg() const;
        const iit::rbd::Vector3d& getCOM_RF_lowerleg() const;
        const iit::rbd::Vector3d& getCOM_RM_hipassembly() const;
        const iit::rbd::Vector3d& getCOM_RM_upperleg() const;
        const iit::rbd::Vector3d& getCOM_RM_lowerleg() const;
        const iit::rbd::Vector3d& getCOM_RR_hipassembly() const;
        const iit::rbd::Vector3d& getCOM_RR_upperleg() const;
        const iit::rbd::Vector3d& getCOM_RR_lowerleg() const;
        double getTotalMass() const;

    private:

        InertiaMatrix tensor_trunk;
        InertiaMatrix tensor_LF_hipassembly;
        InertiaMatrix tensor_LF_upperleg;
        InertiaMatrix tensor_LF_lowerleg;
        InertiaMatrix tensor_LM_hipassembly;
        InertiaMatrix tensor_LM_upperleg;
        InertiaMatrix tensor_LM_lowerleg;
        InertiaMatrix tensor_LR_hipassembly;
        InertiaMatrix tensor_LR_upperleg;
        InertiaMatrix tensor_LR_lowerleg;
        InertiaMatrix tensor_RF_hipassembly;
        InertiaMatrix tensor_RF_upperleg;
        InertiaMatrix tensor_RF_lowerleg;
        InertiaMatrix tensor_RM_hipassembly;
        InertiaMatrix tensor_RM_upperleg;
        InertiaMatrix tensor_RM_lowerleg;
        InertiaMatrix tensor_RR_hipassembly;
        InertiaMatrix tensor_RR_upperleg;
        InertiaMatrix tensor_RR_lowerleg;
        iit::rbd::Vector3d com_trunk;
        iit::rbd::Vector3d com_LF_hipassembly;
        iit::rbd::Vector3d com_LF_upperleg;
        iit::rbd::Vector3d com_LF_lowerleg;
        iit::rbd::Vector3d com_LM_hipassembly;
        iit::rbd::Vector3d com_LM_upperleg;
        iit::rbd::Vector3d com_LM_lowerleg;
        iit::rbd::Vector3d com_LR_hipassembly;
        iit::rbd::Vector3d com_LR_upperleg;
        iit::rbd::Vector3d com_LR_lowerleg;
        iit::rbd::Vector3d com_RF_hipassembly;
        iit::rbd::Vector3d com_RF_upperleg;
        iit::rbd::Vector3d com_RF_lowerleg;
        iit::rbd::Vector3d com_RM_hipassembly;
        iit::rbd::Vector3d com_RM_upperleg;
        iit::rbd::Vector3d com_RM_lowerleg;
        iit::rbd::Vector3d com_RR_hipassembly;
        iit::rbd::Vector3d com_RR_upperleg;
        iit::rbd::Vector3d com_RR_lowerleg;
};


inline InertiaProperties::~InertiaProperties() {}

inline const InertiaMatrix& InertiaProperties::getTensor_trunk() const {
    return this->tensor_trunk;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LF_hipassembly() const {
    return this->tensor_LF_hipassembly;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LF_upperleg() const {
    return this->tensor_LF_upperleg;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LF_lowerleg() const {
    return this->tensor_LF_lowerleg;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LM_hipassembly() const {
    return this->tensor_LM_hipassembly;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LM_upperleg() const {
    return this->tensor_LM_upperleg;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LM_lowerleg() const {
    return this->tensor_LM_lowerleg;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LR_hipassembly() const {
    return this->tensor_LR_hipassembly;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LR_upperleg() const {
    return this->tensor_LR_upperleg;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LR_lowerleg() const {
    return this->tensor_LR_lowerleg;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RF_hipassembly() const {
    return this->tensor_RF_hipassembly;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RF_upperleg() const {
    return this->tensor_RF_upperleg;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RF_lowerleg() const {
    return this->tensor_RF_lowerleg;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RM_hipassembly() const {
    return this->tensor_RM_hipassembly;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RM_upperleg() const {
    return this->tensor_RM_upperleg;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RM_lowerleg() const {
    return this->tensor_RM_lowerleg;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RR_hipassembly() const {
    return this->tensor_RR_hipassembly;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RR_upperleg() const {
    return this->tensor_RR_upperleg;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RR_lowerleg() const {
    return this->tensor_RR_lowerleg;
}
inline double InertiaProperties::getMass_trunk() const {
    return this->tensor_trunk.getMass();
}
inline double InertiaProperties::getMass_LF_hipassembly() const {
    return this->tensor_LF_hipassembly.getMass();
}
inline double InertiaProperties::getMass_LF_upperleg() const {
    return this->tensor_LF_upperleg.getMass();
}
inline double InertiaProperties::getMass_LF_lowerleg() const {
    return this->tensor_LF_lowerleg.getMass();
}
inline double InertiaProperties::getMass_LM_hipassembly() const {
    return this->tensor_LM_hipassembly.getMass();
}
inline double InertiaProperties::getMass_LM_upperleg() const {
    return this->tensor_LM_upperleg.getMass();
}
inline double InertiaProperties::getMass_LM_lowerleg() const {
    return this->tensor_LM_lowerleg.getMass();
}
inline double InertiaProperties::getMass_LR_hipassembly() const {
    return this->tensor_LR_hipassembly.getMass();
}
inline double InertiaProperties::getMass_LR_upperleg() const {
    return this->tensor_LR_upperleg.getMass();
}
inline double InertiaProperties::getMass_LR_lowerleg() const {
    return this->tensor_LR_lowerleg.getMass();
}
inline double InertiaProperties::getMass_RF_hipassembly() const {
    return this->tensor_RF_hipassembly.getMass();
}
inline double InertiaProperties::getMass_RF_upperleg() const {
    return this->tensor_RF_upperleg.getMass();
}
inline double InertiaProperties::getMass_RF_lowerleg() const {
    return this->tensor_RF_lowerleg.getMass();
}
inline double InertiaProperties::getMass_RM_hipassembly() const {
    return this->tensor_RM_hipassembly.getMass();
}
inline double InertiaProperties::getMass_RM_upperleg() const {
    return this->tensor_RM_upperleg.getMass();
}
inline double InertiaProperties::getMass_RM_lowerleg() const {
    return this->tensor_RM_lowerleg.getMass();
}
inline double InertiaProperties::getMass_RR_hipassembly() const {
    return this->tensor_RR_hipassembly.getMass();
}
inline double InertiaProperties::getMass_RR_upperleg() const {
    return this->tensor_RR_upperleg.getMass();
}
inline double InertiaProperties::getMass_RR_lowerleg() const {
    return this->tensor_RR_lowerleg.getMass();
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_trunk() const {
    return this->com_trunk;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_LF_hipassembly() const {
    return this->com_LF_hipassembly;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_LF_upperleg() const {
    return this->com_LF_upperleg;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_LF_lowerleg() const {
    return this->com_LF_lowerleg;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_LM_hipassembly() const {
    return this->com_LM_hipassembly;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_LM_upperleg() const {
    return this->com_LM_upperleg;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_LM_lowerleg() const {
    return this->com_LM_lowerleg;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_LR_hipassembly() const {
    return this->com_LR_hipassembly;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_LR_upperleg() const {
    return this->com_LR_upperleg;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_LR_lowerleg() const {
    return this->com_LR_lowerleg;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_RF_hipassembly() const {
    return this->com_RF_hipassembly;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_RF_upperleg() const {
    return this->com_RF_upperleg;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_RF_lowerleg() const {
    return this->com_RF_lowerleg;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_RM_hipassembly() const {
    return this->com_RM_hipassembly;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_RM_upperleg() const {
    return this->com_RM_upperleg;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_RM_lowerleg() const {
    return this->com_RM_lowerleg;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_RR_hipassembly() const {
    return this->com_RR_hipassembly;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_RR_upperleg() const {
    return this->com_RR_upperleg;
}
inline const iit::rbd::Vector3d& InertiaProperties::getCOM_RR_lowerleg() const {
    return this->com_RR_lowerleg;
}

inline double InertiaProperties::getTotalMass() const {
    return 1.5 + 0.18 + 0.1554 + 0.055 + 0.18 + 0.1554 + 0.055 + 0.18 + 0.1554 + 0.055 + 0.18 + 0.1554 + 0.055 + 0.18 + 0.1554 + 0.055 + 0.18 + 0.1554 + 0.055;
}

}
}
}

#endif

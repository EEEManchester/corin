#ifndef MCORIN_JACOBIANS_H_
#define MCORIN_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace mcorin {

template<int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<JointState, COLS, M>
{};

/**
 *
 */
class Jacobians {
    public:
        class Type_fr_trunk_J_LF_foot : public JacobianT<3, Type_fr_trunk_J_LF_foot>
        {
        public:
            Type_fr_trunk_J_LF_foot();
            const Type_fr_trunk_J_LF_foot& update(const JointState&);
        protected:
        };
        
        class Type_fr_trunk_J_LM_foot : public JacobianT<3, Type_fr_trunk_J_LM_foot>
        {
        public:
            Type_fr_trunk_J_LM_foot();
            const Type_fr_trunk_J_LM_foot& update(const JointState&);
        protected:
        };
        
        class Type_fr_trunk_J_LR_foot : public JacobianT<3, Type_fr_trunk_J_LR_foot>
        {
        public:
            Type_fr_trunk_J_LR_foot();
            const Type_fr_trunk_J_LR_foot& update(const JointState&);
        protected:
        };
        
        class Type_fr_trunk_J_RF_foot : public JacobianT<3, Type_fr_trunk_J_RF_foot>
        {
        public:
            Type_fr_trunk_J_RF_foot();
            const Type_fr_trunk_J_RF_foot& update(const JointState&);
        protected:
        };
        
        class Type_fr_trunk_J_RM_foot : public JacobianT<3, Type_fr_trunk_J_RM_foot>
        {
        public:
            Type_fr_trunk_J_RM_foot();
            const Type_fr_trunk_J_RM_foot& update(const JointState&);
        protected:
        };
        
        class Type_fr_trunk_J_RR_foot : public JacobianT<3, Type_fr_trunk_J_RR_foot>
        {
        public:
            Type_fr_trunk_J_RR_foot();
            const Type_fr_trunk_J_RR_foot& update(const JointState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters();
    public:
        Type_fr_trunk_J_LF_foot fr_trunk_J_LF_foot;
        Type_fr_trunk_J_LM_foot fr_trunk_J_LM_foot;
        Type_fr_trunk_J_LR_foot fr_trunk_J_LR_foot;
        Type_fr_trunk_J_RF_foot fr_trunk_J_RF_foot;
        Type_fr_trunk_J_RM_foot fr_trunk_J_RM_foot;
        Type_fr_trunk_J_RR_foot fr_trunk_J_RR_foot;

    protected:

};


}
}

#endif

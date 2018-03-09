#ifndef DLM_YP_JACOBIANS_H_
#define DLM_YP_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace DLM_YP {

template<int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<JointState, COLS, M>
{};

/**
 *
 */
class Jacobians {
    public:
        class Type_fr_base_J_ee : public JacobianT<2, Type_fr_base_J_ee>
        {
        public:
            Type_fr_base_J_ee();
            const Type_fr_base_J_ee& update(const JointState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters();
    public:
        Type_fr_base_J_ee fr_base_J_ee;

    protected:

};


}
}

#endif

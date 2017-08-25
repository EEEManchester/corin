#ifndef SLM_JACOBIANS_H_
#define SLM_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace SLM {

template<int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<JointState, COLS, M>
{};

/**
 *
 */
class Jacobians {
    public:
        class Type_fr_base_J_ee : public JacobianT<1, Type_fr_base_J_ee>
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

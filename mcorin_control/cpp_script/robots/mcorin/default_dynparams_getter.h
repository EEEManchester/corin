#ifndef _MCORIN_DEFAULT_GETTER_INERTIA_PARAMETERS_
#define _MCORIN_DEFAULT_GETTER_INERTIA_PARAMETERS_

#include "dynamics_parameters.h"

namespace iit {
namespace mcorin {
namespace dyn {

class DefaultParamsGetter : public RuntimeParamsGetter
{
    public:
        DefaultParamsGetter() {
            resetDefaults();
        }
        ~DefaultParamsGetter() {};

    public:
        void resetDefaults() {
        }

    private:
        RuntimeInertiaParams values;
};

}
}
}
#endif

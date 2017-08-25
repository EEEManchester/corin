#ifndef _DLM_YP_DEFAULT_GETTER_INERTIA_PARAMETERS_
#define _DLM_YP_DEFAULT_GETTER_INERTIA_PARAMETERS_

#include "dynamics_parameters.h"

namespace iit {
namespace DLM_YP {
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

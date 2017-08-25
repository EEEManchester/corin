#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

iit::DLM_YP::dyn::InertiaProperties::InertiaProperties()
{
    com_l1 = iit::rbd::Vector3d(0.05821,0.0,0.0);
    tensor_l1.fill(
        0.18,
        com_l1,
        Utils::buildInertiaTensor(
                3.2E-5,
                6.43E-4,
                6.27361E-4,
                1.0E-6,
                1.0E-6,
                0.0) );

    com_l2 = iit::rbd::Vector3d(0.1213,0.0,0.0);
    tensor_l2.fill(
        0.1554,
        com_l2,
        Utils::buildInertiaTensor(
                1.9E-5,
                0.00235,
                0.002361,
                3.0E-6,
                3.0E-6,
                0.0) );

}


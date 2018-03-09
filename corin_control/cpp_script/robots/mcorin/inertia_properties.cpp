#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

iit::mcorin::dyn::InertiaProperties::InertiaProperties()
{
    com_trunk = iit::rbd::Vector3d(0.0,0.0,0.0);
    tensor_trunk.fill(
        1.5,
        com_trunk,
        Utils::buildInertiaTensor(
                0.0030125,
                0.0080125,
                0.010625,
                0.0,
                0.0,
                0.0) );

    com_LF_hipassembly = iit::rbd::Vector3d(0.05821,0.0,0.0);
    tensor_LF_hipassembly.fill(
        0.18,
        com_LF_hipassembly,
        Utils::buildInertiaTensor(
                3.2E-5,
                6.43E-4,
                6.27361E-4,
                1.0E-6,
                0.0,
                0.0) );

    com_LF_upperleg = iit::rbd::Vector3d(0.1213,0.0,0.0);
    tensor_LF_upperleg.fill(
        0.1554,
        com_LF_upperleg,
        Utils::buildInertiaTensor(
                1.9E-5,
                0.00235,
                0.002361,
                3.0E-6,
                3.0E-6,
                0.0) );

    com_LF_lowerleg = iit::rbd::Vector3d(0.08853,0.0,0.0);
    tensor_LF_lowerleg.fill(
        0.055,
        com_LF_lowerleg,
        Utils::buildInertiaTensor(
                1.6E-5,
                5.03E-4,
                5.15E-4,
                1.0E-6,
                0.0,
                0.0) );

    com_LM_hipassembly = iit::rbd::Vector3d(0.05821,0.0,0.0);
    tensor_LM_hipassembly.fill(
        0.18,
        com_LM_hipassembly,
        Utils::buildInertiaTensor(
                3.2E-5,
                6.43E-4,
                6.27361E-4,
                1.0E-6,
                0.0,
                0.0) );

    com_LM_upperleg = iit::rbd::Vector3d(0.1213,0.0,0.0);
    tensor_LM_upperleg.fill(
        0.1554,
        com_LM_upperleg,
        Utils::buildInertiaTensor(
                1.9E-5,
                0.00235,
                0.002361,
                3.0E-6,
                3.0E-6,
                0.0) );

    com_LM_lowerleg = iit::rbd::Vector3d(0.08853,0.0,0.0);
    tensor_LM_lowerleg.fill(
        0.055,
        com_LM_lowerleg,
        Utils::buildInertiaTensor(
                1.6E-5,
                5.03E-4,
                5.15E-4,
                1.0E-6,
                0.0,
                0.0) );

    com_LR_hipassembly = iit::rbd::Vector3d(0.05821,0.0,0.0);
    tensor_LR_hipassembly.fill(
        0.18,
        com_LR_hipassembly,
        Utils::buildInertiaTensor(
                3.2E-5,
                6.43E-4,
                6.27361E-4,
                1.0E-6,
                0.0,
                0.0) );

    com_LR_upperleg = iit::rbd::Vector3d(0.1213,0.0,0.0);
    tensor_LR_upperleg.fill(
        0.1554,
        com_LR_upperleg,
        Utils::buildInertiaTensor(
                1.9E-5,
                0.00235,
                0.002361,
                3.0E-6,
                3.0E-6,
                0.0) );

    com_LR_lowerleg = iit::rbd::Vector3d(0.08853,0.0,0.0);
    tensor_LR_lowerleg.fill(
        0.055,
        com_LR_lowerleg,
        Utils::buildInertiaTensor(
                1.6E-5,
                5.03E-4,
                5.15E-4,
                1.0E-6,
                0.0,
                0.0) );

    com_RF_hipassembly = iit::rbd::Vector3d(0.05821,0.0,0.0);
    tensor_RF_hipassembly.fill(
        0.18,
        com_RF_hipassembly,
        Utils::buildInertiaTensor(
                3.2E-5,
                6.43E-4,
                6.27361E-4,
                1.0E-6,
                0.0,
                0.0) );

    com_RF_upperleg = iit::rbd::Vector3d(0.1213,0.0,0.0);
    tensor_RF_upperleg.fill(
        0.1554,
        com_RF_upperleg,
        Utils::buildInertiaTensor(
                1.9E-5,
                0.00235,
                0.002361,
                3.0E-6,
                3.0E-6,
                0.0) );

    com_RF_lowerleg = iit::rbd::Vector3d(0.08853,0.0,0.0);
    tensor_RF_lowerleg.fill(
        0.055,
        com_RF_lowerleg,
        Utils::buildInertiaTensor(
                1.6E-5,
                5.03E-4,
                5.15E-4,
                1.0E-6,
                0.0,
                0.0) );

    com_RM_hipassembly = iit::rbd::Vector3d(0.05821,0.0,0.0);
    tensor_RM_hipassembly.fill(
        0.18,
        com_RM_hipassembly,
        Utils::buildInertiaTensor(
                3.2E-5,
                6.43E-4,
                6.27361E-4,
                1.0E-6,
                0.0,
                0.0) );

    com_RM_upperleg = iit::rbd::Vector3d(0.1213,0.0,0.0);
    tensor_RM_upperleg.fill(
        0.1554,
        com_RM_upperleg,
        Utils::buildInertiaTensor(
                1.9E-5,
                0.00235,
                0.002361,
                3.0E-6,
                3.0E-6,
                0.0) );

    com_RM_lowerleg = iit::rbd::Vector3d(0.08853,0.0,0.0);
    tensor_RM_lowerleg.fill(
        0.055,
        com_RM_lowerleg,
        Utils::buildInertiaTensor(
                1.6E-5,
                5.03E-4,
                5.15E-4,
                1.0E-6,
                0.0,
                0.0) );

    com_RR_hipassembly = iit::rbd::Vector3d(0.05821,0.0,0.0);
    tensor_RR_hipassembly.fill(
        0.18,
        com_RR_hipassembly,
        Utils::buildInertiaTensor(
                3.2E-5,
                6.43E-4,
                6.27361E-4,
                1.0E-6,
                0.0,
                0.0) );

    com_RR_upperleg = iit::rbd::Vector3d(0.1213,0.0,0.0);
    tensor_RR_upperleg.fill(
        0.1554,
        com_RR_upperleg,
        Utils::buildInertiaTensor(
                1.9E-5,
                0.00235,
                0.002361,
                3.0E-6,
                3.0E-6,
                0.0) );

    com_RR_lowerleg = iit::rbd::Vector3d(0.08853,0.0,0.0);
    tensor_RR_lowerleg.fill(
        0.055,
        com_RR_lowerleg,
        Utils::buildInertiaTensor(
                1.6E-5,
                5.03E-4,
                5.15E-4,
                1.0E-6,
                0.0,
                0.0) );

}


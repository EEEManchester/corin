#ifndef IIT_ROBOT_MCORIN_FORWARD_DYNAMICS_H_
#define IIT_ROBOT_MCORIN_FORWARD_DYNAMICS_H_

#include <Eigen/Dense>
#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace iit {
namespace mcorin {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot mcorin.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */
class ForwardDynamics {
public:
    // Convenient type aliases:
    typedef LinkDataMap<iit::rbd::ForceVector> ExtForces;
    typedef iit::rbd::ForceVector Force;
    typedef iit::rbd::VelocityVector Velocity;
    typedef iit::rbd::VelocityVector Acceleration;
public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot mcorin, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param trunk_a
     * \param trunk_v
     * \param g the gravity acceleration vector, expressed in the
     *          base coordinates
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
       JointState& qdd, Acceleration& trunk_a, // output parameters,
       const Velocity& trunk_v, const Acceleration& g,
       const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, Acceleration& trunk_a, // output parameters,
        const Velocity& trunk_v, const Acceleration& g,
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* motionTransforms;

    iit::rbd::Matrix66d vcross; // support variable
    iit::rbd::Matrix66d Ia_r;   // support variable, articulated inertia in the case of a revolute joint
    // Link 'trunk'
    iit::rbd::Matrix66d trunk_AI;
    Force trunk_p;

    // Link 'LF_hipassembly' :
    iit::rbd::Matrix66d LF_hipassembly_AI;
    Velocity LF_hipassembly_a;
    Velocity LF_hipassembly_v;
    Velocity LF_hipassembly_c;
    Force    LF_hipassembly_p;

    iit::rbd::Column6d LF_hipassembly_U;
    double LF_hipassembly_D;
    double LF_hipassembly_u;
    // Link 'LF_upperleg' :
    iit::rbd::Matrix66d LF_upperleg_AI;
    Velocity LF_upperleg_a;
    Velocity LF_upperleg_v;
    Velocity LF_upperleg_c;
    Force    LF_upperleg_p;

    iit::rbd::Column6d LF_upperleg_U;
    double LF_upperleg_D;
    double LF_upperleg_u;
    // Link 'LF_lowerleg' :
    iit::rbd::Matrix66d LF_lowerleg_AI;
    Velocity LF_lowerleg_a;
    Velocity LF_lowerleg_v;
    Velocity LF_lowerleg_c;
    Force    LF_lowerleg_p;

    iit::rbd::Column6d LF_lowerleg_U;
    double LF_lowerleg_D;
    double LF_lowerleg_u;
    // Link 'LM_hipassembly' :
    iit::rbd::Matrix66d LM_hipassembly_AI;
    Velocity LM_hipassembly_a;
    Velocity LM_hipassembly_v;
    Velocity LM_hipassembly_c;
    Force    LM_hipassembly_p;

    iit::rbd::Column6d LM_hipassembly_U;
    double LM_hipassembly_D;
    double LM_hipassembly_u;
    // Link 'LM_upperleg' :
    iit::rbd::Matrix66d LM_upperleg_AI;
    Velocity LM_upperleg_a;
    Velocity LM_upperleg_v;
    Velocity LM_upperleg_c;
    Force    LM_upperleg_p;

    iit::rbd::Column6d LM_upperleg_U;
    double LM_upperleg_D;
    double LM_upperleg_u;
    // Link 'LM_lowerleg' :
    iit::rbd::Matrix66d LM_lowerleg_AI;
    Velocity LM_lowerleg_a;
    Velocity LM_lowerleg_v;
    Velocity LM_lowerleg_c;
    Force    LM_lowerleg_p;

    iit::rbd::Column6d LM_lowerleg_U;
    double LM_lowerleg_D;
    double LM_lowerleg_u;
    // Link 'LR_hipassembly' :
    iit::rbd::Matrix66d LR_hipassembly_AI;
    Velocity LR_hipassembly_a;
    Velocity LR_hipassembly_v;
    Velocity LR_hipassembly_c;
    Force    LR_hipassembly_p;

    iit::rbd::Column6d LR_hipassembly_U;
    double LR_hipassembly_D;
    double LR_hipassembly_u;
    // Link 'LR_upperleg' :
    iit::rbd::Matrix66d LR_upperleg_AI;
    Velocity LR_upperleg_a;
    Velocity LR_upperleg_v;
    Velocity LR_upperleg_c;
    Force    LR_upperleg_p;

    iit::rbd::Column6d LR_upperleg_U;
    double LR_upperleg_D;
    double LR_upperleg_u;
    // Link 'LR_lowerleg' :
    iit::rbd::Matrix66d LR_lowerleg_AI;
    Velocity LR_lowerleg_a;
    Velocity LR_lowerleg_v;
    Velocity LR_lowerleg_c;
    Force    LR_lowerleg_p;

    iit::rbd::Column6d LR_lowerleg_U;
    double LR_lowerleg_D;
    double LR_lowerleg_u;
    // Link 'RF_hipassembly' :
    iit::rbd::Matrix66d RF_hipassembly_AI;
    Velocity RF_hipassembly_a;
    Velocity RF_hipassembly_v;
    Velocity RF_hipassembly_c;
    Force    RF_hipassembly_p;

    iit::rbd::Column6d RF_hipassembly_U;
    double RF_hipassembly_D;
    double RF_hipassembly_u;
    // Link 'RF_upperleg' :
    iit::rbd::Matrix66d RF_upperleg_AI;
    Velocity RF_upperleg_a;
    Velocity RF_upperleg_v;
    Velocity RF_upperleg_c;
    Force    RF_upperleg_p;

    iit::rbd::Column6d RF_upperleg_U;
    double RF_upperleg_D;
    double RF_upperleg_u;
    // Link 'RF_lowerleg' :
    iit::rbd::Matrix66d RF_lowerleg_AI;
    Velocity RF_lowerleg_a;
    Velocity RF_lowerleg_v;
    Velocity RF_lowerleg_c;
    Force    RF_lowerleg_p;

    iit::rbd::Column6d RF_lowerleg_U;
    double RF_lowerleg_D;
    double RF_lowerleg_u;
    // Link 'RM_hipassembly' :
    iit::rbd::Matrix66d RM_hipassembly_AI;
    Velocity RM_hipassembly_a;
    Velocity RM_hipassembly_v;
    Velocity RM_hipassembly_c;
    Force    RM_hipassembly_p;

    iit::rbd::Column6d RM_hipassembly_U;
    double RM_hipassembly_D;
    double RM_hipassembly_u;
    // Link 'RM_upperleg' :
    iit::rbd::Matrix66d RM_upperleg_AI;
    Velocity RM_upperleg_a;
    Velocity RM_upperleg_v;
    Velocity RM_upperleg_c;
    Force    RM_upperleg_p;

    iit::rbd::Column6d RM_upperleg_U;
    double RM_upperleg_D;
    double RM_upperleg_u;
    // Link 'RM_lowerleg' :
    iit::rbd::Matrix66d RM_lowerleg_AI;
    Velocity RM_lowerleg_a;
    Velocity RM_lowerleg_v;
    Velocity RM_lowerleg_c;
    Force    RM_lowerleg_p;

    iit::rbd::Column6d RM_lowerleg_U;
    double RM_lowerleg_D;
    double RM_lowerleg_u;
    // Link 'RR_hipassembly' :
    iit::rbd::Matrix66d RR_hipassembly_AI;
    Velocity RR_hipassembly_a;
    Velocity RR_hipassembly_v;
    Velocity RR_hipassembly_c;
    Force    RR_hipassembly_p;

    iit::rbd::Column6d RR_hipassembly_U;
    double RR_hipassembly_D;
    double RR_hipassembly_u;
    // Link 'RR_upperleg' :
    iit::rbd::Matrix66d RR_upperleg_AI;
    Velocity RR_upperleg_a;
    Velocity RR_upperleg_v;
    Velocity RR_upperleg_c;
    Force    RR_upperleg_p;

    iit::rbd::Column6d RR_upperleg_U;
    double RR_upperleg_D;
    double RR_upperleg_u;
    // Link 'RR_lowerleg' :
    iit::rbd::Matrix66d RR_lowerleg_AI;
    Velocity RR_lowerleg_a;
    Velocity RR_lowerleg_v;
    Velocity RR_lowerleg_c;
    Force    RR_lowerleg_p;

    iit::rbd::Column6d RR_lowerleg_U;
    double RR_lowerleg_D;
    double RR_lowerleg_u;
private:
    static const ExtForces zeroExtForces;
};

inline void ForwardDynamics::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_LF_hipassembly_X_fr_trunk)(q);
    (motionTransforms-> fr_LF_upperleg_X_fr_LF_hipassembly)(q);
    (motionTransforms-> fr_LF_lowerleg_X_fr_LF_upperleg)(q);
    (motionTransforms-> fr_LM_hipassembly_X_fr_trunk)(q);
    (motionTransforms-> fr_LM_upperleg_X_fr_LM_hipassembly)(q);
    (motionTransforms-> fr_LM_lowerleg_X_fr_LM_upperleg)(q);
    (motionTransforms-> fr_LR_hipassembly_X_fr_trunk)(q);
    (motionTransforms-> fr_LR_upperleg_X_fr_LR_hipassembly)(q);
    (motionTransforms-> fr_LR_lowerleg_X_fr_LR_upperleg)(q);
    (motionTransforms-> fr_RF_hipassembly_X_fr_trunk)(q);
    (motionTransforms-> fr_RF_upperleg_X_fr_RF_hipassembly)(q);
    (motionTransforms-> fr_RF_lowerleg_X_fr_RF_upperleg)(q);
    (motionTransforms-> fr_RM_hipassembly_X_fr_trunk)(q);
    (motionTransforms-> fr_RM_upperleg_X_fr_RM_hipassembly)(q);
    (motionTransforms-> fr_RM_lowerleg_X_fr_RM_upperleg)(q);
    (motionTransforms-> fr_RR_hipassembly_X_fr_trunk)(q);
    (motionTransforms-> fr_RR_upperleg_X_fr_RR_hipassembly)(q);
    (motionTransforms-> fr_RR_lowerleg_X_fr_RR_upperleg)(q);
}

inline void ForwardDynamics::fd(
    JointState& qdd, Acceleration& trunk_a, // output parameters,
    const Velocity& trunk_v, const Acceleration& g,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, trunk_a, trunk_v, g, qd, tau, fext);
}

}
}
}

#endif

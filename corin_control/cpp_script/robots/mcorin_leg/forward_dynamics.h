#ifndef IIT_ROBOT_MCORIN_LEG_FORWARD_DYNAMICS_H_
#define IIT_ROBOT_MCORIN_LEG_FORWARD_DYNAMICS_H_

#include <Eigen/Dense>
#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace iit {
namespace MCORIN_LEG {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot MCORIN_LEG.
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
     *     the robot MCORIN_LEG, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
        JointState& qdd, // output parameter
        const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, // output parameter
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* motionTransforms;

    iit::rbd::Matrix66d vcross; // support variable
    iit::rbd::Matrix66d Ia_r;   // support variable, articulated inertia in the case of a revolute joint

    // Link 'l1' :
    iit::rbd::Matrix66d l1_AI;
    Velocity l1_a;
    Velocity l1_v;
    Velocity l1_c;
    Force    l1_p;

    iit::rbd::Column6d l1_U;
    double l1_D;
    double l1_u;
    // Link 'l2' :
    iit::rbd::Matrix66d l2_AI;
    Velocity l2_a;
    Velocity l2_v;
    Velocity l2_c;
    Force    l2_p;

    iit::rbd::Column6d l2_U;
    double l2_D;
    double l2_u;
    // Link 'l3' :
    iit::rbd::Matrix66d l3_AI;
    Velocity l3_a;
    Velocity l3_v;
    Velocity l3_c;
    Force    l3_p;

    iit::rbd::Column6d l3_U;
    double l3_D;
    double l3_u;
private:
    static const ExtForces zeroExtForces;
};

inline void ForwardDynamics::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_l1_X_fr_base)(q);
    (motionTransforms-> fr_l2_X_fr_l1)(q);
    (motionTransforms-> fr_l3_X_fr_l2)(q);
}

inline void ForwardDynamics::fd(
    JointState& qdd,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, qd, tau, fext);
}

}
}
}

#endif

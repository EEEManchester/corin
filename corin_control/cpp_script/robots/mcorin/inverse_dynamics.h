#ifndef IIT_MCORIN_INVERSE_DYNAMICS_H_
#define IIT_MCORIN_INVERSE_DYNAMICS_H_

#include <Eigen/Dense>
#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace iit {
namespace mcorin {
namespace dyn {

/**
 * The Inverse Dynamics routine for the robot mcorin.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */
class InverseDynamics {
public:

    typedef iit::rbd::ForceVector Force;
    typedef iit::rbd::VelocityVector Velocity;
    typedef iit::rbd::VelocityVector Acceleration;
    typedef iit::rbd::InertiaMatrixDense InertiaMatrix;
    typedef LinkDataMap<iit::rbd::ForceVector> ExtForces;
public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot mcorin, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(InertiaProperties& in, MotionTransforms& tr);

    /** \name Inverse dynamics
     * The full algorithm for the inverse dynamics of this robot.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[out] baseAccel the spatial acceleration of the robot base
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] trunk_v the spatial velocity of the base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id(
        JointState& jForces, Acceleration& trunk_a,
        const Acceleration& g, const Velocity& trunk_v,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces, Acceleration& trunk_a,
        const Acceleration& g, const Velocity& trunk_v,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Inverse dynamics, fully actuated base
     * The inverse dynamics algorithm for the floating base robot,
     * in the assumption of a fully actuated base.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] baseWrench the spatial force to be applied to the robot base to achieve
     *             the desired accelerations
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] trunk_v the spatial velocity of the base
     * \param[in] baseAccel the desired spatial acceleration of the robot base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& trunk_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& trunk_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Gravity terms, fully actuated base
     */
    ///@{
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const JointState& q);
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g);
    ///@}
    /** \name Centrifugal and Coriolis terms, fully actuated base
     *
     * These functions take only velocity inputs, that is, they assume
     * a zero spatial acceleration of the base (in addition to zero acceleration
     * at the actuated joints).
     * Note that this is NOT the same as imposing zero acceleration
     * at the virtual 6-dof-floting-base joint, which would result, in general,
     * in a non-zero spatial acceleration of the base, due to velocity
     * product terms.
     */
    ///@{
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& trunk_v, const JointState& q, const JointState& qd);
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& trunk_v, const JointState& qd);
    ///@}
    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    void setJointStatus(const JointState& q) const;

public:
    /** \name Getters
     * These functions return various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * The getters can be useful to retrieve the additional data that is not
     * returned explicitly by the inverse dynamics routines even though it
     * is computed. For example, after a call to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    const Force& getForce_trunk() const { return trunk_f; }
    const Velocity& getVelocity_LF_hipassembly() const { return LF_hipassembly_v; }
    const Acceleration& getAcceleration_LF_hipassembly() const { return LF_hipassembly_a; }
    const Force& getForce_LF_hipassembly() const { return LF_hipassembly_f; }
    const Velocity& getVelocity_LF_upperleg() const { return LF_upperleg_v; }
    const Acceleration& getAcceleration_LF_upperleg() const { return LF_upperleg_a; }
    const Force& getForce_LF_upperleg() const { return LF_upperleg_f; }
    const Velocity& getVelocity_LF_lowerleg() const { return LF_lowerleg_v; }
    const Acceleration& getAcceleration_LF_lowerleg() const { return LF_lowerleg_a; }
    const Force& getForce_LF_lowerleg() const { return LF_lowerleg_f; }
    const Velocity& getVelocity_LM_hipassembly() const { return LM_hipassembly_v; }
    const Acceleration& getAcceleration_LM_hipassembly() const { return LM_hipassembly_a; }
    const Force& getForce_LM_hipassembly() const { return LM_hipassembly_f; }
    const Velocity& getVelocity_LM_upperleg() const { return LM_upperleg_v; }
    const Acceleration& getAcceleration_LM_upperleg() const { return LM_upperleg_a; }
    const Force& getForce_LM_upperleg() const { return LM_upperleg_f; }
    const Velocity& getVelocity_LM_lowerleg() const { return LM_lowerleg_v; }
    const Acceleration& getAcceleration_LM_lowerleg() const { return LM_lowerleg_a; }
    const Force& getForce_LM_lowerleg() const { return LM_lowerleg_f; }
    const Velocity& getVelocity_LR_hipassembly() const { return LR_hipassembly_v; }
    const Acceleration& getAcceleration_LR_hipassembly() const { return LR_hipassembly_a; }
    const Force& getForce_LR_hipassembly() const { return LR_hipassembly_f; }
    const Velocity& getVelocity_LR_upperleg() const { return LR_upperleg_v; }
    const Acceleration& getAcceleration_LR_upperleg() const { return LR_upperleg_a; }
    const Force& getForce_LR_upperleg() const { return LR_upperleg_f; }
    const Velocity& getVelocity_LR_lowerleg() const { return LR_lowerleg_v; }
    const Acceleration& getAcceleration_LR_lowerleg() const { return LR_lowerleg_a; }
    const Force& getForce_LR_lowerleg() const { return LR_lowerleg_f; }
    const Velocity& getVelocity_RF_hipassembly() const { return RF_hipassembly_v; }
    const Acceleration& getAcceleration_RF_hipassembly() const { return RF_hipassembly_a; }
    const Force& getForce_RF_hipassembly() const { return RF_hipassembly_f; }
    const Velocity& getVelocity_RF_upperleg() const { return RF_upperleg_v; }
    const Acceleration& getAcceleration_RF_upperleg() const { return RF_upperleg_a; }
    const Force& getForce_RF_upperleg() const { return RF_upperleg_f; }
    const Velocity& getVelocity_RF_lowerleg() const { return RF_lowerleg_v; }
    const Acceleration& getAcceleration_RF_lowerleg() const { return RF_lowerleg_a; }
    const Force& getForce_RF_lowerleg() const { return RF_lowerleg_f; }
    const Velocity& getVelocity_RM_hipassembly() const { return RM_hipassembly_v; }
    const Acceleration& getAcceleration_RM_hipassembly() const { return RM_hipassembly_a; }
    const Force& getForce_RM_hipassembly() const { return RM_hipassembly_f; }
    const Velocity& getVelocity_RM_upperleg() const { return RM_upperleg_v; }
    const Acceleration& getAcceleration_RM_upperleg() const { return RM_upperleg_a; }
    const Force& getForce_RM_upperleg() const { return RM_upperleg_f; }
    const Velocity& getVelocity_RM_lowerleg() const { return RM_lowerleg_v; }
    const Acceleration& getAcceleration_RM_lowerleg() const { return RM_lowerleg_a; }
    const Force& getForce_RM_lowerleg() const { return RM_lowerleg_f; }
    const Velocity& getVelocity_RR_hipassembly() const { return RR_hipassembly_v; }
    const Acceleration& getAcceleration_RR_hipassembly() const { return RR_hipassembly_a; }
    const Force& getForce_RR_hipassembly() const { return RR_hipassembly_f; }
    const Velocity& getVelocity_RR_upperleg() const { return RR_upperleg_v; }
    const Acceleration& getAcceleration_RR_upperleg() const { return RR_upperleg_a; }
    const Force& getForce_RR_upperleg() const { return RR_upperleg_f; }
    const Velocity& getVelocity_RR_lowerleg() const { return RR_lowerleg_v; }
    const Acceleration& getAcceleration_RR_lowerleg() const { return RR_lowerleg_a; }
    const Force& getForce_RR_lowerleg() const { return RR_lowerleg_f; }
    ///@}
protected:
    void secondPass_fullyActuated(JointState& jForces);

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* xm;
private:
    iit::rbd::Matrix66d vcross; // support variable
    // Link 'LF_hipassembly' :
    const InertiaMatrix& LF_hipassembly_I;
    Velocity      LF_hipassembly_v;
    Acceleration  LF_hipassembly_a;
    Force         LF_hipassembly_f;
    // Link 'LF_upperleg' :
    const InertiaMatrix& LF_upperleg_I;
    Velocity      LF_upperleg_v;
    Acceleration  LF_upperleg_a;
    Force         LF_upperleg_f;
    // Link 'LF_lowerleg' :
    const InertiaMatrix& LF_lowerleg_I;
    Velocity      LF_lowerleg_v;
    Acceleration  LF_lowerleg_a;
    Force         LF_lowerleg_f;
    // Link 'LM_hipassembly' :
    const InertiaMatrix& LM_hipassembly_I;
    Velocity      LM_hipassembly_v;
    Acceleration  LM_hipassembly_a;
    Force         LM_hipassembly_f;
    // Link 'LM_upperleg' :
    const InertiaMatrix& LM_upperleg_I;
    Velocity      LM_upperleg_v;
    Acceleration  LM_upperleg_a;
    Force         LM_upperleg_f;
    // Link 'LM_lowerleg' :
    const InertiaMatrix& LM_lowerleg_I;
    Velocity      LM_lowerleg_v;
    Acceleration  LM_lowerleg_a;
    Force         LM_lowerleg_f;
    // Link 'LR_hipassembly' :
    const InertiaMatrix& LR_hipassembly_I;
    Velocity      LR_hipassembly_v;
    Acceleration  LR_hipassembly_a;
    Force         LR_hipassembly_f;
    // Link 'LR_upperleg' :
    const InertiaMatrix& LR_upperleg_I;
    Velocity      LR_upperleg_v;
    Acceleration  LR_upperleg_a;
    Force         LR_upperleg_f;
    // Link 'LR_lowerleg' :
    const InertiaMatrix& LR_lowerleg_I;
    Velocity      LR_lowerleg_v;
    Acceleration  LR_lowerleg_a;
    Force         LR_lowerleg_f;
    // Link 'RF_hipassembly' :
    const InertiaMatrix& RF_hipassembly_I;
    Velocity      RF_hipassembly_v;
    Acceleration  RF_hipassembly_a;
    Force         RF_hipassembly_f;
    // Link 'RF_upperleg' :
    const InertiaMatrix& RF_upperleg_I;
    Velocity      RF_upperleg_v;
    Acceleration  RF_upperleg_a;
    Force         RF_upperleg_f;
    // Link 'RF_lowerleg' :
    const InertiaMatrix& RF_lowerleg_I;
    Velocity      RF_lowerleg_v;
    Acceleration  RF_lowerleg_a;
    Force         RF_lowerleg_f;
    // Link 'RM_hipassembly' :
    const InertiaMatrix& RM_hipassembly_I;
    Velocity      RM_hipassembly_v;
    Acceleration  RM_hipassembly_a;
    Force         RM_hipassembly_f;
    // Link 'RM_upperleg' :
    const InertiaMatrix& RM_upperleg_I;
    Velocity      RM_upperleg_v;
    Acceleration  RM_upperleg_a;
    Force         RM_upperleg_f;
    // Link 'RM_lowerleg' :
    const InertiaMatrix& RM_lowerleg_I;
    Velocity      RM_lowerleg_v;
    Acceleration  RM_lowerleg_a;
    Force         RM_lowerleg_f;
    // Link 'RR_hipassembly' :
    const InertiaMatrix& RR_hipassembly_I;
    Velocity      RR_hipassembly_v;
    Acceleration  RR_hipassembly_a;
    Force         RR_hipassembly_f;
    // Link 'RR_upperleg' :
    const InertiaMatrix& RR_upperleg_I;
    Velocity      RR_upperleg_v;
    Acceleration  RR_upperleg_a;
    Force         RR_upperleg_f;
    // Link 'RR_lowerleg' :
    const InertiaMatrix& RR_lowerleg_I;
    Velocity      RR_lowerleg_v;
    Acceleration  RR_lowerleg_a;
    Force         RR_lowerleg_f;

    // The robot base
    const InertiaMatrix& trunk_I;
    InertiaMatrix trunk_Ic;
    Force         trunk_f;
    // The composite inertia tensors
    InertiaMatrix LF_hipassembly_Ic;
    InertiaMatrix LF_upperleg_Ic;
    const InertiaMatrix& LF_lowerleg_Ic;
    InertiaMatrix LM_hipassembly_Ic;
    InertiaMatrix LM_upperleg_Ic;
    const InertiaMatrix& LM_lowerleg_Ic;
    InertiaMatrix LR_hipassembly_Ic;
    InertiaMatrix LR_upperleg_Ic;
    const InertiaMatrix& LR_lowerleg_Ic;
    InertiaMatrix RF_hipassembly_Ic;
    InertiaMatrix RF_upperleg_Ic;
    const InertiaMatrix& RF_lowerleg_Ic;
    InertiaMatrix RM_hipassembly_Ic;
    InertiaMatrix RM_upperleg_Ic;
    const InertiaMatrix& RM_lowerleg_Ic;
    InertiaMatrix RR_hipassembly_Ic;
    InertiaMatrix RR_upperleg_Ic;
    const InertiaMatrix& RR_lowerleg_Ic;

private:
    static const ExtForces zeroExtForces;
};

inline void InverseDynamics::setJointStatus(const JointState& q) const
{
    (xm->fr_LF_hipassembly_X_fr_trunk)(q);
    (xm->fr_RF_hipassembly_X_fr_trunk)(q);
    (xm->fr_LF_upperleg_X_fr_LF_hipassembly)(q);
    (xm->fr_RF_upperleg_X_fr_RF_hipassembly)(q);
    (xm->fr_LF_lowerleg_X_fr_LF_upperleg)(q);
    (xm->fr_RF_lowerleg_X_fr_RF_upperleg)(q);
    (xm->fr_LM_hipassembly_X_fr_trunk)(q);
    (xm->fr_LR_hipassembly_X_fr_trunk)(q);
    (xm->fr_RM_hipassembly_X_fr_trunk)(q);
    (xm->fr_RR_hipassembly_X_fr_trunk)(q);
    (xm->fr_LM_upperleg_X_fr_LM_hipassembly)(q);
    (xm->fr_LR_upperleg_X_fr_LR_hipassembly)(q);
    (xm->fr_RM_upperleg_X_fr_RM_hipassembly)(q);
    (xm->fr_RR_upperleg_X_fr_RR_hipassembly)(q);
    (xm->fr_LM_lowerleg_X_fr_LM_upperleg)(q);
    (xm->fr_LR_lowerleg_X_fr_LR_upperleg)(q);
    (xm->fr_RM_lowerleg_X_fr_RM_upperleg)(q);
    (xm->fr_RR_lowerleg_X_fr_RR_upperleg)(q);
}

inline void InverseDynamics::id(
    JointState& jForces, Acceleration& trunk_a,
    const Acceleration& g, const Velocity& trunk_v,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, trunk_a, g, trunk_v,
       qd, qdd, fext);
}

inline void InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g, const JointState& q)
{
    setJointStatus(q);
    G_terms_fully_actuated(baseWrench, jForces, g);
}

inline void InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& trunk_v, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms_fully_actuated(baseWrench, jForces, trunk_v, qd);
}

inline void InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& trunk_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    setJointStatus(q);
    id_fully_actuated(baseWrench, jForces, g, trunk_v,
        baseAccel, qd, qdd, fext);
}

}
}
}

#endif

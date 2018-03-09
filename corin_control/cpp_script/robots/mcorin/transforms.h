#ifndef MCORIN_TRANSFORMS_H_
#define MCORIN_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace mcorin {

// The type of the "vector" with the status of the variables
typedef iit::mcorin::JointState state_t;

template<class M>
class TransformMotion : public iit::rbd::SpatialTransformBase<state_t, M> {};

template<class M>
class TransformForce : public iit::rbd::SpatialTransformBase<state_t, M> {};

template<class M>
class TransformHomogeneous : public iit::rbd::HomogeneousTransformBase<state_t, M> {};


/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
class MotionTransforms {
public:
    class Dummy {};
    typedef TransformMotion<Dummy>::MatrixType MatrixType;
public:
    class Type_fr_trunk_X_LF_hipassemblyCOM : public TransformMotion<Type_fr_trunk_X_LF_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_LF_hipassemblyCOM();
        const Type_fr_trunk_X_LF_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_upperlegCOM : public TransformMotion<Type_fr_trunk_X_LF_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_LF_upperlegCOM();
        const Type_fr_trunk_X_LF_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_lowerlegCOM : public TransformMotion<Type_fr_trunk_X_LF_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_LF_lowerlegCOM();
        const Type_fr_trunk_X_LF_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LM_hipassemblyCOM : public TransformMotion<Type_fr_trunk_X_LM_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_LM_hipassemblyCOM();
        const Type_fr_trunk_X_LM_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LM_upperlegCOM : public TransformMotion<Type_fr_trunk_X_LM_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_LM_upperlegCOM();
        const Type_fr_trunk_X_LM_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LM_lowerlegCOM : public TransformMotion<Type_fr_trunk_X_LM_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_LM_lowerlegCOM();
        const Type_fr_trunk_X_LM_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LR_hipassemblyCOM : public TransformMotion<Type_fr_trunk_X_LR_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_LR_hipassemblyCOM();
        const Type_fr_trunk_X_LR_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LR_upperlegCOM : public TransformMotion<Type_fr_trunk_X_LR_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_LR_upperlegCOM();
        const Type_fr_trunk_X_LR_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LR_lowerlegCOM : public TransformMotion<Type_fr_trunk_X_LR_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_LR_lowerlegCOM();
        const Type_fr_trunk_X_LR_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_hipassemblyCOM : public TransformMotion<Type_fr_trunk_X_RF_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_RF_hipassemblyCOM();
        const Type_fr_trunk_X_RF_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_upperlegCOM : public TransformMotion<Type_fr_trunk_X_RF_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_RF_upperlegCOM();
        const Type_fr_trunk_X_RF_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_lowerlegCOM : public TransformMotion<Type_fr_trunk_X_RF_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_RF_lowerlegCOM();
        const Type_fr_trunk_X_RF_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RM_hipassemblyCOM : public TransformMotion<Type_fr_trunk_X_RM_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_RM_hipassemblyCOM();
        const Type_fr_trunk_X_RM_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RM_upperlegCOM : public TransformMotion<Type_fr_trunk_X_RM_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_RM_upperlegCOM();
        const Type_fr_trunk_X_RM_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RM_lowerlegCOM : public TransformMotion<Type_fr_trunk_X_RM_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_RM_lowerlegCOM();
        const Type_fr_trunk_X_RM_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RR_hipassemblyCOM : public TransformMotion<Type_fr_trunk_X_RR_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_RR_hipassemblyCOM();
        const Type_fr_trunk_X_RR_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RR_upperlegCOM : public TransformMotion<Type_fr_trunk_X_RR_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_RR_upperlegCOM();
        const Type_fr_trunk_X_RR_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RR_lowerlegCOM : public TransformMotion<Type_fr_trunk_X_RR_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_RR_lowerlegCOM();
        const Type_fr_trunk_X_RR_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_trunk : public TransformMotion<Type_fr_LF_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_LF_upperleg_X_fr_trunk();
        const Type_fr_LF_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_lowerleg_X_fr_trunk : public TransformMotion<Type_fr_LF_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_LF_lowerleg_X_fr_trunk();
        const Type_fr_LF_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_upperleg_X_fr_trunk : public TransformMotion<Type_fr_LM_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_LM_upperleg_X_fr_trunk();
        const Type_fr_LM_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_lowerleg_X_fr_trunk : public TransformMotion<Type_fr_LM_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_LM_lowerleg_X_fr_trunk();
        const Type_fr_LM_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_upperleg_X_fr_trunk : public TransformMotion<Type_fr_LR_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_LR_upperleg_X_fr_trunk();
        const Type_fr_LR_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_lowerleg_X_fr_trunk : public TransformMotion<Type_fr_LR_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_LR_lowerleg_X_fr_trunk();
        const Type_fr_LR_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_trunk : public TransformMotion<Type_fr_RF_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_RF_upperleg_X_fr_trunk();
        const Type_fr_RF_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_lowerleg_X_fr_trunk : public TransformMotion<Type_fr_RF_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_RF_lowerleg_X_fr_trunk();
        const Type_fr_RF_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_upperleg_X_fr_trunk : public TransformMotion<Type_fr_RM_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_RM_upperleg_X_fr_trunk();
        const Type_fr_RM_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_lowerleg_X_fr_trunk : public TransformMotion<Type_fr_RM_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_RM_lowerleg_X_fr_trunk();
        const Type_fr_RM_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_upperleg_X_fr_trunk : public TransformMotion<Type_fr_RR_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_RR_upperleg_X_fr_trunk();
        const Type_fr_RR_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_lowerleg_X_fr_trunk : public TransformMotion<Type_fr_RR_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_RR_lowerleg_X_fr_trunk();
        const Type_fr_RR_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_foot : public TransformMotion<Type_fr_trunk_X_LF_foot>
    {
    public:
        Type_fr_trunk_X_LF_foot();
        const Type_fr_trunk_X_LF_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_q1_joint : public TransformMotion<Type_fr_trunk_X_fr_LF_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_LF_q1_joint();
        const Type_fr_trunk_X_fr_LF_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_q2_joint : public TransformMotion<Type_fr_trunk_X_fr_LF_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_LF_q2_joint();
        const Type_fr_trunk_X_fr_LF_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_q3_joint : public TransformMotion<Type_fr_trunk_X_fr_LF_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_LF_q3_joint();
        const Type_fr_trunk_X_fr_LF_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LM_foot : public TransformMotion<Type_fr_trunk_X_LM_foot>
    {
    public:
        Type_fr_trunk_X_LM_foot();
        const Type_fr_trunk_X_LM_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LM_q1_joint : public TransformMotion<Type_fr_trunk_X_fr_LM_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_LM_q1_joint();
        const Type_fr_trunk_X_fr_LM_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LM_q2_joint : public TransformMotion<Type_fr_trunk_X_fr_LM_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_LM_q2_joint();
        const Type_fr_trunk_X_fr_LM_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LM_q3_joint : public TransformMotion<Type_fr_trunk_X_fr_LM_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_LM_q3_joint();
        const Type_fr_trunk_X_fr_LM_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LR_foot : public TransformMotion<Type_fr_trunk_X_LR_foot>
    {
    public:
        Type_fr_trunk_X_LR_foot();
        const Type_fr_trunk_X_LR_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LR_q1_joint : public TransformMotion<Type_fr_trunk_X_fr_LR_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_LR_q1_joint();
        const Type_fr_trunk_X_fr_LR_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LR_q2_joint : public TransformMotion<Type_fr_trunk_X_fr_LR_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_LR_q2_joint();
        const Type_fr_trunk_X_fr_LR_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LR_q3_joint : public TransformMotion<Type_fr_trunk_X_fr_LR_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_LR_q3_joint();
        const Type_fr_trunk_X_fr_LR_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_foot : public TransformMotion<Type_fr_trunk_X_RF_foot>
    {
    public:
        Type_fr_trunk_X_RF_foot();
        const Type_fr_trunk_X_RF_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_q1_joint : public TransformMotion<Type_fr_trunk_X_fr_RF_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_RF_q1_joint();
        const Type_fr_trunk_X_fr_RF_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_q2_joint : public TransformMotion<Type_fr_trunk_X_fr_RF_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_RF_q2_joint();
        const Type_fr_trunk_X_fr_RF_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_q3_joint : public TransformMotion<Type_fr_trunk_X_fr_RF_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_RF_q3_joint();
        const Type_fr_trunk_X_fr_RF_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RM_foot : public TransformMotion<Type_fr_trunk_X_RM_foot>
    {
    public:
        Type_fr_trunk_X_RM_foot();
        const Type_fr_trunk_X_RM_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RM_q1_joint : public TransformMotion<Type_fr_trunk_X_fr_RM_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_RM_q1_joint();
        const Type_fr_trunk_X_fr_RM_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RM_q2_joint : public TransformMotion<Type_fr_trunk_X_fr_RM_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_RM_q2_joint();
        const Type_fr_trunk_X_fr_RM_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RM_q3_joint : public TransformMotion<Type_fr_trunk_X_fr_RM_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_RM_q3_joint();
        const Type_fr_trunk_X_fr_RM_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RR_foot : public TransformMotion<Type_fr_trunk_X_RR_foot>
    {
    public:
        Type_fr_trunk_X_RR_foot();
        const Type_fr_trunk_X_RR_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RR_q1_joint : public TransformMotion<Type_fr_trunk_X_fr_RR_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_RR_q1_joint();
        const Type_fr_trunk_X_fr_RR_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RR_q2_joint : public TransformMotion<Type_fr_trunk_X_fr_RR_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_RR_q2_joint();
        const Type_fr_trunk_X_fr_RR_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RR_q3_joint : public TransformMotion<Type_fr_trunk_X_fr_RR_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_RR_q3_joint();
        const Type_fr_trunk_X_fr_RR_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_hipassembly_X_fr_trunk : public TransformMotion<Type_fr_LF_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_LF_hipassembly_X_fr_trunk();
        const Type_fr_LF_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_hipassembly : public TransformMotion<Type_fr_trunk_X_fr_LF_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_LF_hipassembly();
        const Type_fr_trunk_X_fr_LF_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_LF_hipassembly : public TransformMotion<Type_fr_LF_upperleg_X_fr_LF_hipassembly>
    {
    public:
        Type_fr_LF_upperleg_X_fr_LF_hipassembly();
        const Type_fr_LF_upperleg_X_fr_LF_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_hipassembly_X_fr_LF_upperleg : public TransformMotion<Type_fr_LF_hipassembly_X_fr_LF_upperleg>
    {
    public:
        Type_fr_LF_hipassembly_X_fr_LF_upperleg();
        const Type_fr_LF_hipassembly_X_fr_LF_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_lowerleg_X_fr_LF_upperleg : public TransformMotion<Type_fr_LF_lowerleg_X_fr_LF_upperleg>
    {
    public:
        Type_fr_LF_lowerleg_X_fr_LF_upperleg();
        const Type_fr_LF_lowerleg_X_fr_LF_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_LF_lowerleg : public TransformMotion<Type_fr_LF_upperleg_X_fr_LF_lowerleg>
    {
    public:
        Type_fr_LF_upperleg_X_fr_LF_lowerleg();
        const Type_fr_LF_upperleg_X_fr_LF_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_hipassembly_X_fr_trunk : public TransformMotion<Type_fr_LM_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_LM_hipassembly_X_fr_trunk();
        const Type_fr_LM_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LM_hipassembly : public TransformMotion<Type_fr_trunk_X_fr_LM_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_LM_hipassembly();
        const Type_fr_trunk_X_fr_LM_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_upperleg_X_fr_LM_hipassembly : public TransformMotion<Type_fr_LM_upperleg_X_fr_LM_hipassembly>
    {
    public:
        Type_fr_LM_upperleg_X_fr_LM_hipassembly();
        const Type_fr_LM_upperleg_X_fr_LM_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_hipassembly_X_fr_LM_upperleg : public TransformMotion<Type_fr_LM_hipassembly_X_fr_LM_upperleg>
    {
    public:
        Type_fr_LM_hipassembly_X_fr_LM_upperleg();
        const Type_fr_LM_hipassembly_X_fr_LM_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_lowerleg_X_fr_LM_upperleg : public TransformMotion<Type_fr_LM_lowerleg_X_fr_LM_upperleg>
    {
    public:
        Type_fr_LM_lowerleg_X_fr_LM_upperleg();
        const Type_fr_LM_lowerleg_X_fr_LM_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_upperleg_X_fr_LM_lowerleg : public TransformMotion<Type_fr_LM_upperleg_X_fr_LM_lowerleg>
    {
    public:
        Type_fr_LM_upperleg_X_fr_LM_lowerleg();
        const Type_fr_LM_upperleg_X_fr_LM_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_hipassembly_X_fr_trunk : public TransformMotion<Type_fr_LR_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_LR_hipassembly_X_fr_trunk();
        const Type_fr_LR_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LR_hipassembly : public TransformMotion<Type_fr_trunk_X_fr_LR_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_LR_hipassembly();
        const Type_fr_trunk_X_fr_LR_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_upperleg_X_fr_LR_hipassembly : public TransformMotion<Type_fr_LR_upperleg_X_fr_LR_hipassembly>
    {
    public:
        Type_fr_LR_upperleg_X_fr_LR_hipassembly();
        const Type_fr_LR_upperleg_X_fr_LR_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_hipassembly_X_fr_LR_upperleg : public TransformMotion<Type_fr_LR_hipassembly_X_fr_LR_upperleg>
    {
    public:
        Type_fr_LR_hipassembly_X_fr_LR_upperleg();
        const Type_fr_LR_hipassembly_X_fr_LR_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_lowerleg_X_fr_LR_upperleg : public TransformMotion<Type_fr_LR_lowerleg_X_fr_LR_upperleg>
    {
    public:
        Type_fr_LR_lowerleg_X_fr_LR_upperleg();
        const Type_fr_LR_lowerleg_X_fr_LR_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_upperleg_X_fr_LR_lowerleg : public TransformMotion<Type_fr_LR_upperleg_X_fr_LR_lowerleg>
    {
    public:
        Type_fr_LR_upperleg_X_fr_LR_lowerleg();
        const Type_fr_LR_upperleg_X_fr_LR_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_hipassembly_X_fr_trunk : public TransformMotion<Type_fr_RF_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_RF_hipassembly_X_fr_trunk();
        const Type_fr_RF_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_hipassembly : public TransformMotion<Type_fr_trunk_X_fr_RF_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_RF_hipassembly();
        const Type_fr_trunk_X_fr_RF_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_RF_hipassembly : public TransformMotion<Type_fr_RF_upperleg_X_fr_RF_hipassembly>
    {
    public:
        Type_fr_RF_upperleg_X_fr_RF_hipassembly();
        const Type_fr_RF_upperleg_X_fr_RF_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_hipassembly_X_fr_RF_upperleg : public TransformMotion<Type_fr_RF_hipassembly_X_fr_RF_upperleg>
    {
    public:
        Type_fr_RF_hipassembly_X_fr_RF_upperleg();
        const Type_fr_RF_hipassembly_X_fr_RF_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_lowerleg_X_fr_RF_upperleg : public TransformMotion<Type_fr_RF_lowerleg_X_fr_RF_upperleg>
    {
    public:
        Type_fr_RF_lowerleg_X_fr_RF_upperleg();
        const Type_fr_RF_lowerleg_X_fr_RF_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_RF_lowerleg : public TransformMotion<Type_fr_RF_upperleg_X_fr_RF_lowerleg>
    {
    public:
        Type_fr_RF_upperleg_X_fr_RF_lowerleg();
        const Type_fr_RF_upperleg_X_fr_RF_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_hipassembly_X_fr_trunk : public TransformMotion<Type_fr_RM_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_RM_hipassembly_X_fr_trunk();
        const Type_fr_RM_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RM_hipassembly : public TransformMotion<Type_fr_trunk_X_fr_RM_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_RM_hipassembly();
        const Type_fr_trunk_X_fr_RM_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_upperleg_X_fr_RM_hipassembly : public TransformMotion<Type_fr_RM_upperleg_X_fr_RM_hipassembly>
    {
    public:
        Type_fr_RM_upperleg_X_fr_RM_hipassembly();
        const Type_fr_RM_upperleg_X_fr_RM_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_hipassembly_X_fr_RM_upperleg : public TransformMotion<Type_fr_RM_hipassembly_X_fr_RM_upperleg>
    {
    public:
        Type_fr_RM_hipassembly_X_fr_RM_upperleg();
        const Type_fr_RM_hipassembly_X_fr_RM_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_lowerleg_X_fr_RM_upperleg : public TransformMotion<Type_fr_RM_lowerleg_X_fr_RM_upperleg>
    {
    public:
        Type_fr_RM_lowerleg_X_fr_RM_upperleg();
        const Type_fr_RM_lowerleg_X_fr_RM_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_upperleg_X_fr_RM_lowerleg : public TransformMotion<Type_fr_RM_upperleg_X_fr_RM_lowerleg>
    {
    public:
        Type_fr_RM_upperleg_X_fr_RM_lowerleg();
        const Type_fr_RM_upperleg_X_fr_RM_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_hipassembly_X_fr_trunk : public TransformMotion<Type_fr_RR_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_RR_hipassembly_X_fr_trunk();
        const Type_fr_RR_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RR_hipassembly : public TransformMotion<Type_fr_trunk_X_fr_RR_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_RR_hipassembly();
        const Type_fr_trunk_X_fr_RR_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_upperleg_X_fr_RR_hipassembly : public TransformMotion<Type_fr_RR_upperleg_X_fr_RR_hipassembly>
    {
    public:
        Type_fr_RR_upperleg_X_fr_RR_hipassembly();
        const Type_fr_RR_upperleg_X_fr_RR_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_hipassembly_X_fr_RR_upperleg : public TransformMotion<Type_fr_RR_hipassembly_X_fr_RR_upperleg>
    {
    public:
        Type_fr_RR_hipassembly_X_fr_RR_upperleg();
        const Type_fr_RR_hipassembly_X_fr_RR_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_lowerleg_X_fr_RR_upperleg : public TransformMotion<Type_fr_RR_lowerleg_X_fr_RR_upperleg>
    {
    public:
        Type_fr_RR_lowerleg_X_fr_RR_upperleg();
        const Type_fr_RR_lowerleg_X_fr_RR_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_upperleg_X_fr_RR_lowerleg : public TransformMotion<Type_fr_RR_upperleg_X_fr_RR_lowerleg>
    {
    public:
        Type_fr_RR_upperleg_X_fr_RR_lowerleg();
        const Type_fr_RR_upperleg_X_fr_RR_lowerleg& update(const state_t&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_trunk_X_LF_hipassemblyCOM fr_trunk_X_LF_hipassemblyCOM;
    Type_fr_trunk_X_LF_upperlegCOM fr_trunk_X_LF_upperlegCOM;
    Type_fr_trunk_X_LF_lowerlegCOM fr_trunk_X_LF_lowerlegCOM;
    Type_fr_trunk_X_LM_hipassemblyCOM fr_trunk_X_LM_hipassemblyCOM;
    Type_fr_trunk_X_LM_upperlegCOM fr_trunk_X_LM_upperlegCOM;
    Type_fr_trunk_X_LM_lowerlegCOM fr_trunk_X_LM_lowerlegCOM;
    Type_fr_trunk_X_LR_hipassemblyCOM fr_trunk_X_LR_hipassemblyCOM;
    Type_fr_trunk_X_LR_upperlegCOM fr_trunk_X_LR_upperlegCOM;
    Type_fr_trunk_X_LR_lowerlegCOM fr_trunk_X_LR_lowerlegCOM;
    Type_fr_trunk_X_RF_hipassemblyCOM fr_trunk_X_RF_hipassemblyCOM;
    Type_fr_trunk_X_RF_upperlegCOM fr_trunk_X_RF_upperlegCOM;
    Type_fr_trunk_X_RF_lowerlegCOM fr_trunk_X_RF_lowerlegCOM;
    Type_fr_trunk_X_RM_hipassemblyCOM fr_trunk_X_RM_hipassemblyCOM;
    Type_fr_trunk_X_RM_upperlegCOM fr_trunk_X_RM_upperlegCOM;
    Type_fr_trunk_X_RM_lowerlegCOM fr_trunk_X_RM_lowerlegCOM;
    Type_fr_trunk_X_RR_hipassemblyCOM fr_trunk_X_RR_hipassemblyCOM;
    Type_fr_trunk_X_RR_upperlegCOM fr_trunk_X_RR_upperlegCOM;
    Type_fr_trunk_X_RR_lowerlegCOM fr_trunk_X_RR_lowerlegCOM;
    Type_fr_LF_upperleg_X_fr_trunk fr_LF_upperleg_X_fr_trunk;
    Type_fr_LF_lowerleg_X_fr_trunk fr_LF_lowerleg_X_fr_trunk;
    Type_fr_LM_upperleg_X_fr_trunk fr_LM_upperleg_X_fr_trunk;
    Type_fr_LM_lowerleg_X_fr_trunk fr_LM_lowerleg_X_fr_trunk;
    Type_fr_LR_upperleg_X_fr_trunk fr_LR_upperleg_X_fr_trunk;
    Type_fr_LR_lowerleg_X_fr_trunk fr_LR_lowerleg_X_fr_trunk;
    Type_fr_RF_upperleg_X_fr_trunk fr_RF_upperleg_X_fr_trunk;
    Type_fr_RF_lowerleg_X_fr_trunk fr_RF_lowerleg_X_fr_trunk;
    Type_fr_RM_upperleg_X_fr_trunk fr_RM_upperleg_X_fr_trunk;
    Type_fr_RM_lowerleg_X_fr_trunk fr_RM_lowerleg_X_fr_trunk;
    Type_fr_RR_upperleg_X_fr_trunk fr_RR_upperleg_X_fr_trunk;
    Type_fr_RR_lowerleg_X_fr_trunk fr_RR_lowerleg_X_fr_trunk;
    Type_fr_trunk_X_LF_foot fr_trunk_X_LF_foot;
    Type_fr_trunk_X_fr_LF_q1_joint fr_trunk_X_fr_LF_q1_joint;
    Type_fr_trunk_X_fr_LF_q2_joint fr_trunk_X_fr_LF_q2_joint;
    Type_fr_trunk_X_fr_LF_q3_joint fr_trunk_X_fr_LF_q3_joint;
    Type_fr_trunk_X_LM_foot fr_trunk_X_LM_foot;
    Type_fr_trunk_X_fr_LM_q1_joint fr_trunk_X_fr_LM_q1_joint;
    Type_fr_trunk_X_fr_LM_q2_joint fr_trunk_X_fr_LM_q2_joint;
    Type_fr_trunk_X_fr_LM_q3_joint fr_trunk_X_fr_LM_q3_joint;
    Type_fr_trunk_X_LR_foot fr_trunk_X_LR_foot;
    Type_fr_trunk_X_fr_LR_q1_joint fr_trunk_X_fr_LR_q1_joint;
    Type_fr_trunk_X_fr_LR_q2_joint fr_trunk_X_fr_LR_q2_joint;
    Type_fr_trunk_X_fr_LR_q3_joint fr_trunk_X_fr_LR_q3_joint;
    Type_fr_trunk_X_RF_foot fr_trunk_X_RF_foot;
    Type_fr_trunk_X_fr_RF_q1_joint fr_trunk_X_fr_RF_q1_joint;
    Type_fr_trunk_X_fr_RF_q2_joint fr_trunk_X_fr_RF_q2_joint;
    Type_fr_trunk_X_fr_RF_q3_joint fr_trunk_X_fr_RF_q3_joint;
    Type_fr_trunk_X_RM_foot fr_trunk_X_RM_foot;
    Type_fr_trunk_X_fr_RM_q1_joint fr_trunk_X_fr_RM_q1_joint;
    Type_fr_trunk_X_fr_RM_q2_joint fr_trunk_X_fr_RM_q2_joint;
    Type_fr_trunk_X_fr_RM_q3_joint fr_trunk_X_fr_RM_q3_joint;
    Type_fr_trunk_X_RR_foot fr_trunk_X_RR_foot;
    Type_fr_trunk_X_fr_RR_q1_joint fr_trunk_X_fr_RR_q1_joint;
    Type_fr_trunk_X_fr_RR_q2_joint fr_trunk_X_fr_RR_q2_joint;
    Type_fr_trunk_X_fr_RR_q3_joint fr_trunk_X_fr_RR_q3_joint;
    Type_fr_LF_hipassembly_X_fr_trunk fr_LF_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_LF_hipassembly fr_trunk_X_fr_LF_hipassembly;
    Type_fr_LF_upperleg_X_fr_LF_hipassembly fr_LF_upperleg_X_fr_LF_hipassembly;
    Type_fr_LF_hipassembly_X_fr_LF_upperleg fr_LF_hipassembly_X_fr_LF_upperleg;
    Type_fr_LF_lowerleg_X_fr_LF_upperleg fr_LF_lowerleg_X_fr_LF_upperleg;
    Type_fr_LF_upperleg_X_fr_LF_lowerleg fr_LF_upperleg_X_fr_LF_lowerleg;
    Type_fr_LM_hipassembly_X_fr_trunk fr_LM_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_LM_hipassembly fr_trunk_X_fr_LM_hipassembly;
    Type_fr_LM_upperleg_X_fr_LM_hipassembly fr_LM_upperleg_X_fr_LM_hipassembly;
    Type_fr_LM_hipassembly_X_fr_LM_upperleg fr_LM_hipassembly_X_fr_LM_upperleg;
    Type_fr_LM_lowerleg_X_fr_LM_upperleg fr_LM_lowerleg_X_fr_LM_upperleg;
    Type_fr_LM_upperleg_X_fr_LM_lowerleg fr_LM_upperleg_X_fr_LM_lowerleg;
    Type_fr_LR_hipassembly_X_fr_trunk fr_LR_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_LR_hipassembly fr_trunk_X_fr_LR_hipassembly;
    Type_fr_LR_upperleg_X_fr_LR_hipassembly fr_LR_upperleg_X_fr_LR_hipassembly;
    Type_fr_LR_hipassembly_X_fr_LR_upperleg fr_LR_hipassembly_X_fr_LR_upperleg;
    Type_fr_LR_lowerleg_X_fr_LR_upperleg fr_LR_lowerleg_X_fr_LR_upperleg;
    Type_fr_LR_upperleg_X_fr_LR_lowerleg fr_LR_upperleg_X_fr_LR_lowerleg;
    Type_fr_RF_hipassembly_X_fr_trunk fr_RF_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_RF_hipassembly fr_trunk_X_fr_RF_hipassembly;
    Type_fr_RF_upperleg_X_fr_RF_hipassembly fr_RF_upperleg_X_fr_RF_hipassembly;
    Type_fr_RF_hipassembly_X_fr_RF_upperleg fr_RF_hipassembly_X_fr_RF_upperleg;
    Type_fr_RF_lowerleg_X_fr_RF_upperleg fr_RF_lowerleg_X_fr_RF_upperleg;
    Type_fr_RF_upperleg_X_fr_RF_lowerleg fr_RF_upperleg_X_fr_RF_lowerleg;
    Type_fr_RM_hipassembly_X_fr_trunk fr_RM_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_RM_hipassembly fr_trunk_X_fr_RM_hipassembly;
    Type_fr_RM_upperleg_X_fr_RM_hipassembly fr_RM_upperleg_X_fr_RM_hipassembly;
    Type_fr_RM_hipassembly_X_fr_RM_upperleg fr_RM_hipassembly_X_fr_RM_upperleg;
    Type_fr_RM_lowerleg_X_fr_RM_upperleg fr_RM_lowerleg_X_fr_RM_upperleg;
    Type_fr_RM_upperleg_X_fr_RM_lowerleg fr_RM_upperleg_X_fr_RM_lowerleg;
    Type_fr_RR_hipassembly_X_fr_trunk fr_RR_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_RR_hipassembly fr_trunk_X_fr_RR_hipassembly;
    Type_fr_RR_upperleg_X_fr_RR_hipassembly fr_RR_upperleg_X_fr_RR_hipassembly;
    Type_fr_RR_hipassembly_X_fr_RR_upperleg fr_RR_hipassembly_X_fr_RR_upperleg;
    Type_fr_RR_lowerleg_X_fr_RR_upperleg fr_RR_lowerleg_X_fr_RR_upperleg;
    Type_fr_RR_upperleg_X_fr_RR_lowerleg fr_RR_upperleg_X_fr_RR_lowerleg;

protected:

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
class ForceTransforms {
public:
    class Dummy {};
    typedef TransformForce<Dummy>::MatrixType MatrixType;
public:
    class Type_fr_trunk_X_LF_hipassemblyCOM : public TransformForce<Type_fr_trunk_X_LF_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_LF_hipassemblyCOM();
        const Type_fr_trunk_X_LF_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_upperlegCOM : public TransformForce<Type_fr_trunk_X_LF_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_LF_upperlegCOM();
        const Type_fr_trunk_X_LF_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_lowerlegCOM : public TransformForce<Type_fr_trunk_X_LF_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_LF_lowerlegCOM();
        const Type_fr_trunk_X_LF_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LM_hipassemblyCOM : public TransformForce<Type_fr_trunk_X_LM_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_LM_hipassemblyCOM();
        const Type_fr_trunk_X_LM_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LM_upperlegCOM : public TransformForce<Type_fr_trunk_X_LM_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_LM_upperlegCOM();
        const Type_fr_trunk_X_LM_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LM_lowerlegCOM : public TransformForce<Type_fr_trunk_X_LM_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_LM_lowerlegCOM();
        const Type_fr_trunk_X_LM_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LR_hipassemblyCOM : public TransformForce<Type_fr_trunk_X_LR_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_LR_hipassemblyCOM();
        const Type_fr_trunk_X_LR_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LR_upperlegCOM : public TransformForce<Type_fr_trunk_X_LR_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_LR_upperlegCOM();
        const Type_fr_trunk_X_LR_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LR_lowerlegCOM : public TransformForce<Type_fr_trunk_X_LR_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_LR_lowerlegCOM();
        const Type_fr_trunk_X_LR_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_hipassemblyCOM : public TransformForce<Type_fr_trunk_X_RF_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_RF_hipassemblyCOM();
        const Type_fr_trunk_X_RF_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_upperlegCOM : public TransformForce<Type_fr_trunk_X_RF_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_RF_upperlegCOM();
        const Type_fr_trunk_X_RF_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_lowerlegCOM : public TransformForce<Type_fr_trunk_X_RF_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_RF_lowerlegCOM();
        const Type_fr_trunk_X_RF_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RM_hipassemblyCOM : public TransformForce<Type_fr_trunk_X_RM_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_RM_hipassemblyCOM();
        const Type_fr_trunk_X_RM_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RM_upperlegCOM : public TransformForce<Type_fr_trunk_X_RM_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_RM_upperlegCOM();
        const Type_fr_trunk_X_RM_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RM_lowerlegCOM : public TransformForce<Type_fr_trunk_X_RM_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_RM_lowerlegCOM();
        const Type_fr_trunk_X_RM_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RR_hipassemblyCOM : public TransformForce<Type_fr_trunk_X_RR_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_RR_hipassemblyCOM();
        const Type_fr_trunk_X_RR_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RR_upperlegCOM : public TransformForce<Type_fr_trunk_X_RR_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_RR_upperlegCOM();
        const Type_fr_trunk_X_RR_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RR_lowerlegCOM : public TransformForce<Type_fr_trunk_X_RR_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_RR_lowerlegCOM();
        const Type_fr_trunk_X_RR_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_trunk : public TransformForce<Type_fr_LF_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_LF_upperleg_X_fr_trunk();
        const Type_fr_LF_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_lowerleg_X_fr_trunk : public TransformForce<Type_fr_LF_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_LF_lowerleg_X_fr_trunk();
        const Type_fr_LF_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_upperleg_X_fr_trunk : public TransformForce<Type_fr_LM_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_LM_upperleg_X_fr_trunk();
        const Type_fr_LM_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_lowerleg_X_fr_trunk : public TransformForce<Type_fr_LM_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_LM_lowerleg_X_fr_trunk();
        const Type_fr_LM_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_upperleg_X_fr_trunk : public TransformForce<Type_fr_LR_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_LR_upperleg_X_fr_trunk();
        const Type_fr_LR_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_lowerleg_X_fr_trunk : public TransformForce<Type_fr_LR_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_LR_lowerleg_X_fr_trunk();
        const Type_fr_LR_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_trunk : public TransformForce<Type_fr_RF_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_RF_upperleg_X_fr_trunk();
        const Type_fr_RF_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_lowerleg_X_fr_trunk : public TransformForce<Type_fr_RF_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_RF_lowerleg_X_fr_trunk();
        const Type_fr_RF_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_upperleg_X_fr_trunk : public TransformForce<Type_fr_RM_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_RM_upperleg_X_fr_trunk();
        const Type_fr_RM_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_lowerleg_X_fr_trunk : public TransformForce<Type_fr_RM_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_RM_lowerleg_X_fr_trunk();
        const Type_fr_RM_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_upperleg_X_fr_trunk : public TransformForce<Type_fr_RR_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_RR_upperleg_X_fr_trunk();
        const Type_fr_RR_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_lowerleg_X_fr_trunk : public TransformForce<Type_fr_RR_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_RR_lowerleg_X_fr_trunk();
        const Type_fr_RR_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_foot : public TransformForce<Type_fr_trunk_X_LF_foot>
    {
    public:
        Type_fr_trunk_X_LF_foot();
        const Type_fr_trunk_X_LF_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_q1_joint : public TransformForce<Type_fr_trunk_X_fr_LF_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_LF_q1_joint();
        const Type_fr_trunk_X_fr_LF_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_q2_joint : public TransformForce<Type_fr_trunk_X_fr_LF_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_LF_q2_joint();
        const Type_fr_trunk_X_fr_LF_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_q3_joint : public TransformForce<Type_fr_trunk_X_fr_LF_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_LF_q3_joint();
        const Type_fr_trunk_X_fr_LF_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LM_foot : public TransformForce<Type_fr_trunk_X_LM_foot>
    {
    public:
        Type_fr_trunk_X_LM_foot();
        const Type_fr_trunk_X_LM_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LM_q1_joint : public TransformForce<Type_fr_trunk_X_fr_LM_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_LM_q1_joint();
        const Type_fr_trunk_X_fr_LM_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LM_q2_joint : public TransformForce<Type_fr_trunk_X_fr_LM_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_LM_q2_joint();
        const Type_fr_trunk_X_fr_LM_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LM_q3_joint : public TransformForce<Type_fr_trunk_X_fr_LM_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_LM_q3_joint();
        const Type_fr_trunk_X_fr_LM_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LR_foot : public TransformForce<Type_fr_trunk_X_LR_foot>
    {
    public:
        Type_fr_trunk_X_LR_foot();
        const Type_fr_trunk_X_LR_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LR_q1_joint : public TransformForce<Type_fr_trunk_X_fr_LR_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_LR_q1_joint();
        const Type_fr_trunk_X_fr_LR_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LR_q2_joint : public TransformForce<Type_fr_trunk_X_fr_LR_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_LR_q2_joint();
        const Type_fr_trunk_X_fr_LR_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LR_q3_joint : public TransformForce<Type_fr_trunk_X_fr_LR_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_LR_q3_joint();
        const Type_fr_trunk_X_fr_LR_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_foot : public TransformForce<Type_fr_trunk_X_RF_foot>
    {
    public:
        Type_fr_trunk_X_RF_foot();
        const Type_fr_trunk_X_RF_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_q1_joint : public TransformForce<Type_fr_trunk_X_fr_RF_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_RF_q1_joint();
        const Type_fr_trunk_X_fr_RF_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_q2_joint : public TransformForce<Type_fr_trunk_X_fr_RF_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_RF_q2_joint();
        const Type_fr_trunk_X_fr_RF_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_q3_joint : public TransformForce<Type_fr_trunk_X_fr_RF_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_RF_q3_joint();
        const Type_fr_trunk_X_fr_RF_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RM_foot : public TransformForce<Type_fr_trunk_X_RM_foot>
    {
    public:
        Type_fr_trunk_X_RM_foot();
        const Type_fr_trunk_X_RM_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RM_q1_joint : public TransformForce<Type_fr_trunk_X_fr_RM_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_RM_q1_joint();
        const Type_fr_trunk_X_fr_RM_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RM_q2_joint : public TransformForce<Type_fr_trunk_X_fr_RM_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_RM_q2_joint();
        const Type_fr_trunk_X_fr_RM_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RM_q3_joint : public TransformForce<Type_fr_trunk_X_fr_RM_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_RM_q3_joint();
        const Type_fr_trunk_X_fr_RM_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RR_foot : public TransformForce<Type_fr_trunk_X_RR_foot>
    {
    public:
        Type_fr_trunk_X_RR_foot();
        const Type_fr_trunk_X_RR_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RR_q1_joint : public TransformForce<Type_fr_trunk_X_fr_RR_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_RR_q1_joint();
        const Type_fr_trunk_X_fr_RR_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RR_q2_joint : public TransformForce<Type_fr_trunk_X_fr_RR_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_RR_q2_joint();
        const Type_fr_trunk_X_fr_RR_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RR_q3_joint : public TransformForce<Type_fr_trunk_X_fr_RR_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_RR_q3_joint();
        const Type_fr_trunk_X_fr_RR_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_hipassembly_X_fr_trunk : public TransformForce<Type_fr_LF_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_LF_hipassembly_X_fr_trunk();
        const Type_fr_LF_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_hipassembly : public TransformForce<Type_fr_trunk_X_fr_LF_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_LF_hipassembly();
        const Type_fr_trunk_X_fr_LF_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_LF_hipassembly : public TransformForce<Type_fr_LF_upperleg_X_fr_LF_hipassembly>
    {
    public:
        Type_fr_LF_upperleg_X_fr_LF_hipassembly();
        const Type_fr_LF_upperleg_X_fr_LF_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_hipassembly_X_fr_LF_upperleg : public TransformForce<Type_fr_LF_hipassembly_X_fr_LF_upperleg>
    {
    public:
        Type_fr_LF_hipassembly_X_fr_LF_upperleg();
        const Type_fr_LF_hipassembly_X_fr_LF_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_lowerleg_X_fr_LF_upperleg : public TransformForce<Type_fr_LF_lowerleg_X_fr_LF_upperleg>
    {
    public:
        Type_fr_LF_lowerleg_X_fr_LF_upperleg();
        const Type_fr_LF_lowerleg_X_fr_LF_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_LF_lowerleg : public TransformForce<Type_fr_LF_upperleg_X_fr_LF_lowerleg>
    {
    public:
        Type_fr_LF_upperleg_X_fr_LF_lowerleg();
        const Type_fr_LF_upperleg_X_fr_LF_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_hipassembly_X_fr_trunk : public TransformForce<Type_fr_LM_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_LM_hipassembly_X_fr_trunk();
        const Type_fr_LM_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LM_hipassembly : public TransformForce<Type_fr_trunk_X_fr_LM_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_LM_hipassembly();
        const Type_fr_trunk_X_fr_LM_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_upperleg_X_fr_LM_hipassembly : public TransformForce<Type_fr_LM_upperleg_X_fr_LM_hipassembly>
    {
    public:
        Type_fr_LM_upperleg_X_fr_LM_hipassembly();
        const Type_fr_LM_upperleg_X_fr_LM_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_hipassembly_X_fr_LM_upperleg : public TransformForce<Type_fr_LM_hipassembly_X_fr_LM_upperleg>
    {
    public:
        Type_fr_LM_hipassembly_X_fr_LM_upperleg();
        const Type_fr_LM_hipassembly_X_fr_LM_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_lowerleg_X_fr_LM_upperleg : public TransformForce<Type_fr_LM_lowerleg_X_fr_LM_upperleg>
    {
    public:
        Type_fr_LM_lowerleg_X_fr_LM_upperleg();
        const Type_fr_LM_lowerleg_X_fr_LM_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_upperleg_X_fr_LM_lowerleg : public TransformForce<Type_fr_LM_upperleg_X_fr_LM_lowerleg>
    {
    public:
        Type_fr_LM_upperleg_X_fr_LM_lowerleg();
        const Type_fr_LM_upperleg_X_fr_LM_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_hipassembly_X_fr_trunk : public TransformForce<Type_fr_LR_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_LR_hipassembly_X_fr_trunk();
        const Type_fr_LR_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LR_hipassembly : public TransformForce<Type_fr_trunk_X_fr_LR_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_LR_hipassembly();
        const Type_fr_trunk_X_fr_LR_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_upperleg_X_fr_LR_hipassembly : public TransformForce<Type_fr_LR_upperleg_X_fr_LR_hipassembly>
    {
    public:
        Type_fr_LR_upperleg_X_fr_LR_hipassembly();
        const Type_fr_LR_upperleg_X_fr_LR_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_hipassembly_X_fr_LR_upperleg : public TransformForce<Type_fr_LR_hipassembly_X_fr_LR_upperleg>
    {
    public:
        Type_fr_LR_hipassembly_X_fr_LR_upperleg();
        const Type_fr_LR_hipassembly_X_fr_LR_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_lowerleg_X_fr_LR_upperleg : public TransformForce<Type_fr_LR_lowerleg_X_fr_LR_upperleg>
    {
    public:
        Type_fr_LR_lowerleg_X_fr_LR_upperleg();
        const Type_fr_LR_lowerleg_X_fr_LR_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_upperleg_X_fr_LR_lowerleg : public TransformForce<Type_fr_LR_upperleg_X_fr_LR_lowerleg>
    {
    public:
        Type_fr_LR_upperleg_X_fr_LR_lowerleg();
        const Type_fr_LR_upperleg_X_fr_LR_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_hipassembly_X_fr_trunk : public TransformForce<Type_fr_RF_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_RF_hipassembly_X_fr_trunk();
        const Type_fr_RF_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_hipassembly : public TransformForce<Type_fr_trunk_X_fr_RF_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_RF_hipassembly();
        const Type_fr_trunk_X_fr_RF_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_RF_hipassembly : public TransformForce<Type_fr_RF_upperleg_X_fr_RF_hipassembly>
    {
    public:
        Type_fr_RF_upperleg_X_fr_RF_hipassembly();
        const Type_fr_RF_upperleg_X_fr_RF_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_hipassembly_X_fr_RF_upperleg : public TransformForce<Type_fr_RF_hipassembly_X_fr_RF_upperleg>
    {
    public:
        Type_fr_RF_hipassembly_X_fr_RF_upperleg();
        const Type_fr_RF_hipassembly_X_fr_RF_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_lowerleg_X_fr_RF_upperleg : public TransformForce<Type_fr_RF_lowerleg_X_fr_RF_upperleg>
    {
    public:
        Type_fr_RF_lowerleg_X_fr_RF_upperleg();
        const Type_fr_RF_lowerleg_X_fr_RF_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_RF_lowerleg : public TransformForce<Type_fr_RF_upperleg_X_fr_RF_lowerleg>
    {
    public:
        Type_fr_RF_upperleg_X_fr_RF_lowerleg();
        const Type_fr_RF_upperleg_X_fr_RF_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_hipassembly_X_fr_trunk : public TransformForce<Type_fr_RM_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_RM_hipassembly_X_fr_trunk();
        const Type_fr_RM_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RM_hipassembly : public TransformForce<Type_fr_trunk_X_fr_RM_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_RM_hipassembly();
        const Type_fr_trunk_X_fr_RM_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_upperleg_X_fr_RM_hipassembly : public TransformForce<Type_fr_RM_upperleg_X_fr_RM_hipassembly>
    {
    public:
        Type_fr_RM_upperleg_X_fr_RM_hipassembly();
        const Type_fr_RM_upperleg_X_fr_RM_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_hipassembly_X_fr_RM_upperleg : public TransformForce<Type_fr_RM_hipassembly_X_fr_RM_upperleg>
    {
    public:
        Type_fr_RM_hipassembly_X_fr_RM_upperleg();
        const Type_fr_RM_hipassembly_X_fr_RM_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_lowerleg_X_fr_RM_upperleg : public TransformForce<Type_fr_RM_lowerleg_X_fr_RM_upperleg>
    {
    public:
        Type_fr_RM_lowerleg_X_fr_RM_upperleg();
        const Type_fr_RM_lowerleg_X_fr_RM_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_upperleg_X_fr_RM_lowerleg : public TransformForce<Type_fr_RM_upperleg_X_fr_RM_lowerleg>
    {
    public:
        Type_fr_RM_upperleg_X_fr_RM_lowerleg();
        const Type_fr_RM_upperleg_X_fr_RM_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_hipassembly_X_fr_trunk : public TransformForce<Type_fr_RR_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_RR_hipassembly_X_fr_trunk();
        const Type_fr_RR_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RR_hipassembly : public TransformForce<Type_fr_trunk_X_fr_RR_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_RR_hipassembly();
        const Type_fr_trunk_X_fr_RR_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_upperleg_X_fr_RR_hipassembly : public TransformForce<Type_fr_RR_upperleg_X_fr_RR_hipassembly>
    {
    public:
        Type_fr_RR_upperleg_X_fr_RR_hipassembly();
        const Type_fr_RR_upperleg_X_fr_RR_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_hipassembly_X_fr_RR_upperleg : public TransformForce<Type_fr_RR_hipassembly_X_fr_RR_upperleg>
    {
    public:
        Type_fr_RR_hipassembly_X_fr_RR_upperleg();
        const Type_fr_RR_hipassembly_X_fr_RR_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_lowerleg_X_fr_RR_upperleg : public TransformForce<Type_fr_RR_lowerleg_X_fr_RR_upperleg>
    {
    public:
        Type_fr_RR_lowerleg_X_fr_RR_upperleg();
        const Type_fr_RR_lowerleg_X_fr_RR_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_upperleg_X_fr_RR_lowerleg : public TransformForce<Type_fr_RR_upperleg_X_fr_RR_lowerleg>
    {
    public:
        Type_fr_RR_upperleg_X_fr_RR_lowerleg();
        const Type_fr_RR_upperleg_X_fr_RR_lowerleg& update(const state_t&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_trunk_X_LF_hipassemblyCOM fr_trunk_X_LF_hipassemblyCOM;
    Type_fr_trunk_X_LF_upperlegCOM fr_trunk_X_LF_upperlegCOM;
    Type_fr_trunk_X_LF_lowerlegCOM fr_trunk_X_LF_lowerlegCOM;
    Type_fr_trunk_X_LM_hipassemblyCOM fr_trunk_X_LM_hipassemblyCOM;
    Type_fr_trunk_X_LM_upperlegCOM fr_trunk_X_LM_upperlegCOM;
    Type_fr_trunk_X_LM_lowerlegCOM fr_trunk_X_LM_lowerlegCOM;
    Type_fr_trunk_X_LR_hipassemblyCOM fr_trunk_X_LR_hipassemblyCOM;
    Type_fr_trunk_X_LR_upperlegCOM fr_trunk_X_LR_upperlegCOM;
    Type_fr_trunk_X_LR_lowerlegCOM fr_trunk_X_LR_lowerlegCOM;
    Type_fr_trunk_X_RF_hipassemblyCOM fr_trunk_X_RF_hipassemblyCOM;
    Type_fr_trunk_X_RF_upperlegCOM fr_trunk_X_RF_upperlegCOM;
    Type_fr_trunk_X_RF_lowerlegCOM fr_trunk_X_RF_lowerlegCOM;
    Type_fr_trunk_X_RM_hipassemblyCOM fr_trunk_X_RM_hipassemblyCOM;
    Type_fr_trunk_X_RM_upperlegCOM fr_trunk_X_RM_upperlegCOM;
    Type_fr_trunk_X_RM_lowerlegCOM fr_trunk_X_RM_lowerlegCOM;
    Type_fr_trunk_X_RR_hipassemblyCOM fr_trunk_X_RR_hipassemblyCOM;
    Type_fr_trunk_X_RR_upperlegCOM fr_trunk_X_RR_upperlegCOM;
    Type_fr_trunk_X_RR_lowerlegCOM fr_trunk_X_RR_lowerlegCOM;
    Type_fr_LF_upperleg_X_fr_trunk fr_LF_upperleg_X_fr_trunk;
    Type_fr_LF_lowerleg_X_fr_trunk fr_LF_lowerleg_X_fr_trunk;
    Type_fr_LM_upperleg_X_fr_trunk fr_LM_upperleg_X_fr_trunk;
    Type_fr_LM_lowerleg_X_fr_trunk fr_LM_lowerleg_X_fr_trunk;
    Type_fr_LR_upperleg_X_fr_trunk fr_LR_upperleg_X_fr_trunk;
    Type_fr_LR_lowerleg_X_fr_trunk fr_LR_lowerleg_X_fr_trunk;
    Type_fr_RF_upperleg_X_fr_trunk fr_RF_upperleg_X_fr_trunk;
    Type_fr_RF_lowerleg_X_fr_trunk fr_RF_lowerleg_X_fr_trunk;
    Type_fr_RM_upperleg_X_fr_trunk fr_RM_upperleg_X_fr_trunk;
    Type_fr_RM_lowerleg_X_fr_trunk fr_RM_lowerleg_X_fr_trunk;
    Type_fr_RR_upperleg_X_fr_trunk fr_RR_upperleg_X_fr_trunk;
    Type_fr_RR_lowerleg_X_fr_trunk fr_RR_lowerleg_X_fr_trunk;
    Type_fr_trunk_X_LF_foot fr_trunk_X_LF_foot;
    Type_fr_trunk_X_fr_LF_q1_joint fr_trunk_X_fr_LF_q1_joint;
    Type_fr_trunk_X_fr_LF_q2_joint fr_trunk_X_fr_LF_q2_joint;
    Type_fr_trunk_X_fr_LF_q3_joint fr_trunk_X_fr_LF_q3_joint;
    Type_fr_trunk_X_LM_foot fr_trunk_X_LM_foot;
    Type_fr_trunk_X_fr_LM_q1_joint fr_trunk_X_fr_LM_q1_joint;
    Type_fr_trunk_X_fr_LM_q2_joint fr_trunk_X_fr_LM_q2_joint;
    Type_fr_trunk_X_fr_LM_q3_joint fr_trunk_X_fr_LM_q3_joint;
    Type_fr_trunk_X_LR_foot fr_trunk_X_LR_foot;
    Type_fr_trunk_X_fr_LR_q1_joint fr_trunk_X_fr_LR_q1_joint;
    Type_fr_trunk_X_fr_LR_q2_joint fr_trunk_X_fr_LR_q2_joint;
    Type_fr_trunk_X_fr_LR_q3_joint fr_trunk_X_fr_LR_q3_joint;
    Type_fr_trunk_X_RF_foot fr_trunk_X_RF_foot;
    Type_fr_trunk_X_fr_RF_q1_joint fr_trunk_X_fr_RF_q1_joint;
    Type_fr_trunk_X_fr_RF_q2_joint fr_trunk_X_fr_RF_q2_joint;
    Type_fr_trunk_X_fr_RF_q3_joint fr_trunk_X_fr_RF_q3_joint;
    Type_fr_trunk_X_RM_foot fr_trunk_X_RM_foot;
    Type_fr_trunk_X_fr_RM_q1_joint fr_trunk_X_fr_RM_q1_joint;
    Type_fr_trunk_X_fr_RM_q2_joint fr_trunk_X_fr_RM_q2_joint;
    Type_fr_trunk_X_fr_RM_q3_joint fr_trunk_X_fr_RM_q3_joint;
    Type_fr_trunk_X_RR_foot fr_trunk_X_RR_foot;
    Type_fr_trunk_X_fr_RR_q1_joint fr_trunk_X_fr_RR_q1_joint;
    Type_fr_trunk_X_fr_RR_q2_joint fr_trunk_X_fr_RR_q2_joint;
    Type_fr_trunk_X_fr_RR_q3_joint fr_trunk_X_fr_RR_q3_joint;
    Type_fr_LF_hipassembly_X_fr_trunk fr_LF_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_LF_hipassembly fr_trunk_X_fr_LF_hipassembly;
    Type_fr_LF_upperleg_X_fr_LF_hipassembly fr_LF_upperleg_X_fr_LF_hipassembly;
    Type_fr_LF_hipassembly_X_fr_LF_upperleg fr_LF_hipassembly_X_fr_LF_upperleg;
    Type_fr_LF_lowerleg_X_fr_LF_upperleg fr_LF_lowerleg_X_fr_LF_upperleg;
    Type_fr_LF_upperleg_X_fr_LF_lowerleg fr_LF_upperleg_X_fr_LF_lowerleg;
    Type_fr_LM_hipassembly_X_fr_trunk fr_LM_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_LM_hipassembly fr_trunk_X_fr_LM_hipassembly;
    Type_fr_LM_upperleg_X_fr_LM_hipassembly fr_LM_upperleg_X_fr_LM_hipassembly;
    Type_fr_LM_hipassembly_X_fr_LM_upperleg fr_LM_hipassembly_X_fr_LM_upperleg;
    Type_fr_LM_lowerleg_X_fr_LM_upperleg fr_LM_lowerleg_X_fr_LM_upperleg;
    Type_fr_LM_upperleg_X_fr_LM_lowerleg fr_LM_upperleg_X_fr_LM_lowerleg;
    Type_fr_LR_hipassembly_X_fr_trunk fr_LR_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_LR_hipassembly fr_trunk_X_fr_LR_hipassembly;
    Type_fr_LR_upperleg_X_fr_LR_hipassembly fr_LR_upperleg_X_fr_LR_hipassembly;
    Type_fr_LR_hipassembly_X_fr_LR_upperleg fr_LR_hipassembly_X_fr_LR_upperleg;
    Type_fr_LR_lowerleg_X_fr_LR_upperleg fr_LR_lowerleg_X_fr_LR_upperleg;
    Type_fr_LR_upperleg_X_fr_LR_lowerleg fr_LR_upperleg_X_fr_LR_lowerleg;
    Type_fr_RF_hipassembly_X_fr_trunk fr_RF_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_RF_hipassembly fr_trunk_X_fr_RF_hipassembly;
    Type_fr_RF_upperleg_X_fr_RF_hipassembly fr_RF_upperleg_X_fr_RF_hipassembly;
    Type_fr_RF_hipassembly_X_fr_RF_upperleg fr_RF_hipassembly_X_fr_RF_upperleg;
    Type_fr_RF_lowerleg_X_fr_RF_upperleg fr_RF_lowerleg_X_fr_RF_upperleg;
    Type_fr_RF_upperleg_X_fr_RF_lowerleg fr_RF_upperleg_X_fr_RF_lowerleg;
    Type_fr_RM_hipassembly_X_fr_trunk fr_RM_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_RM_hipassembly fr_trunk_X_fr_RM_hipassembly;
    Type_fr_RM_upperleg_X_fr_RM_hipassembly fr_RM_upperleg_X_fr_RM_hipassembly;
    Type_fr_RM_hipassembly_X_fr_RM_upperleg fr_RM_hipassembly_X_fr_RM_upperleg;
    Type_fr_RM_lowerleg_X_fr_RM_upperleg fr_RM_lowerleg_X_fr_RM_upperleg;
    Type_fr_RM_upperleg_X_fr_RM_lowerleg fr_RM_upperleg_X_fr_RM_lowerleg;
    Type_fr_RR_hipassembly_X_fr_trunk fr_RR_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_RR_hipassembly fr_trunk_X_fr_RR_hipassembly;
    Type_fr_RR_upperleg_X_fr_RR_hipassembly fr_RR_upperleg_X_fr_RR_hipassembly;
    Type_fr_RR_hipassembly_X_fr_RR_upperleg fr_RR_hipassembly_X_fr_RR_upperleg;
    Type_fr_RR_lowerleg_X_fr_RR_upperleg fr_RR_lowerleg_X_fr_RR_upperleg;
    Type_fr_RR_upperleg_X_fr_RR_lowerleg fr_RR_upperleg_X_fr_RR_lowerleg;

protected:

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
class HomogeneousTransforms {
public:
    class Dummy {};
    typedef TransformHomogeneous<Dummy>::MatrixType MatrixType;
public:
    class Type_fr_trunk_X_LF_hipassemblyCOM : public TransformHomogeneous<Type_fr_trunk_X_LF_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_LF_hipassemblyCOM();
        const Type_fr_trunk_X_LF_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_upperlegCOM : public TransformHomogeneous<Type_fr_trunk_X_LF_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_LF_upperlegCOM();
        const Type_fr_trunk_X_LF_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_lowerlegCOM : public TransformHomogeneous<Type_fr_trunk_X_LF_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_LF_lowerlegCOM();
        const Type_fr_trunk_X_LF_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LM_hipassemblyCOM : public TransformHomogeneous<Type_fr_trunk_X_LM_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_LM_hipassemblyCOM();
        const Type_fr_trunk_X_LM_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LM_upperlegCOM : public TransformHomogeneous<Type_fr_trunk_X_LM_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_LM_upperlegCOM();
        const Type_fr_trunk_X_LM_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LM_lowerlegCOM : public TransformHomogeneous<Type_fr_trunk_X_LM_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_LM_lowerlegCOM();
        const Type_fr_trunk_X_LM_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LR_hipassemblyCOM : public TransformHomogeneous<Type_fr_trunk_X_LR_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_LR_hipassemblyCOM();
        const Type_fr_trunk_X_LR_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LR_upperlegCOM : public TransformHomogeneous<Type_fr_trunk_X_LR_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_LR_upperlegCOM();
        const Type_fr_trunk_X_LR_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LR_lowerlegCOM : public TransformHomogeneous<Type_fr_trunk_X_LR_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_LR_lowerlegCOM();
        const Type_fr_trunk_X_LR_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_hipassemblyCOM : public TransformHomogeneous<Type_fr_trunk_X_RF_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_RF_hipassemblyCOM();
        const Type_fr_trunk_X_RF_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_upperlegCOM : public TransformHomogeneous<Type_fr_trunk_X_RF_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_RF_upperlegCOM();
        const Type_fr_trunk_X_RF_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_lowerlegCOM : public TransformHomogeneous<Type_fr_trunk_X_RF_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_RF_lowerlegCOM();
        const Type_fr_trunk_X_RF_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RM_hipassemblyCOM : public TransformHomogeneous<Type_fr_trunk_X_RM_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_RM_hipassemblyCOM();
        const Type_fr_trunk_X_RM_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RM_upperlegCOM : public TransformHomogeneous<Type_fr_trunk_X_RM_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_RM_upperlegCOM();
        const Type_fr_trunk_X_RM_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RM_lowerlegCOM : public TransformHomogeneous<Type_fr_trunk_X_RM_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_RM_lowerlegCOM();
        const Type_fr_trunk_X_RM_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RR_hipassemblyCOM : public TransformHomogeneous<Type_fr_trunk_X_RR_hipassemblyCOM>
    {
    public:
        Type_fr_trunk_X_RR_hipassemblyCOM();
        const Type_fr_trunk_X_RR_hipassemblyCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RR_upperlegCOM : public TransformHomogeneous<Type_fr_trunk_X_RR_upperlegCOM>
    {
    public:
        Type_fr_trunk_X_RR_upperlegCOM();
        const Type_fr_trunk_X_RR_upperlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RR_lowerlegCOM : public TransformHomogeneous<Type_fr_trunk_X_RR_lowerlegCOM>
    {
    public:
        Type_fr_trunk_X_RR_lowerlegCOM();
        const Type_fr_trunk_X_RR_lowerlegCOM& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_trunk : public TransformHomogeneous<Type_fr_LF_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_LF_upperleg_X_fr_trunk();
        const Type_fr_LF_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_lowerleg_X_fr_trunk : public TransformHomogeneous<Type_fr_LF_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_LF_lowerleg_X_fr_trunk();
        const Type_fr_LF_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_upperleg_X_fr_trunk : public TransformHomogeneous<Type_fr_LM_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_LM_upperleg_X_fr_trunk();
        const Type_fr_LM_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_lowerleg_X_fr_trunk : public TransformHomogeneous<Type_fr_LM_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_LM_lowerleg_X_fr_trunk();
        const Type_fr_LM_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_upperleg_X_fr_trunk : public TransformHomogeneous<Type_fr_LR_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_LR_upperleg_X_fr_trunk();
        const Type_fr_LR_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_lowerleg_X_fr_trunk : public TransformHomogeneous<Type_fr_LR_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_LR_lowerleg_X_fr_trunk();
        const Type_fr_LR_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_trunk : public TransformHomogeneous<Type_fr_RF_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_RF_upperleg_X_fr_trunk();
        const Type_fr_RF_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_lowerleg_X_fr_trunk : public TransformHomogeneous<Type_fr_RF_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_RF_lowerleg_X_fr_trunk();
        const Type_fr_RF_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_upperleg_X_fr_trunk : public TransformHomogeneous<Type_fr_RM_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_RM_upperleg_X_fr_trunk();
        const Type_fr_RM_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_lowerleg_X_fr_trunk : public TransformHomogeneous<Type_fr_RM_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_RM_lowerleg_X_fr_trunk();
        const Type_fr_RM_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_upperleg_X_fr_trunk : public TransformHomogeneous<Type_fr_RR_upperleg_X_fr_trunk>
    {
    public:
        Type_fr_RR_upperleg_X_fr_trunk();
        const Type_fr_RR_upperleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_lowerleg_X_fr_trunk : public TransformHomogeneous<Type_fr_RR_lowerleg_X_fr_trunk>
    {
    public:
        Type_fr_RR_lowerleg_X_fr_trunk();
        const Type_fr_RR_lowerleg_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LF_foot : public TransformHomogeneous<Type_fr_trunk_X_LF_foot>
    {
    public:
        Type_fr_trunk_X_LF_foot();
        const Type_fr_trunk_X_LF_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_q1_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_LF_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_LF_q1_joint();
        const Type_fr_trunk_X_fr_LF_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_q2_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_LF_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_LF_q2_joint();
        const Type_fr_trunk_X_fr_LF_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_q3_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_LF_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_LF_q3_joint();
        const Type_fr_trunk_X_fr_LF_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LM_foot : public TransformHomogeneous<Type_fr_trunk_X_LM_foot>
    {
    public:
        Type_fr_trunk_X_LM_foot();
        const Type_fr_trunk_X_LM_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LM_q1_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_LM_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_LM_q1_joint();
        const Type_fr_trunk_X_fr_LM_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LM_q2_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_LM_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_LM_q2_joint();
        const Type_fr_trunk_X_fr_LM_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LM_q3_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_LM_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_LM_q3_joint();
        const Type_fr_trunk_X_fr_LM_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_LR_foot : public TransformHomogeneous<Type_fr_trunk_X_LR_foot>
    {
    public:
        Type_fr_trunk_X_LR_foot();
        const Type_fr_trunk_X_LR_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LR_q1_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_LR_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_LR_q1_joint();
        const Type_fr_trunk_X_fr_LR_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LR_q2_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_LR_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_LR_q2_joint();
        const Type_fr_trunk_X_fr_LR_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LR_q3_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_LR_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_LR_q3_joint();
        const Type_fr_trunk_X_fr_LR_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RF_foot : public TransformHomogeneous<Type_fr_trunk_X_RF_foot>
    {
    public:
        Type_fr_trunk_X_RF_foot();
        const Type_fr_trunk_X_RF_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_q1_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_RF_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_RF_q1_joint();
        const Type_fr_trunk_X_fr_RF_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_q2_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_RF_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_RF_q2_joint();
        const Type_fr_trunk_X_fr_RF_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_q3_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_RF_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_RF_q3_joint();
        const Type_fr_trunk_X_fr_RF_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RM_foot : public TransformHomogeneous<Type_fr_trunk_X_RM_foot>
    {
    public:
        Type_fr_trunk_X_RM_foot();
        const Type_fr_trunk_X_RM_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RM_q1_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_RM_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_RM_q1_joint();
        const Type_fr_trunk_X_fr_RM_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RM_q2_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_RM_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_RM_q2_joint();
        const Type_fr_trunk_X_fr_RM_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RM_q3_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_RM_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_RM_q3_joint();
        const Type_fr_trunk_X_fr_RM_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_RR_foot : public TransformHomogeneous<Type_fr_trunk_X_RR_foot>
    {
    public:
        Type_fr_trunk_X_RR_foot();
        const Type_fr_trunk_X_RR_foot& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RR_q1_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_RR_q1_joint>
    {
    public:
        Type_fr_trunk_X_fr_RR_q1_joint();
        const Type_fr_trunk_X_fr_RR_q1_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RR_q2_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_RR_q2_joint>
    {
    public:
        Type_fr_trunk_X_fr_RR_q2_joint();
        const Type_fr_trunk_X_fr_RR_q2_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RR_q3_joint : public TransformHomogeneous<Type_fr_trunk_X_fr_RR_q3_joint>
    {
    public:
        Type_fr_trunk_X_fr_RR_q3_joint();
        const Type_fr_trunk_X_fr_RR_q3_joint& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_hipassembly_X_fr_trunk : public TransformHomogeneous<Type_fr_LF_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_LF_hipassembly_X_fr_trunk();
        const Type_fr_LF_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LF_hipassembly : public TransformHomogeneous<Type_fr_trunk_X_fr_LF_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_LF_hipassembly();
        const Type_fr_trunk_X_fr_LF_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_LF_hipassembly : public TransformHomogeneous<Type_fr_LF_upperleg_X_fr_LF_hipassembly>
    {
    public:
        Type_fr_LF_upperleg_X_fr_LF_hipassembly();
        const Type_fr_LF_upperleg_X_fr_LF_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_hipassembly_X_fr_LF_upperleg : public TransformHomogeneous<Type_fr_LF_hipassembly_X_fr_LF_upperleg>
    {
    public:
        Type_fr_LF_hipassembly_X_fr_LF_upperleg();
        const Type_fr_LF_hipassembly_X_fr_LF_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_lowerleg_X_fr_LF_upperleg : public TransformHomogeneous<Type_fr_LF_lowerleg_X_fr_LF_upperleg>
    {
    public:
        Type_fr_LF_lowerleg_X_fr_LF_upperleg();
        const Type_fr_LF_lowerleg_X_fr_LF_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LF_upperleg_X_fr_LF_lowerleg : public TransformHomogeneous<Type_fr_LF_upperleg_X_fr_LF_lowerleg>
    {
    public:
        Type_fr_LF_upperleg_X_fr_LF_lowerleg();
        const Type_fr_LF_upperleg_X_fr_LF_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_hipassembly_X_fr_trunk : public TransformHomogeneous<Type_fr_LM_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_LM_hipassembly_X_fr_trunk();
        const Type_fr_LM_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LM_hipassembly : public TransformHomogeneous<Type_fr_trunk_X_fr_LM_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_LM_hipassembly();
        const Type_fr_trunk_X_fr_LM_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_upperleg_X_fr_LM_hipassembly : public TransformHomogeneous<Type_fr_LM_upperleg_X_fr_LM_hipassembly>
    {
    public:
        Type_fr_LM_upperleg_X_fr_LM_hipassembly();
        const Type_fr_LM_upperleg_X_fr_LM_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_hipassembly_X_fr_LM_upperleg : public TransformHomogeneous<Type_fr_LM_hipassembly_X_fr_LM_upperleg>
    {
    public:
        Type_fr_LM_hipassembly_X_fr_LM_upperleg();
        const Type_fr_LM_hipassembly_X_fr_LM_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_lowerleg_X_fr_LM_upperleg : public TransformHomogeneous<Type_fr_LM_lowerleg_X_fr_LM_upperleg>
    {
    public:
        Type_fr_LM_lowerleg_X_fr_LM_upperleg();
        const Type_fr_LM_lowerleg_X_fr_LM_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LM_upperleg_X_fr_LM_lowerleg : public TransformHomogeneous<Type_fr_LM_upperleg_X_fr_LM_lowerleg>
    {
    public:
        Type_fr_LM_upperleg_X_fr_LM_lowerleg();
        const Type_fr_LM_upperleg_X_fr_LM_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_hipassembly_X_fr_trunk : public TransformHomogeneous<Type_fr_LR_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_LR_hipassembly_X_fr_trunk();
        const Type_fr_LR_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_LR_hipassembly : public TransformHomogeneous<Type_fr_trunk_X_fr_LR_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_LR_hipassembly();
        const Type_fr_trunk_X_fr_LR_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_upperleg_X_fr_LR_hipassembly : public TransformHomogeneous<Type_fr_LR_upperleg_X_fr_LR_hipassembly>
    {
    public:
        Type_fr_LR_upperleg_X_fr_LR_hipassembly();
        const Type_fr_LR_upperleg_X_fr_LR_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_hipassembly_X_fr_LR_upperleg : public TransformHomogeneous<Type_fr_LR_hipassembly_X_fr_LR_upperleg>
    {
    public:
        Type_fr_LR_hipassembly_X_fr_LR_upperleg();
        const Type_fr_LR_hipassembly_X_fr_LR_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_lowerleg_X_fr_LR_upperleg : public TransformHomogeneous<Type_fr_LR_lowerleg_X_fr_LR_upperleg>
    {
    public:
        Type_fr_LR_lowerleg_X_fr_LR_upperleg();
        const Type_fr_LR_lowerleg_X_fr_LR_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_LR_upperleg_X_fr_LR_lowerleg : public TransformHomogeneous<Type_fr_LR_upperleg_X_fr_LR_lowerleg>
    {
    public:
        Type_fr_LR_upperleg_X_fr_LR_lowerleg();
        const Type_fr_LR_upperleg_X_fr_LR_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_hipassembly_X_fr_trunk : public TransformHomogeneous<Type_fr_RF_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_RF_hipassembly_X_fr_trunk();
        const Type_fr_RF_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RF_hipassembly : public TransformHomogeneous<Type_fr_trunk_X_fr_RF_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_RF_hipassembly();
        const Type_fr_trunk_X_fr_RF_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_RF_hipassembly : public TransformHomogeneous<Type_fr_RF_upperleg_X_fr_RF_hipassembly>
    {
    public:
        Type_fr_RF_upperleg_X_fr_RF_hipassembly();
        const Type_fr_RF_upperleg_X_fr_RF_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_hipassembly_X_fr_RF_upperleg : public TransformHomogeneous<Type_fr_RF_hipassembly_X_fr_RF_upperleg>
    {
    public:
        Type_fr_RF_hipassembly_X_fr_RF_upperleg();
        const Type_fr_RF_hipassembly_X_fr_RF_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_lowerleg_X_fr_RF_upperleg : public TransformHomogeneous<Type_fr_RF_lowerleg_X_fr_RF_upperleg>
    {
    public:
        Type_fr_RF_lowerleg_X_fr_RF_upperleg();
        const Type_fr_RF_lowerleg_X_fr_RF_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RF_upperleg_X_fr_RF_lowerleg : public TransformHomogeneous<Type_fr_RF_upperleg_X_fr_RF_lowerleg>
    {
    public:
        Type_fr_RF_upperleg_X_fr_RF_lowerleg();
        const Type_fr_RF_upperleg_X_fr_RF_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_hipassembly_X_fr_trunk : public TransformHomogeneous<Type_fr_RM_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_RM_hipassembly_X_fr_trunk();
        const Type_fr_RM_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RM_hipassembly : public TransformHomogeneous<Type_fr_trunk_X_fr_RM_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_RM_hipassembly();
        const Type_fr_trunk_X_fr_RM_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_upperleg_X_fr_RM_hipassembly : public TransformHomogeneous<Type_fr_RM_upperleg_X_fr_RM_hipassembly>
    {
    public:
        Type_fr_RM_upperleg_X_fr_RM_hipassembly();
        const Type_fr_RM_upperleg_X_fr_RM_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_hipassembly_X_fr_RM_upperleg : public TransformHomogeneous<Type_fr_RM_hipassembly_X_fr_RM_upperleg>
    {
    public:
        Type_fr_RM_hipassembly_X_fr_RM_upperleg();
        const Type_fr_RM_hipassembly_X_fr_RM_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_lowerleg_X_fr_RM_upperleg : public TransformHomogeneous<Type_fr_RM_lowerleg_X_fr_RM_upperleg>
    {
    public:
        Type_fr_RM_lowerleg_X_fr_RM_upperleg();
        const Type_fr_RM_lowerleg_X_fr_RM_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RM_upperleg_X_fr_RM_lowerleg : public TransformHomogeneous<Type_fr_RM_upperleg_X_fr_RM_lowerleg>
    {
    public:
        Type_fr_RM_upperleg_X_fr_RM_lowerleg();
        const Type_fr_RM_upperleg_X_fr_RM_lowerleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_hipassembly_X_fr_trunk : public TransformHomogeneous<Type_fr_RR_hipassembly_X_fr_trunk>
    {
    public:
        Type_fr_RR_hipassembly_X_fr_trunk();
        const Type_fr_RR_hipassembly_X_fr_trunk& update(const state_t&);
    protected:
    };
    
    class Type_fr_trunk_X_fr_RR_hipassembly : public TransformHomogeneous<Type_fr_trunk_X_fr_RR_hipassembly>
    {
    public:
        Type_fr_trunk_X_fr_RR_hipassembly();
        const Type_fr_trunk_X_fr_RR_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_upperleg_X_fr_RR_hipassembly : public TransformHomogeneous<Type_fr_RR_upperleg_X_fr_RR_hipassembly>
    {
    public:
        Type_fr_RR_upperleg_X_fr_RR_hipassembly();
        const Type_fr_RR_upperleg_X_fr_RR_hipassembly& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_hipassembly_X_fr_RR_upperleg : public TransformHomogeneous<Type_fr_RR_hipassembly_X_fr_RR_upperleg>
    {
    public:
        Type_fr_RR_hipassembly_X_fr_RR_upperleg();
        const Type_fr_RR_hipassembly_X_fr_RR_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_lowerleg_X_fr_RR_upperleg : public TransformHomogeneous<Type_fr_RR_lowerleg_X_fr_RR_upperleg>
    {
    public:
        Type_fr_RR_lowerleg_X_fr_RR_upperleg();
        const Type_fr_RR_lowerleg_X_fr_RR_upperleg& update(const state_t&);
    protected:
    };
    
    class Type_fr_RR_upperleg_X_fr_RR_lowerleg : public TransformHomogeneous<Type_fr_RR_upperleg_X_fr_RR_lowerleg>
    {
    public:
        Type_fr_RR_upperleg_X_fr_RR_lowerleg();
        const Type_fr_RR_upperleg_X_fr_RR_lowerleg& update(const state_t&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_trunk_X_LF_hipassemblyCOM fr_trunk_X_LF_hipassemblyCOM;
    Type_fr_trunk_X_LF_upperlegCOM fr_trunk_X_LF_upperlegCOM;
    Type_fr_trunk_X_LF_lowerlegCOM fr_trunk_X_LF_lowerlegCOM;
    Type_fr_trunk_X_LM_hipassemblyCOM fr_trunk_X_LM_hipassemblyCOM;
    Type_fr_trunk_X_LM_upperlegCOM fr_trunk_X_LM_upperlegCOM;
    Type_fr_trunk_X_LM_lowerlegCOM fr_trunk_X_LM_lowerlegCOM;
    Type_fr_trunk_X_LR_hipassemblyCOM fr_trunk_X_LR_hipassemblyCOM;
    Type_fr_trunk_X_LR_upperlegCOM fr_trunk_X_LR_upperlegCOM;
    Type_fr_trunk_X_LR_lowerlegCOM fr_trunk_X_LR_lowerlegCOM;
    Type_fr_trunk_X_RF_hipassemblyCOM fr_trunk_X_RF_hipassemblyCOM;
    Type_fr_trunk_X_RF_upperlegCOM fr_trunk_X_RF_upperlegCOM;
    Type_fr_trunk_X_RF_lowerlegCOM fr_trunk_X_RF_lowerlegCOM;
    Type_fr_trunk_X_RM_hipassemblyCOM fr_trunk_X_RM_hipassemblyCOM;
    Type_fr_trunk_X_RM_upperlegCOM fr_trunk_X_RM_upperlegCOM;
    Type_fr_trunk_X_RM_lowerlegCOM fr_trunk_X_RM_lowerlegCOM;
    Type_fr_trunk_X_RR_hipassemblyCOM fr_trunk_X_RR_hipassemblyCOM;
    Type_fr_trunk_X_RR_upperlegCOM fr_trunk_X_RR_upperlegCOM;
    Type_fr_trunk_X_RR_lowerlegCOM fr_trunk_X_RR_lowerlegCOM;
    Type_fr_LF_upperleg_X_fr_trunk fr_LF_upperleg_X_fr_trunk;
    Type_fr_LF_lowerleg_X_fr_trunk fr_LF_lowerleg_X_fr_trunk;
    Type_fr_LM_upperleg_X_fr_trunk fr_LM_upperleg_X_fr_trunk;
    Type_fr_LM_lowerleg_X_fr_trunk fr_LM_lowerleg_X_fr_trunk;
    Type_fr_LR_upperleg_X_fr_trunk fr_LR_upperleg_X_fr_trunk;
    Type_fr_LR_lowerleg_X_fr_trunk fr_LR_lowerleg_X_fr_trunk;
    Type_fr_RF_upperleg_X_fr_trunk fr_RF_upperleg_X_fr_trunk;
    Type_fr_RF_lowerleg_X_fr_trunk fr_RF_lowerleg_X_fr_trunk;
    Type_fr_RM_upperleg_X_fr_trunk fr_RM_upperleg_X_fr_trunk;
    Type_fr_RM_lowerleg_X_fr_trunk fr_RM_lowerleg_X_fr_trunk;
    Type_fr_RR_upperleg_X_fr_trunk fr_RR_upperleg_X_fr_trunk;
    Type_fr_RR_lowerleg_X_fr_trunk fr_RR_lowerleg_X_fr_trunk;
    Type_fr_trunk_X_LF_foot fr_trunk_X_LF_foot;
    Type_fr_trunk_X_fr_LF_q1_joint fr_trunk_X_fr_LF_q1_joint;
    Type_fr_trunk_X_fr_LF_q2_joint fr_trunk_X_fr_LF_q2_joint;
    Type_fr_trunk_X_fr_LF_q3_joint fr_trunk_X_fr_LF_q3_joint;
    Type_fr_trunk_X_LM_foot fr_trunk_X_LM_foot;
    Type_fr_trunk_X_fr_LM_q1_joint fr_trunk_X_fr_LM_q1_joint;
    Type_fr_trunk_X_fr_LM_q2_joint fr_trunk_X_fr_LM_q2_joint;
    Type_fr_trunk_X_fr_LM_q3_joint fr_trunk_X_fr_LM_q3_joint;
    Type_fr_trunk_X_LR_foot fr_trunk_X_LR_foot;
    Type_fr_trunk_X_fr_LR_q1_joint fr_trunk_X_fr_LR_q1_joint;
    Type_fr_trunk_X_fr_LR_q2_joint fr_trunk_X_fr_LR_q2_joint;
    Type_fr_trunk_X_fr_LR_q3_joint fr_trunk_X_fr_LR_q3_joint;
    Type_fr_trunk_X_RF_foot fr_trunk_X_RF_foot;
    Type_fr_trunk_X_fr_RF_q1_joint fr_trunk_X_fr_RF_q1_joint;
    Type_fr_trunk_X_fr_RF_q2_joint fr_trunk_X_fr_RF_q2_joint;
    Type_fr_trunk_X_fr_RF_q3_joint fr_trunk_X_fr_RF_q3_joint;
    Type_fr_trunk_X_RM_foot fr_trunk_X_RM_foot;
    Type_fr_trunk_X_fr_RM_q1_joint fr_trunk_X_fr_RM_q1_joint;
    Type_fr_trunk_X_fr_RM_q2_joint fr_trunk_X_fr_RM_q2_joint;
    Type_fr_trunk_X_fr_RM_q3_joint fr_trunk_X_fr_RM_q3_joint;
    Type_fr_trunk_X_RR_foot fr_trunk_X_RR_foot;
    Type_fr_trunk_X_fr_RR_q1_joint fr_trunk_X_fr_RR_q1_joint;
    Type_fr_trunk_X_fr_RR_q2_joint fr_trunk_X_fr_RR_q2_joint;
    Type_fr_trunk_X_fr_RR_q3_joint fr_trunk_X_fr_RR_q3_joint;
    Type_fr_LF_hipassembly_X_fr_trunk fr_LF_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_LF_hipassembly fr_trunk_X_fr_LF_hipassembly;
    Type_fr_LF_upperleg_X_fr_LF_hipassembly fr_LF_upperleg_X_fr_LF_hipassembly;
    Type_fr_LF_hipassembly_X_fr_LF_upperleg fr_LF_hipassembly_X_fr_LF_upperleg;
    Type_fr_LF_lowerleg_X_fr_LF_upperleg fr_LF_lowerleg_X_fr_LF_upperleg;
    Type_fr_LF_upperleg_X_fr_LF_lowerleg fr_LF_upperleg_X_fr_LF_lowerleg;
    Type_fr_LM_hipassembly_X_fr_trunk fr_LM_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_LM_hipassembly fr_trunk_X_fr_LM_hipassembly;
    Type_fr_LM_upperleg_X_fr_LM_hipassembly fr_LM_upperleg_X_fr_LM_hipassembly;
    Type_fr_LM_hipassembly_X_fr_LM_upperleg fr_LM_hipassembly_X_fr_LM_upperleg;
    Type_fr_LM_lowerleg_X_fr_LM_upperleg fr_LM_lowerleg_X_fr_LM_upperleg;
    Type_fr_LM_upperleg_X_fr_LM_lowerleg fr_LM_upperleg_X_fr_LM_lowerleg;
    Type_fr_LR_hipassembly_X_fr_trunk fr_LR_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_LR_hipassembly fr_trunk_X_fr_LR_hipassembly;
    Type_fr_LR_upperleg_X_fr_LR_hipassembly fr_LR_upperleg_X_fr_LR_hipassembly;
    Type_fr_LR_hipassembly_X_fr_LR_upperleg fr_LR_hipassembly_X_fr_LR_upperleg;
    Type_fr_LR_lowerleg_X_fr_LR_upperleg fr_LR_lowerleg_X_fr_LR_upperleg;
    Type_fr_LR_upperleg_X_fr_LR_lowerleg fr_LR_upperleg_X_fr_LR_lowerleg;
    Type_fr_RF_hipassembly_X_fr_trunk fr_RF_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_RF_hipassembly fr_trunk_X_fr_RF_hipassembly;
    Type_fr_RF_upperleg_X_fr_RF_hipassembly fr_RF_upperleg_X_fr_RF_hipassembly;
    Type_fr_RF_hipassembly_X_fr_RF_upperleg fr_RF_hipassembly_X_fr_RF_upperleg;
    Type_fr_RF_lowerleg_X_fr_RF_upperleg fr_RF_lowerleg_X_fr_RF_upperleg;
    Type_fr_RF_upperleg_X_fr_RF_lowerleg fr_RF_upperleg_X_fr_RF_lowerleg;
    Type_fr_RM_hipassembly_X_fr_trunk fr_RM_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_RM_hipassembly fr_trunk_X_fr_RM_hipassembly;
    Type_fr_RM_upperleg_X_fr_RM_hipassembly fr_RM_upperleg_X_fr_RM_hipassembly;
    Type_fr_RM_hipassembly_X_fr_RM_upperleg fr_RM_hipassembly_X_fr_RM_upperleg;
    Type_fr_RM_lowerleg_X_fr_RM_upperleg fr_RM_lowerleg_X_fr_RM_upperleg;
    Type_fr_RM_upperleg_X_fr_RM_lowerleg fr_RM_upperleg_X_fr_RM_lowerleg;
    Type_fr_RR_hipassembly_X_fr_trunk fr_RR_hipassembly_X_fr_trunk;
    Type_fr_trunk_X_fr_RR_hipassembly fr_trunk_X_fr_RR_hipassembly;
    Type_fr_RR_upperleg_X_fr_RR_hipassembly fr_RR_upperleg_X_fr_RR_hipassembly;
    Type_fr_RR_hipassembly_X_fr_RR_upperleg fr_RR_hipassembly_X_fr_RR_upperleg;
    Type_fr_RR_lowerleg_X_fr_RR_upperleg fr_RR_lowerleg_X_fr_RR_upperleg;
    Type_fr_RR_upperleg_X_fr_RR_lowerleg fr_RR_upperleg_X_fr_RR_lowerleg;

protected:

}; //class 'HomogeneousTransforms'

}
}

#endif

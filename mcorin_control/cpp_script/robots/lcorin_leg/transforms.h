#ifndef LCORIN_LEG_TRANSFORMS_H_
#define LCORIN_LEG_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace LCORIN_LEG {

// The type of the "vector" with the status of the variables
typedef iit::LCORIN_LEG::JointState state_t;

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
    class Type_fr_base_X_fr_l3 : public TransformMotion<Type_fr_base_X_fr_l3>
    {
    public:
        Type_fr_base_X_fr_l3();
        const Type_fr_base_X_fr_l3& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_ee : public TransformMotion<Type_fr_base_X_ee>
    {
    public:
        Type_fr_base_X_ee();
        const Type_fr_base_X_ee& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_fr_q1 : public TransformMotion<Type_fr_base_X_fr_q1>
    {
    public:
        Type_fr_base_X_fr_q1();
        const Type_fr_base_X_fr_q1& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_fr_q2 : public TransformMotion<Type_fr_base_X_fr_q2>
    {
    public:
        Type_fr_base_X_fr_q2();
        const Type_fr_base_X_fr_q2& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_fr_q3 : public TransformMotion<Type_fr_base_X_fr_q3>
    {
    public:
        Type_fr_base_X_fr_q3();
        const Type_fr_base_X_fr_q3& update(const state_t&);
    protected:
    };
    
    class Type_fr_l1_X_fr_base : public TransformMotion<Type_fr_l1_X_fr_base>
    {
    public:
        Type_fr_l1_X_fr_base();
        const Type_fr_l1_X_fr_base& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_fr_l1 : public TransformMotion<Type_fr_base_X_fr_l1>
    {
    public:
        Type_fr_base_X_fr_l1();
        const Type_fr_base_X_fr_l1& update(const state_t&);
    protected:
    };
    
    class Type_fr_l2_X_fr_l1 : public TransformMotion<Type_fr_l2_X_fr_l1>
    {
    public:
        Type_fr_l2_X_fr_l1();
        const Type_fr_l2_X_fr_l1& update(const state_t&);
    protected:
    };
    
    class Type_fr_l1_X_fr_l2 : public TransformMotion<Type_fr_l1_X_fr_l2>
    {
    public:
        Type_fr_l1_X_fr_l2();
        const Type_fr_l1_X_fr_l2& update(const state_t&);
    protected:
    };
    
    class Type_fr_l3_X_fr_l2 : public TransformMotion<Type_fr_l3_X_fr_l2>
    {
    public:
        Type_fr_l3_X_fr_l2();
        const Type_fr_l3_X_fr_l2& update(const state_t&);
    protected:
    };
    
    class Type_fr_l2_X_fr_l3 : public TransformMotion<Type_fr_l2_X_fr_l3>
    {
    public:
        Type_fr_l2_X_fr_l3();
        const Type_fr_l2_X_fr_l3& update(const state_t&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_base_X_fr_l3 fr_base_X_fr_l3;
    Type_fr_base_X_ee fr_base_X_ee;
    Type_fr_base_X_fr_q1 fr_base_X_fr_q1;
    Type_fr_base_X_fr_q2 fr_base_X_fr_q2;
    Type_fr_base_X_fr_q3 fr_base_X_fr_q3;
    Type_fr_l1_X_fr_base fr_l1_X_fr_base;
    Type_fr_base_X_fr_l1 fr_base_X_fr_l1;
    Type_fr_l2_X_fr_l1 fr_l2_X_fr_l1;
    Type_fr_l1_X_fr_l2 fr_l1_X_fr_l2;
    Type_fr_l3_X_fr_l2 fr_l3_X_fr_l2;
    Type_fr_l2_X_fr_l3 fr_l2_X_fr_l3;

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
    class Type_fr_base_X_fr_l3 : public TransformForce<Type_fr_base_X_fr_l3>
    {
    public:
        Type_fr_base_X_fr_l3();
        const Type_fr_base_X_fr_l3& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_ee : public TransformForce<Type_fr_base_X_ee>
    {
    public:
        Type_fr_base_X_ee();
        const Type_fr_base_X_ee& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_fr_q1 : public TransformForce<Type_fr_base_X_fr_q1>
    {
    public:
        Type_fr_base_X_fr_q1();
        const Type_fr_base_X_fr_q1& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_fr_q2 : public TransformForce<Type_fr_base_X_fr_q2>
    {
    public:
        Type_fr_base_X_fr_q2();
        const Type_fr_base_X_fr_q2& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_fr_q3 : public TransformForce<Type_fr_base_X_fr_q3>
    {
    public:
        Type_fr_base_X_fr_q3();
        const Type_fr_base_X_fr_q3& update(const state_t&);
    protected:
    };
    
    class Type_fr_l1_X_fr_base : public TransformForce<Type_fr_l1_X_fr_base>
    {
    public:
        Type_fr_l1_X_fr_base();
        const Type_fr_l1_X_fr_base& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_fr_l1 : public TransformForce<Type_fr_base_X_fr_l1>
    {
    public:
        Type_fr_base_X_fr_l1();
        const Type_fr_base_X_fr_l1& update(const state_t&);
    protected:
    };
    
    class Type_fr_l2_X_fr_l1 : public TransformForce<Type_fr_l2_X_fr_l1>
    {
    public:
        Type_fr_l2_X_fr_l1();
        const Type_fr_l2_X_fr_l1& update(const state_t&);
    protected:
    };
    
    class Type_fr_l1_X_fr_l2 : public TransformForce<Type_fr_l1_X_fr_l2>
    {
    public:
        Type_fr_l1_X_fr_l2();
        const Type_fr_l1_X_fr_l2& update(const state_t&);
    protected:
    };
    
    class Type_fr_l3_X_fr_l2 : public TransformForce<Type_fr_l3_X_fr_l2>
    {
    public:
        Type_fr_l3_X_fr_l2();
        const Type_fr_l3_X_fr_l2& update(const state_t&);
    protected:
    };
    
    class Type_fr_l2_X_fr_l3 : public TransformForce<Type_fr_l2_X_fr_l3>
    {
    public:
        Type_fr_l2_X_fr_l3();
        const Type_fr_l2_X_fr_l3& update(const state_t&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_base_X_fr_l3 fr_base_X_fr_l3;
    Type_fr_base_X_ee fr_base_X_ee;
    Type_fr_base_X_fr_q1 fr_base_X_fr_q1;
    Type_fr_base_X_fr_q2 fr_base_X_fr_q2;
    Type_fr_base_X_fr_q3 fr_base_X_fr_q3;
    Type_fr_l1_X_fr_base fr_l1_X_fr_base;
    Type_fr_base_X_fr_l1 fr_base_X_fr_l1;
    Type_fr_l2_X_fr_l1 fr_l2_X_fr_l1;
    Type_fr_l1_X_fr_l2 fr_l1_X_fr_l2;
    Type_fr_l3_X_fr_l2 fr_l3_X_fr_l2;
    Type_fr_l2_X_fr_l3 fr_l2_X_fr_l3;

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
    class Type_fr_base_X_fr_l3 : public TransformHomogeneous<Type_fr_base_X_fr_l3>
    {
    public:
        Type_fr_base_X_fr_l3();
        const Type_fr_base_X_fr_l3& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_ee : public TransformHomogeneous<Type_fr_base_X_ee>
    {
    public:
        Type_fr_base_X_ee();
        const Type_fr_base_X_ee& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_fr_q1 : public TransformHomogeneous<Type_fr_base_X_fr_q1>
    {
    public:
        Type_fr_base_X_fr_q1();
        const Type_fr_base_X_fr_q1& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_fr_q2 : public TransformHomogeneous<Type_fr_base_X_fr_q2>
    {
    public:
        Type_fr_base_X_fr_q2();
        const Type_fr_base_X_fr_q2& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_fr_q3 : public TransformHomogeneous<Type_fr_base_X_fr_q3>
    {
    public:
        Type_fr_base_X_fr_q3();
        const Type_fr_base_X_fr_q3& update(const state_t&);
    protected:
    };
    
    class Type_fr_l1_X_fr_base : public TransformHomogeneous<Type_fr_l1_X_fr_base>
    {
    public:
        Type_fr_l1_X_fr_base();
        const Type_fr_l1_X_fr_base& update(const state_t&);
    protected:
    };
    
    class Type_fr_base_X_fr_l1 : public TransformHomogeneous<Type_fr_base_X_fr_l1>
    {
    public:
        Type_fr_base_X_fr_l1();
        const Type_fr_base_X_fr_l1& update(const state_t&);
    protected:
    };
    
    class Type_fr_l2_X_fr_l1 : public TransformHomogeneous<Type_fr_l2_X_fr_l1>
    {
    public:
        Type_fr_l2_X_fr_l1();
        const Type_fr_l2_X_fr_l1& update(const state_t&);
    protected:
    };
    
    class Type_fr_l1_X_fr_l2 : public TransformHomogeneous<Type_fr_l1_X_fr_l2>
    {
    public:
        Type_fr_l1_X_fr_l2();
        const Type_fr_l1_X_fr_l2& update(const state_t&);
    protected:
    };
    
    class Type_fr_l3_X_fr_l2 : public TransformHomogeneous<Type_fr_l3_X_fr_l2>
    {
    public:
        Type_fr_l3_X_fr_l2();
        const Type_fr_l3_X_fr_l2& update(const state_t&);
    protected:
    };
    
    class Type_fr_l2_X_fr_l3 : public TransformHomogeneous<Type_fr_l2_X_fr_l3>
    {
    public:
        Type_fr_l2_X_fr_l3();
        const Type_fr_l2_X_fr_l3& update(const state_t&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_base_X_fr_l3 fr_base_X_fr_l3;
    Type_fr_base_X_ee fr_base_X_ee;
    Type_fr_base_X_fr_q1 fr_base_X_fr_q1;
    Type_fr_base_X_fr_q2 fr_base_X_fr_q2;
    Type_fr_base_X_fr_q3 fr_base_X_fr_q3;
    Type_fr_l1_X_fr_base fr_l1_X_fr_base;
    Type_fr_base_X_fr_l1 fr_base_X_fr_l1;
    Type_fr_l2_X_fr_l1 fr_l2_X_fr_l1;
    Type_fr_l1_X_fr_l2 fr_l1_X_fr_l2;
    Type_fr_l3_X_fr_l2 fr_l3_X_fr_l2;
    Type_fr_l2_X_fr_l3 fr_l2_X_fr_l3;

protected:

}; //class 'HomogeneousTransforms'


}
}

#endif

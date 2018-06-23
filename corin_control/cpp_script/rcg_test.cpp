// Leg node utilising action clients for sending goals and monitoring state
// Explanation on action available at: http://wiki.ros.org/actionlib/DetailedDescription
// Action class details at: http://docs.ros.org/jade/api/actionlib/html/classactionlib_1_1SimpleActionClient.html

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <Eigen/Dense>

#include "kinematics/kdl.h"

#include "robots/mcorin/declarations.h"
#include "robots/mcorin/transforms.h"
#include "robots/mcorin/jacobians.h"
#include "robots/mcorin/inertia_properties.h"
#include "robots/mcorin/inverse_dynamics.h"

#include <qpOASES.hpp>

using namespace iit::mcorin;
// namespace mcorin = iit::mcorin;

// SLM-------------------------------------------------


int main(int argc, char **argv)
{
  // MotionTransforms xM;
  // HomogeneousTransforms xH;
  // JointState q, qd, qdd, tau;

  // dyn::InertiaProperties inertias;
  // dyn::InverseDynamics invdyn(inertias, xM);

  // dyn::InverseDynamics::Velocity trunk_v;       // input : desired velocity of trunk (spatial)
  // dyn::InverseDynamics::Acceleration trunk_a;   // I/O   : desired acceleration/acceleration for trunk (spatial)
  // dyn::InverseDynamics::Acceleration gravity;   // input : gravity (spatial)
  // dyn::InverseDynamics::ExtForces extForce;     // input : forces applied to robot
  // dyn::InverseDynamics::Force baseForce;        // output: force applied to base

  // ros::init(argc, argv, "leg_node");    // initializes node
  // ros::NodeHandle n;

  // ros::Rate loop_rate(2);
  // ROS_INFO("First pass done");

  // q(0) = 0.5; // set the joint status
  // q(1) = 1.5;
  // qd(0) = 0.0;
  // qdd(0) = 0.01;
  
  // auto inertia_LF_hip = xM.fr_LF_hipassembly_X_fr_trunk(q).transpose()*inertias.getTensor_LF_hipassembly()*xM.fr_LF_hipassembly_X_fr_trunk(q);
  // auto inertia_LF_upp = xM.fr_LF_upperleg_X_fr_trunk(q).transpose()*inertias.getTensor_LF_upperleg()*xM.fr_LF_upperleg_X_fr_trunk(q);
  // auto inertia_LF_low = xM.fr_LF_lowerleg_X_fr_trunk(q).transpose()*inertias.getTensor_LF_lowerleg()*xM.fr_LF_lowerleg_X_fr_trunk(q);

  // auto inertia_LM_hip = xM.fr_LM_hipassembly_X_fr_trunk(q).transpose()*inertias.getTensor_LM_hipassembly()*xM.fr_LM_hipassembly_X_fr_trunk(q);
  // auto inertia_LM_upp = xM.fr_LM_upperleg_X_fr_trunk(q).transpose()*inertias.getTensor_LM_upperleg()*xM.fr_LM_upperleg_X_fr_trunk(q);
  // auto inertia_LM_low = xM.fr_LM_lowerleg_X_fr_trunk(q).transpose()*inertias.getTensor_LM_lowerleg()*xM.fr_LM_lowerleg_X_fr_trunk(q);

  // auto inertia_LR_hip = xM.fr_LR_hipassembly_X_fr_trunk(q).transpose()*inertias.getTensor_LR_hipassembly()*xM.fr_LR_hipassembly_X_fr_trunk(q);
  // auto inertia_LR_upp = xM.fr_LR_upperleg_X_fr_trunk(q).transpose()*inertias.getTensor_LR_upperleg()*xM.fr_LR_upperleg_X_fr_trunk(q);
  // auto inertia_LR_low = xM.fr_LR_lowerleg_X_fr_trunk(q).transpose()*inertias.getTensor_LR_lowerleg()*xM.fr_LR_lowerleg_X_fr_trunk(q);

  // auto inertia_RF_hip = xM.fr_RF_hipassembly_X_fr_trunk(q).transpose()*inertias.getTensor_RF_hipassembly()*xM.fr_RF_hipassembly_X_fr_trunk(q);
  // auto inertia_RF_upp = xM.fr_RF_upperleg_X_fr_trunk(q).transpose()*inertias.getTensor_RF_upperleg()*xM.fr_RF_upperleg_X_fr_trunk(q);
  // auto inertia_RF_low = xM.fr_RF_lowerleg_X_fr_trunk(q).transpose()*inertias.getTensor_RF_lowerleg()*xM.fr_RF_lowerleg_X_fr_trunk(q);

  // auto inertia_RM_hip = xM.fr_RM_hipassembly_X_fr_trunk(q).transpose()*inertias.getTensor_RM_hipassembly()*xM.fr_RM_hipassembly_X_fr_trunk(q);
  // auto inertia_RM_upp = xM.fr_RM_upperleg_X_fr_trunk(q).transpose()*inertias.getTensor_RM_upperleg()*xM.fr_RM_upperleg_X_fr_trunk(q);
  // auto inertia_RM_low = xM.fr_RM_lowerleg_X_fr_trunk(q).transpose()*inertias.getTensor_RM_lowerleg()*xM.fr_RM_lowerleg_X_fr_trunk(q);

  // auto inertia_RR_hip = xM.fr_RR_hipassembly_X_fr_trunk(q).transpose()*inertias.getTensor_RR_hipassembly()*xM.fr_RR_hipassembly_X_fr_trunk(q);
  // auto inertia_RR_upp = xM.fr_RR_upperleg_X_fr_trunk(q).transpose()*inertias.getTensor_RR_upperleg()*xM.fr_RR_upperleg_X_fr_trunk(q);
  // auto inertia_RR_low = xM.fr_RR_lowerleg_X_fr_trunk(q).transpose()*inertias.getTensor_RR_lowerleg()*xM.fr_RR_lowerleg_X_fr_trunk(q);

  // auto composite_inertia = inertias.getTensor_trunk() +
  //                           inertia_LF_hip + inertia_LF_upp + inertia_LF_low +
  //                           inertia_LM_hip + inertia_LM_upp + inertia_LM_low +
  //                           inertia_LR_hip + inertia_LR_upp + inertia_LR_low +
  //                           inertia_RF_hip + inertia_RF_upp + inertia_RF_low +
  //                           inertia_RM_hip + inertia_RM_upp + inertia_RM_low +
  //                           inertia_RR_hip + inertia_RR_upp + inertia_RR_low;

  // std::cout << "===================================================" << std::endl;
  // // std::cout << xM.fr_trunk_X_fr_LF_hipassembly(q) << std::endl;
  // std::cout << composite_inertia << std::endl;
  // // output: joint torque and base acceleration
  // invdyn.id(tau, trunk_a, gravity, trunk_v, q, qd, qdd, extForce);  

  // // output: base force and joint torques
  // invdyn.id_fully_actuated(baseForce, tau, gravity, trunk_v, trunk_a, q, qd, qdd, extForce);


  return 0;
}


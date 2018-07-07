#include "ros/ros.h"

#include "corin_msgs/MotionPlan.h"
#include "corin_msgs/RigidBody.h"

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "sensor_msgs/JointState.h"
#include "robots/mcorin/declarations.h"
#include "robots/mcorin/transforms.h"
#include "robots/mcorin/jacobians.h"
#include "robots/mcorin/inertia_properties.h"
#include "robots/mcorin/inverse_dynamics.h"
#include "robots/mcorin/miscellaneous.h"

using namespace std;
using namespace Eigen;
using namespace iit::mcorin;

MotionTransforms xM;
dyn::InertiaProperties inertias;

Eigen::MatrixXd CRBI(JointState qpr);
/*========================================================================
                            Services
  ========================================================================*/

// Service to return motor state and torque for a leg i.e. three motor state and torque
bool getInertiaState(corin_msgs::RigidBody::Request  &req,
                    corin_msgs::RigidBody::Response &res)
{
  // res.jointState  = current_jointState;                       // motor state
  if ( (req.motionPlan.setpoint.positions.size() == 24) && 
          (req.motionPlan.setpoint.velocities.size() == 24) &&
          (req.motionPlan.setpoint.accelerations.size() == 24) &&
          (req.motionPlan.gait_phase.size() == 6)  )
    {
      cout << " data valid " << endl;

      JointState qpr, qvr, qar;                       // reference joint state
      
      HomogeneousTransforms xH;
      

      Eigen::MatrixXd qbpr, qbvr, qbar;   // base reference states
      Eigen::MatrixXi gait_phase;         // reference gait phase
      qbpr = qbvr = qbar = Eigen::MatrixXd::Zero(6,1);
      gait_phase = Eigen::MatrixXi::Zero(6,1);

      // Update Robot State
      for (int i=0; i < 6; i++){
        // cout << req.motionPlan.setpoint.positions[i] << endl;
        qbpr(i) = req.motionPlan.setpoint.positions[i];
        qbvr(i) = req.motionPlan.setpoint.velocities[i];
        qbar(i) = req.motionPlan.setpoint.accelerations[i];
        gait_phase(i) = req.motionPlan.gait_phase[i];
      }

      // Leg joint angles
      for (int i=6; i < 24; i++){    
        qpr(i-6) = req.motionPlan.setpoint.positions[i];         // rcg: reference joint space position
        qvr(i-6) = req.motionPlan.setpoint.velocities[i];        // rcg: reference joint space velocity
        qar(i-6) = req.motionPlan.setpoint.accelerations[i];     // joint space acceleration
      }

      // compute CoM location Re^3
      auto x_com = getWholeBodyCOM(inertias, qpr, xH);
      auto crbim = CRBI(qpr);
      cout << crbim << endl;
      // remap from eigen to vector
      std::vector<double> vec_xcom(x_com.data(), x_com.data() + x_com.rows() * x_com.cols());
      std::vector<double> vec_crbi(crbim.data(), crbim.data() + crbim.rows() * crbim.cols());

      res.CoM  = vec_xcom;
      res.CRBI = vec_crbi;
    }
  else
    cout << "data size less" << endl;
  

  //ROS_INFO("Request processed");

  return true;
}

Eigen::MatrixXd CRBI(JointState qpr)
{
  auto inertia_LF_hip = xM.fr_LF_hipassembly_X_fr_trunk(qpr).transpose()*inertias.getTensor_LF_hipassembly()*xM.fr_LF_hipassembly_X_fr_trunk(qpr);
  auto inertia_LF_upp = xM.fr_LF_upperleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_LF_upperleg()*xM.fr_LF_upperleg_X_fr_trunk(qpr);
  auto inertia_LF_low = xM.fr_LF_lowerleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_LF_lowerleg()*xM.fr_LF_lowerleg_X_fr_trunk(qpr);

  auto inertia_LM_hip = xM.fr_LM_hipassembly_X_fr_trunk(qpr).transpose()*inertias.getTensor_LM_hipassembly()*xM.fr_LM_hipassembly_X_fr_trunk(qpr);
  auto inertia_LM_upp = xM.fr_LM_upperleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_LM_upperleg()*xM.fr_LM_upperleg_X_fr_trunk(qpr);
  auto inertia_LM_low = xM.fr_LM_lowerleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_LM_lowerleg()*xM.fr_LM_lowerleg_X_fr_trunk(qpr);

  auto inertia_LR_hip = xM.fr_LR_hipassembly_X_fr_trunk(qpr).transpose()*inertias.getTensor_LR_hipassembly()*xM.fr_LR_hipassembly_X_fr_trunk(qpr);
  auto inertia_LR_upp = xM.fr_LR_upperleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_LR_upperleg()*xM.fr_LR_upperleg_X_fr_trunk(qpr);
  auto inertia_LR_low = xM.fr_LR_lowerleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_LR_lowerleg()*xM.fr_LR_lowerleg_X_fr_trunk(qpr);

  auto inertia_RF_hip = xM.fr_RF_hipassembly_X_fr_trunk(qpr).transpose()*inertias.getTensor_RF_hipassembly()*xM.fr_RF_hipassembly_X_fr_trunk(qpr);
  auto inertia_RF_upp = xM.fr_RF_upperleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_RF_upperleg()*xM.fr_RF_upperleg_X_fr_trunk(qpr);
  auto inertia_RF_low = xM.fr_RF_lowerleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_RF_lowerleg()*xM.fr_RF_lowerleg_X_fr_trunk(qpr);

  auto inertia_RM_hip = xM.fr_RM_hipassembly_X_fr_trunk(qpr).transpose()*inertias.getTensor_RM_hipassembly()*xM.fr_RM_hipassembly_X_fr_trunk(qpr);
  auto inertia_RM_upp = xM.fr_RM_upperleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_RM_upperleg()*xM.fr_RM_upperleg_X_fr_trunk(qpr);
  auto inertia_RM_low = xM.fr_RM_lowerleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_RM_lowerleg()*xM.fr_RM_lowerleg_X_fr_trunk(qpr);

  auto inertia_RR_hip = xM.fr_RR_hipassembly_X_fr_trunk(qpr).transpose()*inertias.getTensor_RR_hipassembly()*xM.fr_RR_hipassembly_X_fr_trunk(qpr);
  auto inertia_RR_upp = xM.fr_RR_upperleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_RR_upperleg()*xM.fr_RR_upperleg_X_fr_trunk(qpr);
  auto inertia_RR_low = xM.fr_RR_lowerleg_X_fr_trunk(qpr).transpose()*inertias.getTensor_RR_lowerleg()*xM.fr_RR_lowerleg_X_fr_trunk(qpr);

  auto composite_inertia = inertias.getTensor_trunk() +
                            inertia_LF_hip + inertia_LF_upp + inertia_LF_low +
                            inertia_LM_hip + inertia_LM_upp + inertia_LM_low +
                            inertia_LR_hip + inertia_LR_upp + inertia_LR_low +
                            inertia_RF_hip + inertia_RF_upp + inertia_RF_low +
                            inertia_RM_hip + inertia_RM_upp + inertia_RM_low +
                            inertia_RR_hip + inertia_RR_upp + inertia_RR_low;
  return composite_inertia.block<3,3>(0,0);
}

/*==========================  Main  ===================================*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rigid_body_server");
  ros::NodeHandle nh_;
  
  //---service server---//
  ros::ServiceServer rigid_body_serv_ = nh_.advertiseService("/corin/get_rigid_body_matrix", getInertiaState);

  ROS_INFO("Ready to get inertia states");

  ros::spin();

  return 0;
}

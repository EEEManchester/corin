#include "kdl.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <Eigen/Dense>

#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"
 

Eigen::Matrix<double, 3, 1> IK(double ctp[])
{
  Eigen::Matrix<double, 3, 1> ctx;//, ctv, cta;
  double q2t[2], q3t[2];
  double q1, q2, q3;
  // Robot parameters
  float l1 = 0.077;
  float l2 = 0.150;
  float l3 = 0.170;
  
  double x = ctp[0]; double y = ctp[1]; double z = ctp[2];

  q1 = atan2(y,x);
  
  // Adding the square of element (1,4) & (2,4) in inv(H01)*Psym = T12*T23
  double c3  = ( pow((x*cos(q1) + y*sin(q1) - l1),2.0) + z*z -l3*l3-l2*l2 )/(2*l2*l3);
  double s3  = sqrt(1.0-c3*c3);
  q3t[0] = atan2(s3,c3);
  q3t[1] = atan2(-s3,c3);
  
  if (q3t[0] < 0.0)
    q3 = q3t[0];
  else
    q3 = q3t[1];

  // Dividing the element (1,4) & (2,4) in inv(H01)*Psym = T12*T23
  double xp = x*cos(q1) + sin(q1)*y - l1; 
  double yp = z;
  q2 = atan2(yp,xp) - atan2(l3*sin(q3), l2+l3*cos(q3));
  // q2t[0] = atan2(yp,xp) - atan2(l3*sin(q3), l2+l3*cos(q3));
  // q2t[1] = atan2(yp,xp) - atan2(l3*sin(q3), l2+l3*cos(q3));

  ctx(0,0) = q1;
  ctx(1,0) = q2;
  ctx(2,0) = q3;

  // JointState ik_q;
  // ik_q(0) = q1;
  // ik_q(1) = q2;
  // ik_q(2) = q3;
  // q1 = 0.0; q2 = 0.0;q3 = 0.0;
  // Eigen::Matrix<double, 3, 3> ik_jq;
  // ik_jq = jacobians.fr_base_J_ee(ik_q).block<3,3>(3,0);
  
  // Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(ik_jq);
  // int lu_rank = lu_decomp.rank();

  // Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(ik_jq.rows(), ik_jq.cols());
  // qr.compute(ik_jq);
  // int qr_rank = qr.rank();

  // std::cout << "LU rank: " << lu_rank << std::endl;
  // std::cout << "QR rank: " << qr_rank << std::endl;

  return ctx;
}
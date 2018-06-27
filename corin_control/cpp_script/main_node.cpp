#include <ros/ros.h>
// #include "body_controller.h"
#include "force_distribution.h"
#include <eigen-quadprog/QuadProg.h>
#include <Eigen/Dense>
void test_qp_01();
void test_qp_02();
void test_qp_03();
void test_eqp_01();
int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_control_node"); // register the node on ROS
  ros::NodeHandle nh; // get a handle to ROS

  // body_controller::bodyController* mcorin_class = new body_controller::bodyController();
  force_distribution::ForceDistribution* ForceClass = new force_distribution::ForceDistribution();

  // instatiate our class object
  ROS_INFO_STREAM( "Corin Force Node Ready");

  // test_qp_01();
  // test_qp_02();
  // test_qp_03();
  // test_eqp_01();
  
  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  // dynamic_reconfigure::Server<mcorin_control::dynreconfigConfig> dr_srv;
  // dynamic_reconfigure::Server<mcorin_control::dynreconfigConfig>::CallbackType cb;
  // cb = boost::bind(&body_controller::bodyController::dconfig_callback, mcorin_class, _1, _2);
  // dr_srv.setCallback(cb);

  

  ros::spin(); // check for new messages and call the callback if we get one

  return 0;
}

void test_eqp_01()
{
  int nrvar, nreq, nrineq;
  Eigen::MatrixXd Q, Aeq, Aineq;
  Eigen::VectorXd C, Beq, Bineq, XL, XU, X;

  nrvar = 6;
  nreq = 3;
  nrineq = 2;

  Q.resize(nrvar, nrvar);
  Aeq.resize(nreq, nrvar);
  Aineq.resize(nrineq, nrvar);

  C.resize(nrvar);
  Beq.resize(nreq);
  Bineq.resize(nrineq);
  XL.resize(nrvar);
  XU.resize(nrvar);
  X.resize(nrvar);


  Aeq << 1., -1., 1., 0., 3., 1.,
         -1., 0., -3., -4., 5., 6.,
         2., 5., 3., 0., 1., 0.;
  Beq << 1., 2., 3.;

  Aineq << 0., 1., 0., 1., 2., -1.,
           -1., 0., 2., 1., 1., 0.;
  Bineq << -1., 2.5;

  //with  x between ci and cs:
  XL << -1000., -10000., 0., -1000., -1000.,-1000.;
  XU << 10000., 100., 1.5, 100., 100., 1000.;

  //and minimize 0.5*x'*Q*x + p'*x with
  C << 1., 2., 3., 4., 5., 6.;
  Q.setIdentity();

  X << 1.7975426, -0.3381487, 0.1633880, -4.9884023, 0.6054943, -3.1155623;
  
  using namespace Eigen;
  int nrineq1 = static_cast<int>(Aineq.rows());
  // QuadProgDense qp();
  // QuadProgDense qp(nrvar, nreq, nrineq1);
  // qp.solve(Q, C, Aeq, Beq, Aineq, Bineq);

}
void test_qp_01()
{
  /* Setup data of first QP. */
  USING_NAMESPACE_QPOASES
  real_t H[2*2] = { 1.0, 0.0, 0.0, 0.0 };
  real_t g[2] = { 3.0, 4.0 };
  real_t A[5*2] = { -1.0, 0.0, 0.0, -1.0,-1.0,-3.0,2.0,5.0,3.0,4.0 };
  real_t *lbA = NULL;
  real_t ubA[5] = { 0.0,0.0,-15.0,100.0,80.0 };
  real_t *lb = NULL;
  real_t *ub = NULL;

  /* Setting up QProblem object. */
  QProblem example( 2,5 );
  Options options;
  example.setOptions( options );

  /* Solve first QP. */
  int nWSR = 10;
  example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

  /* Get and print solution of first QP. */
  real_t xOpt[2];
  real_t yOpt[2+1];
  example.getPrimalSolution( xOpt );
  example.getDualSolution( yOpt );
  printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
      xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );
}

void test_qp_02()
{
  using namespace Eigen;
  using namespace std;
  MatrixXd qH(2,2);
  qH << 2.0, 3.1, 3.55, 10;
  cout << qH << endl;

  /* Setup data of first QP. */
  USING_NAMESPACE_QPOASES
  // real_t H[2*2] = { 1.0, 0.0, 0.0, 5.0 };
  real_t H[qH.size()];
  Map<RowVectorXd> v1(qH.data(), qH.size());  // first, flatten to 1D row vector
  Map<RowVectorXd>(&H[0], qH.size()) = v1;  // second, convert to array - pointer

  real_t g[2] = { 3.0, 4.0 };
  real_t A[5*2] = { -1.0, 0.0, 0.0, -1.0,-1.0,-3.0,2.0,5.0,3.0,4.0 };
  real_t *lbA = NULL;
  real_t ubA[5] = { 0.0,0.0,-15.0,100.0,80.0 };
  real_t *lb = NULL;
  real_t *ub = NULL;

  /* Setting up QProblem object. */
  QProblem example( 2,5 );
  Options options;
  example.setOptions( options );

  /* Solve first QP. */
  int nWSR = 10;
  example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

  /* Get and print solution of first QP. */
  real_t xOpt[2];
  real_t yOpt[2+1];
  example.getPrimalSolution( xOpt );
  example.getDualSolution( yOpt );
  printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
      xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );
}

void test_qp_03()
{
  using namespace Eigen;
  using namespace std;
  

  int n_contact = 3;
  Vector3d g(0,0,9.81), wg(0,0,0);

  VectorXd B(6);
  B.block<3,1>(0,0) = 5.0*(g);
  // B.block<3,1>(0,0) = inertias.getTotalMass()*(qbpr.block<3,1>(0,0) + g);
  B.block<3,1>(3,0) = wg;
  cout << B << endl;

  // constructing A
  int pc = 0;
  MatrixXd A(6,3*n_contact);
  A.setZero(6,3*n_contact);
  // MatrixXd A(3,3*n_contact);
  // cycles through all six legs
  for (int j=0; j<n_contact; j++){
    // amend matrix if legs are in support phase
    A.block<3,3>(0,j*3) = MatrixXd::Identity(3, 3);
  }
  cout << A << endl;

  // friction constraint
  MatrixXd C(6,3), Cineq(6*n_contact, 3*n_contact);
  MatrixXd D(6,1), Dineq(6*n_contact, 1);

  Cineq.setZero(6*n_contact, 3*n_contact);
  Dineq.setZero(6*n_contact, 1);

  double mu = 1.0;  // friction coefficient
  double fmin = 1.0;  // min. force
  double fmax = 10*5.0; // max. force

  C <<   1, 0, -mu,
        -1, 0, -mu,
         0, 1, -mu,
         0,-1, -mu,
         0, 0, -1,
         0, 0, 1;

  D << 0,0,0,0, -fmin, fmax;

  for (int i=0; i<n_contact; i++){
    Cineq.block<6,3>(i*6,i*3) = C;
    Dineq.block<6,1>(i*6,0) = D;
  }
  // cout << "=========================" << endl;
  cout << Cineq << endl;
  cout << Dineq << endl;

  // weightage matrix
  VectorXd sV(6);
  sV << 1, 1, 1, 1, 1, 1;
  // VectorXd sV(3);
  // sV << 1, 1, 10;
  MatrixXd wS = sV.asDiagonal();

  // construct equation
  MatrixXd Hs = 2*A.transpose()*wS*A;
  MatrixXd Qs = (-2*B.transpose()*wS*A).transpose();
  cout << Hs << endl;
  cout << Qs << endl;
  cout << "=================" << endl;
  USING_NAMESPACE_QPOASES
  real_t qpH[Hs.size()];   // Hessian Matrix, Re^(3cx3c)
  real_t qpG[Qs.size()];   // Gradient Vector, Re^(3c)
  real_t qpA[Cineq.size()]; // Constraint Matrix, 
  real_t qpubA[Dineq.size()]; // Upper boundary for constraint
  real_t *qplbA = NULL;
  real_t *qpLB = NULL;
  real_t *qpUB = NULL;
  
  // remap from eigen to std array form
  Eigen::Matrix<double,Dynamic,Dynamic,RowMajor> M1(Hs);
  Map<RowVectorXd> v1(M1.data(), M1.size());  // first, flatten to 1D row vector
  Map<RowVectorXd>(&qpH[0], Hs.size()) = v1;  // second, convert to array - pointer
  
  Map<RowVectorXd> v2(Qs.data(), Qs.size());
  Map<RowVectorXd>(&qpG[0], Qs.size()) = v2;
  
  Eigen::Matrix<double,Dynamic,Dynamic,RowMajor> M3(Cineq);
  Map<RowVectorXd> v3(M3.data(), M3.size());
  RowVectorXd::Map(&qpA[0], M3.size()) = v3;
  
  // cout << qpA << endl;
  // cout << "----------------" << endl;
  Map<RowVectorXd> v4(Dineq.data(), Dineq.size());
  Map<RowVectorXd>(&qpubA[0], Dineq.size()) = v4;

  // for (int i=0;i<6;i++){
  //   cout << qpH[i*6] << " " << qpH[i*6+1] << " " << qpH[i*6+2]<< " " << qpH[i*6+3]<< " " << qpH[i*6+4]<< " " << qpH[i*6+5] << endl;
  // }
  // for (int i=0;i<6*n_contact;i++){
  //   for (int j=0;j<3*n_contact; j++){
  //     cout << qpA[i*6+j] << " ";
  //   }
  //   cout << endl;
  //   // cout << qpA[i*6] << " " << qpA[i*6+1] << " " << qpA[i*6+2]<< " " << qpA[i*6+3]<< " " << qpA[i*6+4]<< " " << qpA[i*6+5] << endl;
  // }
  cout << v4 << endl;
  cout << "---------------------" << endl;
  // for (int i=0;i<Cineq.size();i++)
  //   cout << qpA[i] << " ";
  // cout << endl;
  for (int i=0;i<Dineq.size();i++)
    cout << qpubA[i] << " ";
  cout << endl;
  // for (int i=0;i<6;i++){
  //   cout << qpG[i*6] << " " << qpG[i*6+1] << " " << qpG[i*6+2]<< " " << qpG[i*6+3]<< " " << qpG[i*6+4]<< " " << qpG[i*6+5] << endl;
  // }
  /* Setting up QProblem object. */
  QProblem qformula( n_contact*6, n_contact*6 );

  Options options;
  options.printLevel = PL_HIGH;
  qformula.setOptions( options );

  /* Solve first QP. */
  int nWSR = 100;
  real_t cputime = 1.0;
  qformula.init( qpH, qpG, qpA, qpLB, qpUB, qplbA, qpubA, nWSR, &cputime );
  // qformula.hotstart( qpH, qpG, qpA, qpLB, qpUB, qplbA, qpubA, nWSR, &cputime );
  // Get and print solution of first QP. 
  real_t xOpt[n_contact*3];
  qformula.getPrimalSolution( xOpt );
  for (int i=0; i<n_contact; i++)
    printf( "Leg %i, \t%f, \t%f, \t%f \n", i, xOpt[i*3], xOpt[i*3+1], xOpt[i*3+2]);
}
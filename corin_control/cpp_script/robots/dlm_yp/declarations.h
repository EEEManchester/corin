#ifndef IIT_ROBOT_DLM_YP_DECLARATIONS_H_
#define IIT_ROBOT_DLM_YP_DECLARATIONS_H_

#include <Eigen/Dense>

namespace iit {
namespace DLM_YP {

static const int JointSpaceDimension = 2;
static const int jointsCount = 2;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 3;

typedef Eigen::Matrix<double, 2, 1> Column2d;
typedef Column2d JointState;

enum JointIdentifiers {
    Q1 = 0
    , Q2
};

enum LinkIdentifiers {
    BASE = 0
    , L1
    , L2
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {Q1,Q2};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE,L1,L2};

}
}
#endif

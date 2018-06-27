#ifndef IIT_ROBOT_LCORIN_LEG_DECLARATIONS_H_
#define IIT_ROBOT_LCORIN_LEG_DECLARATIONS_H_

#include <Eigen/Dense>

namespace iit {
namespace LCORIN_LEG {

static const int JointSpaceDimension = 3;
static const int jointsCount = 3;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 4;

typedef Eigen::Matrix<double, 3, 1> Column3d;
typedef Column3d JointState;

enum JointIdentifiers {
    Q1 = 0
    , Q2
    , Q3
};

enum LinkIdentifiers {
    BASE = 0
    , L1
    , L2
    , L3
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {Q1,Q2,Q3};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE,L1,L2,L3};

}
}
#endif

#ifndef IIT_ROBOT_SLM_DECLARATIONS_H_
#define IIT_ROBOT_SLM_DECLARATIONS_H_

#include <Eigen/Dense>

namespace iit {
namespace SLM {

static const int JointSpaceDimension = 1;
static const int jointsCount = 1;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 2;

typedef Eigen::Matrix<double, 1, 1> Column1d;
typedef Column1d JointState;

enum JointIdentifiers {
    Q1 = 0
};

enum LinkIdentifiers {
    BASE = 0
    , L1
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {Q1};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE,L1};

}
}
#endif

#ifndef IIT_ROBOT_MCORIN_DECLARATIONS_H_
#define IIT_ROBOT_MCORIN_DECLARATIONS_H_

#include <Eigen/Dense>

namespace iit {
namespace mcorin {

static const int JointSpaceDimension = 18;
static const int jointsCount = 18;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 19;

typedef Eigen::Matrix<double, 18, 1> Column18d;
typedef Column18d JointState;

enum JointIdentifiers {
    LF_Q1_JOINT = 0
    , LF_Q2_JOINT
    , LF_Q3_JOINT
    , LM_Q1_JOINT
    , LM_Q2_JOINT
    , LM_Q3_JOINT
    , LR_Q1_JOINT
    , LR_Q2_JOINT
    , LR_Q3_JOINT
    , RF_Q1_JOINT
    , RF_Q2_JOINT
    , RF_Q3_JOINT
    , RM_Q1_JOINT
    , RM_Q2_JOINT
    , RM_Q3_JOINT
    , RR_Q1_JOINT
    , RR_Q2_JOINT
    , RR_Q3_JOINT
};

enum LinkIdentifiers {
    TRUNK = 0
    , LF_HIPASSEMBLY
    , LF_UPPERLEG
    , LF_LOWERLEG
    , LM_HIPASSEMBLY
    , LM_UPPERLEG
    , LM_LOWERLEG
    , LR_HIPASSEMBLY
    , LR_UPPERLEG
    , LR_LOWERLEG
    , RF_HIPASSEMBLY
    , RF_UPPERLEG
    , RF_LOWERLEG
    , RM_HIPASSEMBLY
    , RM_UPPERLEG
    , RM_LOWERLEG
    , RR_HIPASSEMBLY
    , RR_UPPERLEG
    , RR_LOWERLEG
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {LF_Q1_JOINT,LF_Q2_JOINT,LF_Q3_JOINT,LM_Q1_JOINT,LM_Q2_JOINT,LM_Q3_JOINT,LR_Q1_JOINT,LR_Q2_JOINT,LR_Q3_JOINT,RF_Q1_JOINT,RF_Q2_JOINT,RF_Q3_JOINT,RM_Q1_JOINT,RM_Q2_JOINT,RM_Q3_JOINT,RR_Q1_JOINT,RR_Q2_JOINT,RR_Q3_JOINT};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {TRUNK,LF_HIPASSEMBLY,LF_UPPERLEG,LF_LOWERLEG,LM_HIPASSEMBLY,LM_UPPERLEG,LM_LOWERLEG,LR_HIPASSEMBLY,LR_UPPERLEG,LR_LOWERLEG,RF_HIPASSEMBLY,RF_UPPERLEG,RF_LOWERLEG,RM_HIPASSEMBLY,RM_UPPERLEG,RM_LOWERLEG,RR_HIPASSEMBLY,RR_UPPERLEG,RR_LOWERLEG};

}
}
#endif

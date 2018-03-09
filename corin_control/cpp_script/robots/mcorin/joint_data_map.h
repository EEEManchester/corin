#ifndef IIT_MCORIN_JOINT_DATA_MAP_H_
#define IIT_MCORIN_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace mcorin {

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T> class JointDataMap {
private:
    T data[jointsCount];
public:
    JointDataMap() {};
    JointDataMap(const T& defaultValue);
    JointDataMap(const JointDataMap& rhs);
    JointDataMap& operator=(const JointDataMap& rhs);
    JointDataMap& operator=(const T& rhs);
          T& operator[](JointIdentifiers which);
    const T& operator[](JointIdentifiers which) const;
private:
    void copydata(const JointDataMap& rhs);
    void assigndata(const T& rhs);
};

template<typename T> inline
JointDataMap<T>::JointDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
JointDataMap<T>::JointDataMap(const JointDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& JointDataMap<T>::operator[](JointIdentifiers j) {
    return data[j];
}

template<typename T> inline
const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
    return data[j];
}

template<typename T> inline
void JointDataMap<T>::copydata(const JointDataMap& rhs) {
    data[LF_Q1_JOINT] = rhs[LF_Q1_JOINT];
    data[LF_Q2_JOINT] = rhs[LF_Q2_JOINT];
    data[LF_Q3_JOINT] = rhs[LF_Q3_JOINT];
    data[LM_Q1_JOINT] = rhs[LM_Q1_JOINT];
    data[LM_Q2_JOINT] = rhs[LM_Q2_JOINT];
    data[LM_Q3_JOINT] = rhs[LM_Q3_JOINT];
    data[LR_Q1_JOINT] = rhs[LR_Q1_JOINT];
    data[LR_Q2_JOINT] = rhs[LR_Q2_JOINT];
    data[LR_Q3_JOINT] = rhs[LR_Q3_JOINT];
    data[RF_Q1_JOINT] = rhs[RF_Q1_JOINT];
    data[RF_Q2_JOINT] = rhs[RF_Q2_JOINT];
    data[RF_Q3_JOINT] = rhs[RF_Q3_JOINT];
    data[RM_Q1_JOINT] = rhs[RM_Q1_JOINT];
    data[RM_Q2_JOINT] = rhs[RM_Q2_JOINT];
    data[RM_Q3_JOINT] = rhs[RM_Q3_JOINT];
    data[RR_Q1_JOINT] = rhs[RR_Q1_JOINT];
    data[RR_Q2_JOINT] = rhs[RR_Q2_JOINT];
    data[RR_Q3_JOINT] = rhs[RR_Q3_JOINT];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[LF_Q1_JOINT] = value;
    data[LF_Q2_JOINT] = value;
    data[LF_Q3_JOINT] = value;
    data[LM_Q1_JOINT] = value;
    data[LM_Q2_JOINT] = value;
    data[LM_Q3_JOINT] = value;
    data[LR_Q1_JOINT] = value;
    data[LR_Q2_JOINT] = value;
    data[LR_Q3_JOINT] = value;
    data[RF_Q1_JOINT] = value;
    data[RF_Q2_JOINT] = value;
    data[RF_Q3_JOINT] = value;
    data[RM_Q1_JOINT] = value;
    data[RM_Q2_JOINT] = value;
    data[RM_Q3_JOINT] = value;
    data[RR_Q1_JOINT] = value;
    data[RR_Q2_JOINT] = value;
    data[RR_Q3_JOINT] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   LF_q1_joint = "
    << map[LF_Q1_JOINT]
    << "   LF_q2_joint = "
    << map[LF_Q2_JOINT]
    << "   LF_q3_joint = "
    << map[LF_Q3_JOINT]
    << "   LM_q1_joint = "
    << map[LM_Q1_JOINT]
    << "   LM_q2_joint = "
    << map[LM_Q2_JOINT]
    << "   LM_q3_joint = "
    << map[LM_Q3_JOINT]
    << "   LR_q1_joint = "
    << map[LR_Q1_JOINT]
    << "   LR_q2_joint = "
    << map[LR_Q2_JOINT]
    << "   LR_q3_joint = "
    << map[LR_Q3_JOINT]
    << "   RF_q1_joint = "
    << map[RF_Q1_JOINT]
    << "   RF_q2_joint = "
    << map[RF_Q2_JOINT]
    << "   RF_q3_joint = "
    << map[RF_Q3_JOINT]
    << "   RM_q1_joint = "
    << map[RM_Q1_JOINT]
    << "   RM_q2_joint = "
    << map[RM_Q2_JOINT]
    << "   RM_q3_joint = "
    << map[RM_Q3_JOINT]
    << "   RR_q1_joint = "
    << map[RR_Q1_JOINT]
    << "   RR_q2_joint = "
    << map[RR_Q2_JOINT]
    << "   RR_q3_joint = "
    << map[RR_Q3_JOINT]
    ;
    return out;
}

}
}
#endif

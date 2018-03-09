#ifndef IIT_MCORIN_LINK_DATA_MAP_H_
#define IIT_MCORIN_LINK_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace mcorin {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[TRUNK] = rhs[TRUNK];
    data[LF_HIPASSEMBLY] = rhs[LF_HIPASSEMBLY];
    data[LF_UPPERLEG] = rhs[LF_UPPERLEG];
    data[LF_LOWERLEG] = rhs[LF_LOWERLEG];
    data[LM_HIPASSEMBLY] = rhs[LM_HIPASSEMBLY];
    data[LM_UPPERLEG] = rhs[LM_UPPERLEG];
    data[LM_LOWERLEG] = rhs[LM_LOWERLEG];
    data[LR_HIPASSEMBLY] = rhs[LR_HIPASSEMBLY];
    data[LR_UPPERLEG] = rhs[LR_UPPERLEG];
    data[LR_LOWERLEG] = rhs[LR_LOWERLEG];
    data[RF_HIPASSEMBLY] = rhs[RF_HIPASSEMBLY];
    data[RF_UPPERLEG] = rhs[RF_UPPERLEG];
    data[RF_LOWERLEG] = rhs[RF_LOWERLEG];
    data[RM_HIPASSEMBLY] = rhs[RM_HIPASSEMBLY];
    data[RM_UPPERLEG] = rhs[RM_UPPERLEG];
    data[RM_LOWERLEG] = rhs[RM_LOWERLEG];
    data[RR_HIPASSEMBLY] = rhs[RR_HIPASSEMBLY];
    data[RR_UPPERLEG] = rhs[RR_UPPERLEG];
    data[RR_LOWERLEG] = rhs[RR_LOWERLEG];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[TRUNK] = value;
    data[LF_HIPASSEMBLY] = value;
    data[LF_UPPERLEG] = value;
    data[LF_LOWERLEG] = value;
    data[LM_HIPASSEMBLY] = value;
    data[LM_UPPERLEG] = value;
    data[LM_LOWERLEG] = value;
    data[LR_HIPASSEMBLY] = value;
    data[LR_UPPERLEG] = value;
    data[LR_LOWERLEG] = value;
    data[RF_HIPASSEMBLY] = value;
    data[RF_UPPERLEG] = value;
    data[RF_LOWERLEG] = value;
    data[RM_HIPASSEMBLY] = value;
    data[RM_UPPERLEG] = value;
    data[RM_LOWERLEG] = value;
    data[RR_HIPASSEMBLY] = value;
    data[RR_UPPERLEG] = value;
    data[RR_LOWERLEG] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   trunk = "
    << map[TRUNK]
    << "   LF_hipassembly = "
    << map[LF_HIPASSEMBLY]
    << "   LF_upperleg = "
    << map[LF_UPPERLEG]
    << "   LF_lowerleg = "
    << map[LF_LOWERLEG]
    << "   LM_hipassembly = "
    << map[LM_HIPASSEMBLY]
    << "   LM_upperleg = "
    << map[LM_UPPERLEG]
    << "   LM_lowerleg = "
    << map[LM_LOWERLEG]
    << "   LR_hipassembly = "
    << map[LR_HIPASSEMBLY]
    << "   LR_upperleg = "
    << map[LR_UPPERLEG]
    << "   LR_lowerleg = "
    << map[LR_LOWERLEG]
    << "   RF_hipassembly = "
    << map[RF_HIPASSEMBLY]
    << "   RF_upperleg = "
    << map[RF_UPPERLEG]
    << "   RF_lowerleg = "
    << map[RF_LOWERLEG]
    << "   RM_hipassembly = "
    << map[RM_HIPASSEMBLY]
    << "   RM_upperleg = "
    << map[RM_UPPERLEG]
    << "   RM_lowerleg = "
    << map[RM_LOWERLEG]
    << "   RR_hipassembly = "
    << map[RR_HIPASSEMBLY]
    << "   RR_upperleg = "
    << map[RR_UPPERLEG]
    << "   RR_lowerleg = "
    << map[RR_LOWERLEG]
    ;
    return out;
}

}
}
#endif

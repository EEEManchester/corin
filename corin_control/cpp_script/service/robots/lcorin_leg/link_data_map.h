#ifndef IIT_LCORIN_LEG_LINK_DATA_MAP_H_
#define IIT_LCORIN_LEG_LINK_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace LCORIN_LEG {

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
    data[BASE] = rhs[BASE];
    data[L1] = rhs[L1];
    data[L2] = rhs[L2];
    data[L3] = rhs[L3];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[BASE] = value;
    data[L1] = value;
    data[L2] = value;
    data[L3] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   base = "
    << map[BASE]
    << "   l1 = "
    << map[L1]
    << "   l2 = "
    << map[L2]
    << "   l3 = "
    << map[L3]
    ;
    return out;
}

}
}
#endif

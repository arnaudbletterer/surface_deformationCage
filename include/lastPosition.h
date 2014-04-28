#ifndef LASTPOSITION_H
#define LASTPOSITION_H

#include "types.h"

namespace CGoGN
{
namespace SCHNApps
{

class LastPosition {

public :
    std::vector<PFP2::VEC3> m_lastPosition;

public :
    LastPosition(int i = 0)
    {}

    ~LastPosition()
    {}

    bool isInitialized()
    {
        return !m_lastPosition.empty();
    }

    static std::string CGoGNnameOfType()
    {
        return "LastPosition";
    }

};

}
}


#endif // LASTPOSITION_H

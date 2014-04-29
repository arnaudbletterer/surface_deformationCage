#ifndef _SPACEPOINT_H
#define _SPACEPOINT_H

#include "types.h"

#include <vector>

#include "Eigen/Geometry"

namespace CGoGN
{
namespace SCHNApps
{

class SpacePoint {

public :
    Eigen::Matrix<PFP2::REAL, 1, Eigen::Dynamic> m_cageWeightsEigen;

    PFP2::REAL m_cageBoundaryWeight;

private :
    Dart m_cageDart;

public :
    SpacePoint(int i = 0)
    {}

    ~SpacePoint()
    {}

    void setCageNbV(int n)
    {
        m_cageWeightsEigen.setZero(n);
    }

    void setCage(Dart d)
    {
        m_cageDart = d;
    }

    Dart getCageDart()
    {
        return m_cageDart;
    }

    bool isInitialized()
    {
        return m_cageDart != EMBNULL;
    }

    static std::string CGoGNnameOfType()
    {
        return "SpacePoint";
    }

};

}
}

#endif // SPACEPOINT_H

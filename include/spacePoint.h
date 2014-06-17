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
    std::vector<Eigen::Matrix<PFP2::REAL, 1, Eigen::Dynamic> > m_adjCagesWeights;

    std::vector<PFP2::REAL> m_cageBoundaryWeights;
    std::vector<Dart> m_adjCagesDart;

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

    void setNbAdjCages(int n)
    {
        m_adjCagesWeights.resize(n);
        m_cageBoundaryWeights.resize(n);
    }

    void setCage(Dart d)
    {
        m_cageDart = d;
    }

    Dart getCageDart()
    {
        return m_cageDart;
    }

    void addAdjacentCage(Dart d)
    {
        m_adjCagesDart.push_back(d);
    }

    void setAdjacentCages(const std::vector<Dart>& adj)
    {
        m_adjCagesDart = std::vector<Dart>(adj);
        setNbAdjCages(m_adjCagesDart.size());
    }

    bool isInitialized()
    {
        return !m_cageDart.isNil();
    }

    static std::string CGoGNnameOfType()
    {
        return "SpacePoint";
    }

};

}
}

#endif // SPACEPOINT_H

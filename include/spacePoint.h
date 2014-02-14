#ifndef SPACEPOINT_H
#define SPACEPOINT_H

#include "types.h"

#include <vector>

#include "Eigen/Geometry"

namespace CGoGN
{
namespace SCHNApps
{

class SpacePoint {

public :
    Eigen::Matrix<PFP2::REAL, Eigen::Dynamic, 1> m_cageWeightsEigen;

    std::vector<Eigen::Matrix<PFP2::REAL, Eigen::Dynamic, 1> > m_adjCagesCoordinates;
    std::vector<Dart> m_adjCagesDart;
    std::vector<PFP2::REAL> m_adjCagesWeights;

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
        m_adjCagesCoordinates.resize(n);
        m_adjCagesWeights.resize(n);
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
        m_adjCagesDart = adj;
        setNbAdjCages(m_adjCagesDart.size());
    }

    Dart getAdjacentCage(int index)
    {
        return m_adjCagesDart[index];
    }

    void addAdjacentCageWeight(PFP2::REAL w)
    {
        m_adjCagesWeights.push_back(w);
    }

    Dart getAdjacentCageWeight(int index)
    {
        return m_adjCagesWeights[index];
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

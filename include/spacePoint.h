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

private :
    std::vector<Dart> m_cagesDarts;
    std::vector<Eigen::Matrix<PFP2::REAL, 1, Eigen::Dynamic> > m_cagesWeightsEigen;
    std::vector<PFP2::REAL> m_cagesBoundaryWeights;

public :
    SpacePoint(int i = 0)
    {}

    ~SpacePoint()
    {}

    void setCage(Dart d, int n)
    {
        m_cagesDarts.push_back(d);
        Eigen::Matrix<PFP2::REAL, 1, Eigen::Dynamic> weights;
        weights.resize(n);
        m_cagesWeightsEigen.push_back(weights);
    }

    Dart getCageDart(int index)
    {
        return m_cagesDarts[index];
    }

    int getNbAssociatedCages()
    {
        return m_cagesDarts.size();
    }

    Eigen::Matrix<PFP2::REAL, 1, Eigen::Dynamic>& getCageWeights(int index)
    {
        if(index >= 0 && index < m_cagesWeightsEigen.size())
        {
            return m_cagesWeightsEigen[index];
        }
    }

    PFP2::REAL getCageBoundaryWeight(int index)
    {
        if(index >= 0 && index < m_cagesWeightsEigen.size())
        {
            return m_cagesBoundaryWeights[index];
        }
    }

    int getCageIndex(Dart d)
    {
        for(int i = 0; i < m_cagesDarts.size(); ++i)
        {
            if(d == m_cagesDarts[i])
            {
                return i;
            }
        }
        return -1;
    }

    bool isInitialized()
    {
        return m_cagesDarts.size() == 0;
    }

    static std::string CGoGNnameOfType()
    {
        return "SpacePoint";
    }

};

}
}

#endif // SPACEPOINT_H

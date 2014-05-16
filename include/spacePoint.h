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

    void removeCages()
    {
        m_cagesDarts.clear();
        m_cagesWeightsEigen.clear();
        m_cagesBoundaryWeights.clear();
    }

    void addCage(Dart d, int n)
    {
        m_cagesDarts.push_back(d);
        Eigen::Matrix<PFP2::REAL, 1, Eigen::Dynamic> weights;
        weights.resize(n);
        m_cagesWeightsEigen.push_back(weights);
        m_cagesBoundaryWeights.push_back(-1.f);
    }

    Dart getCageDart(unsigned int index)
    {
        if(index < m_cagesDarts.size())
        {
            return m_cagesDarts[index];
        }
        return Dart::nil();
    }

    int getNbAssociatedCages()
    {
        return m_cagesDarts.size();
    }

    Eigen::Matrix<PFP2::REAL, 1, Eigen::Dynamic>* getCageWeights(unsigned int index)
    {
        if(index < m_cagesWeightsEigen.size())
        {
            return &(m_cagesWeightsEigen[index]);
        }
        return NULL;
    }

    PFP2::REAL getCageBoundaryWeight(unsigned int index)
    {
        if(index < m_cagesWeightsEigen.size())
        {
            return m_cagesBoundaryWeights[index];
        }
        return -1.f;
    }

    void setCageBoundaryWeight(unsigned int index, PFP2::REAL value)
    {
        if(index < m_cagesBoundaryWeights.size())
        {
            m_cagesBoundaryWeights[index] = value;
        }
    }

    int getCageIndex(Dart dart)
    {
        for(unsigned int i = 0; i < m_cagesDarts.size(); ++i)
        {
            if(dart == m_cagesDarts[i])
            {
                return i;
            }
        }
        return -1;
    }

    bool isInitialized()
    {
        return m_cagesDarts.size() > 0;
    }

    static std::string CGoGNnameOfType()
    {
        return "SpacePoint";
    }

};

}
}

#endif // SPACEPOINT_H

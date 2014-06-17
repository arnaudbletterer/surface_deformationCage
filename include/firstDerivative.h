#ifndef _FIRSTDERIVATIVE_H
#define _FIRSTDERIVATIVE_H

#include "types.h"

#include "Eigen/Geometry"

namespace CGoGN
{
namespace SCHNApps
{

class FirstDerivative {

public :
    std::vector<Eigen::MatrixXf> m_verticesDerivatives;

private :
    Dart m_beginningDart;

public :
    FirstDerivative(int i = 0)
    {}

    ~FirstDerivative()
    {}

    Dart getBeginningDart()
    {
        return m_beginningDart;
    }

    void setBeginningDart(Dart d)
    {
        m_beginningDart = d;
    }

    void setNbVertices(int n)
    {
        m_verticesDerivatives.resize(n);
        for(int i = 0; i < m_verticesDerivatives.size(); ++i)
        {
            m_verticesDerivatives[i].setZero(2, 2); //Pour l'instant les déformations sont faites en 2 dimensions
        }
    }

    bool isInitialized()
    {
        return !m_beginningDart.isNil();
    }

    static std::string CGoGNnameOfType()
    {
        return "FirstDerivative";
    }

};

}
}

#endif // FIRSTDERIVATIVE_H

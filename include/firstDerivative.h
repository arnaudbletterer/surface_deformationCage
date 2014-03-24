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
    Eigen::Matrix<PFP2::REAL, 2, Eigen::Dynamic> m_verticesDerivatives;

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
        m_verticesDerivatives.setZero(2, n);
    }

    bool isInitialized()
    {
        return m_beginningDart == EMBNULL;
    }

    static std::string CGoGNnameOfType()
    {
        return "FirstDerivative";
    }

};

}
}

#endif // FIRSTDERIVATIVE_H

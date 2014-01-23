#ifndef VCAGE_H
#define VCAGE_H

#include <vector>
#include <algorithm>

#include "Topology/generic/dart.h"

namespace CGoGN
{
namespace SCHNApps
{

/*
* Classe définissant l'attribut de sommet VCage, indiquant les cages englobant le sommet courant
*/
class VCage {
   public:
    VCage(int i=0)
        :   m_cage()
        {}

    void addVertex(Dart vertex)
    {
        if(std::find(m_cage.begin(), m_cage.end(), vertex) == m_cage.end())
        {
            m_cage.push_back(vertex);
        }
    }

    Dart operator[](unsigned int index)
    {
        if(index<m_cage.size())
        {
            return m_cage[index];
        }
    }

    std::vector<Dart> getCage()
    {
        return m_cage;
    }

    static std::string CGoGNnameOfType()
    {
        return "VCage";
    }

    private:
        std::vector<Dart> m_cage;
};

} //namespace SCHNApps
} //namespace CGoGN

#endif // VCAGE_H

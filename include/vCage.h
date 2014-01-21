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
        :   m_cages(1), m_idCages(1)
        {}

    void addVertex(Dart vertex, unsigned int index)
    {
        if(m_cages.size()>index)
        {
            m_cages[index].push_back(vertex);
        }
    }

    void addNewCage()
    {
        m_cages.push_back(std::vector<Dart>());
    }

    std::vector<Dart> getCage(unsigned int index)
    {
        if(m_cages.size()>index)
        {
            return m_cages[index];
        }
        return std::vector<Dart>();
    }

    std::vector<int> getIds()
    {
        return m_idCages;
    }

    void addId(int id)
    {
        m_idCages.push_back(id);
    }

    int size()
    {
        return m_cages.size();
    }

    int sizeOfCage(unsigned int index)
    {
        if(m_cages.size()>index)
        {
            return m_cages[index].size();
        }
        return -1;
    }

    static std::string CGoGNnameOfType()
    {
        return "VCages";
    }

    private:
        std::vector<std::vector<Dart> > m_cages;    //Vecteur de cages contenant le sommet
        std::vector<int> m_idCages;  //Identifiant de la cage composée du sommet
};

} //namespace SCHNApps
} //namespace CGoGN

#endif // VCAGE_H

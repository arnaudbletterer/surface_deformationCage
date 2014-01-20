#ifndef IDCAGE_H
#define IDCAGE_H

#include <vector>
#include <algorithm>

namespace CGoGN
{
namespace SCHNApps
{

/*
* Classe définissant l'attribut de sommet Voxel, définissant l'affectation à un voxel
*/
class IdCage {
   public:
    IdCage(int id = -1)
        :   m_idCage(id)
        {}

    void addId(int id)
    {
        if(std::find(m_idCage.begin(), m_idCage.end(),id) == m_idCage.end())
        {
            m_idCage.push_back(id);
        }
    }

    int getId(int index)
    {
        return m_idCage[index];
    }

    bool contains(int id)
    {
        return (std::find(m_idCage.begin(), m_idCage.end(),id) == m_idCage.end());
    }

    static std::string CGoGNnameOfType()
    {
        return "Id_Cage" ;
    }

    private:
         std::vector<int> m_idCage;
};

} //namespace SCHNApps
} //namespace CGoGN

#endif // IDCAGE_H

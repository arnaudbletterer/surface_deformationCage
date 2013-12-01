#ifndef MVCCOORDINATES_H
#define MVCCOORDINATES_H

#include <vector>
#include <string>

namespace CGoGN
{
namespace SCHNApps
{

/*
* Classe définissant l'attribut de sommet MVCCoordinates, définissant les coordonnées MVC associées au sommet
*/
class MVCCoordinates {
   public:
    MVCCoordinates()
        :   m_coordinates()
        {}
    MVCCoordinates(int i)
        :   m_coordinates()
        {}

    std::vector<PFP2::REAL>& getCoordinates()
    {
        return m_coordinates;
    }

    void setCoordinates(const std::vector<PFP2::REAL>& m)
    {
        m_coordinates = m;
    }

    bool isCalculated()
    {
        return m_coordinates.size()!=0;
    }

    void clear()
    {
        m_coordinates.clear();
    }

    void push_back(PFP2::REAL v)
    {
        m_coordinates.push_back(v);
    }

    void reserve(unsigned int i)
    {
        m_coordinates.reserve(i);
    }

    PFP2::REAL& operator[](int i) {
        return m_coordinates[i];
    }

    static std::string CGoGNnameOfType()
    {
        return "MVCCoordinates" ;
    }

private:
    std::vector<PFP2::REAL> m_coordinates;
};

} //namespace SCHNApps
} //namespace CGoGN

#endif // MVCCOORDINATES_H

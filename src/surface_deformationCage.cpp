#include "surface_deformationCage.h"

namespace CGoGN
{

namespace SCHNApps
{

MapParameters::MapParameters() :
    m_initialized(false),
    m_linked(false),
    m_toComputeMVC(true),
    m_coordinates()
{}

MapParameters::~MapParameters() {}

void MapParameters::start() {
    if(!m_initialized) {
        m_initialized = true;
    }
}

void MapParameters::stop() {
    if(m_initialized) {
        m_initialized = false;
    }
}

bool Surface_DeformationCage_Plugin::enable()
{
    m_deformationCageDialog = new Dialog_DeformationCage(m_schnapps, this);

    m_deformationCageAction = new QAction("Cage deformation", this);

    m_schnapps->addMenuAction(this, "Surface;Cage deformation", m_deformationCageAction);

    connect(m_deformationCageAction, SIGNAL(triggered()), this, SLOT(openDeformationCageDialog()));

    connect(m_schnapps, SIGNAL(mapAdded(MapHandlerGen*)), this, SLOT(mapAdded(MapHandlerGen*)));
    connect(m_schnapps, SIGNAL(mapRemoved(MapHandlerGen*)), this, SLOT(mapRemoved(MapHandlerGen*)));

    foreach(MapHandlerGen* map, m_schnapps->getMapSet().values())
        mapAdded(map);

    return true;
}

void Surface_DeformationCage_Plugin::disable()
{
    disconnect(m_deformationCageAction, SIGNAL(triggered()), this, SLOT(openGenerationCageDialog()));

    disconnect(m_schnapps, SIGNAL(mapAdded(MapHandlerGen*)), this, SLOT(mapAdded(MapHandlerGen*)));
    disconnect(m_schnapps, SIGNAL(mapRemoved(MapHandlerGen*)), this, SLOT(mapRemoved(MapHandlerGen*)));
}

void Surface_DeformationCage_Plugin::mapAdded(MapHandlerGen *map)
{
    connect(map, SIGNAL(attributeModified(unsigned int, QString)), this, SLOT(attributeModified(unsigned int, QString)));
}

void Surface_DeformationCage_Plugin::mapRemoved(MapHandlerGen *map)
{
    disconnect(map, SIGNAL(attributeModified(unsigned int, QString)), this, SLOT(attributeModified(unsigned int, QString)));
}

void Surface_DeformationCage_Plugin::attributeModified(unsigned int orbit, QString nameAttr)
{
    if(orbit==VERTEX) {
        MapHandlerGen* mhg_modified = static_cast<MapHandlerGen*>(QObject::sender());
        MapSet maps = m_schnapps->getMapSet();
        AttributeSet attributes;
        for(MapSet::iterator it = maps.begin(); it!=maps.end(); ++it)
        {
            MapHandlerGen* mhg_current = static_cast<MapHandlerGen*>(m_schnapps->getMap(it.key()));
            attributes = mhg_current->getAttributeSet(orbit);
            foreach(QString attribute, attributes)
            {
                //On considère d'abord la carte modifiée comme étant la cage
                MapParameters& p = h_parameterSet[mhg_current->getName()+attribute+mhg_modified->getName()+nameAttr];
                if(p.m_initialized)
                {   //Si des coordonnées ont été calculées
                    if(p.m_linked)
                    {   //Si les deux cartes sont actuellement liées
                        moveObjectsPointsFromCageMovement(mhg_current, mhg_modified, attribute, nameAttr);
                    }
                }

                //On considère ensuite la carte modifiée comme étant l'objet
                p = h_parameterSet[mhg_modified->getName()+nameAttr+mhg_current->getName()+attribute];
                if(p.m_initialized)
                {   //Si des coordonnées ont été calculées
                    if(p.m_linked)
                    {   //Si les deux cartes sont actuellement liées
                        moveObjectsPointsFromCageMovement(mhg_modified, mhg_current, nameAttr, attribute);
                    }
                }
            }
        }
    }
}

void Surface_DeformationCage_Plugin::moveObjectsPointsFromCageMovement(MapHandlerGen* o, MapHandlerGen* c, const QString& objectNameAttr, const QString& cageNameAttr)
{
    MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(o);
    PFP2::MAP* object = mh_object->getMap();
    MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(c);
    PFP2::MAP* cage = mh_cage->getMap();

    VertexAttribute<PFP2::VEC3> objectPosition = object->getAttribute<PFP2::VEC3, VERTEX>(objectNameAttr.toStdString());
    VertexAttribute<PFP2::VEC3> cagePosition = cage->getAttribute<PFP2::VEC3, VERTEX>(cageNameAttr.toStdString());
    VertexAttribute<MVCCoordinates> objectCoordinates = object->getAttribute<MVCCoordinates, VERTEX>("MVCCoordinates");

    TraversorV<PFP2::MAP> trav_vert_object(*object);
    TraversorV<PFP2::MAP> trav_vert_cage(*cage);
    for(Dart d = trav_vert_object.begin(); d!=trav_vert_object.end(); d = trav_vert_object.next())
    {   //Pour chaque sommet de l'objet
        objectPosition[d] = PFP2::VEC3(0);
        unsigned int j = 0;
        for(Dart dd = trav_vert_cage.begin(); dd!=trav_vert_cage.end(); dd = trav_vert_cage.next())
        {
            objectPosition[d] =+ objectCoordinates[d][j]*cagePosition[dd];
            ++j;
        }
    }
}

void Surface_DeformationCage_Plugin::openDeformationCageDialog()
{
    m_deformationCageDialog->updateAppearanceFromPlugin();
    m_deformationCageDialog->show();
}

void Surface_DeformationCage_Plugin::computeMVCFromDialog()
{
    MapHandlerGen* mhg_object = m_deformationCageDialog->getSelectedObject();
    MapHandlerGen* mhg_cage = m_deformationCageDialog->getSelectedCage();
    const QString objectNameAttr = m_deformationCageDialog->combo_objectPositionAttribute->currentText();
    const QString cageNameAttr = m_deformationCageDialog->combo_cagePositionAttribute->currentText();
    if(mhg_object && mhg_cage && !objectNameAttr.isEmpty() && !cageNameAttr.isEmpty())
    {
        computeAllPointsFromObject(mhg_object->getName(), mhg_cage->getName(), objectNameAttr, cageNameAttr);
    }
}

void Surface_DeformationCage_Plugin::computeAllPointsFromObject(const QString& objectName, const QString& cageName, const QString& objectNameAttr, const QString& cageNameAttr)
{
    MapParameters& p = h_parameterSet[objectName+objectNameAttr+cageName+cageNameAttr];
    if(!p.m_initialized)
        p.start();
    if(p.m_toComputeMVC)
    {   //Si les coordonnées sont à calculer
        MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(m_schnapps->getMap(objectName));
        PFP2::MAP* object = mh_object->getMap();
        MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(m_schnapps->getMap(cageName));
        PFP2::MAP* cage = mh_cage->getMap();

        VertexAttribute<MVCCoordinates> coordinates = object->getAttribute<MVCCoordinates, VERTEX>("MVCCoordinates");

        if(!coordinates.isValid())
            coordinates = object->addAttribute<MVCCoordinates, VERTEX>("MVCCoordinates");

        VertexAttribute<PFP2::VEC3> position = object->getAttribute<PFP2::VEC3, VERTEX>(objectNameAttr.toStdString());

        if(!position.isValid())
        {
            CGoGNout << "Position attribute chosen for the object isn't valid" << CGoGNendl;
            return;
        }

        TraversorV<PFP2::MAP> trav_vert_object(*object);
        int i=0;
        for(Dart it = trav_vert_object.begin(); it!=trav_vert_object.end(); it = trav_vert_object.next())
        {
            if(i%1000==0)
                CGoGNout << i << CGoGNendl;
            computePointMVCFromCage(it, objectName, cageName, cageNameAttr, position, coordinates, object, cage);
            ++i;
        }
        p.m_toComputeMVC = false;
    }
}

/*
  * Fonction qui calcule les coordonnées MVC d'un point par rapport à une cage
  */
void Surface_DeformationCage_Plugin::computePointMVCFromCage(Dart vertex, const QString& objectName, const QString& cageName,
                                                             const QString& cageNameAttr, VertexAttribute<PFP2::VEC3> position,
                                                             VertexAttribute<MVCCoordinates> coordinates, PFP2::MAP* object,
                                                             PFP2::MAP* cage)
{
    PFP2::REAL c;
    PFP2::REAL sumMVC(0);
    unsigned int j=0;

    TraversorV<PFP2::MAP> trav_vert_cage(*cage);

    for(Dart it = trav_vert_cage.begin(); it!=trav_vert_cage.end(); it = trav_vert_cage.next())
    {
        ++j;
        sumMVC += c;
    }

    coordinates[vertex].setCoordinates(std::vector<PFP2::REAL>(j));

    for(Dart it = trav_vert_cage.begin(); it!=trav_vert_cage.end(); it = trav_vert_cage.next())
    {   //On parcourt l'ensemble des sommets de la cage
        c = computeMVC(position[vertex], it, cage, cageNameAttr);
        coordinates[vertex][c/sumMVC];
    }
}

PFP2::REAL Surface_DeformationCage_Plugin::computeMVC(PFP2::VEC3 p, Dart vertex, PFP2::MAP* cage, const QString& cageNameAttr)
{
    VertexAttribute<PFP2::VEC3> position = cage->getAttribute<PFP2::VEC3, VERTEX>(cageNameAttr.toStdString()) ;
    PFP2::REAL r = (position[vertex]-p).norm();

    PFP2::REAL sumU(0.);
    Dart it = vertex;
    do
    {
        PFP2::VEC3 vi = position[it];
        PFP2::VEC3 vj = position[cage->phi1(it)];
        PFP2::VEC3 vk = position[cage->phi_1(it)];


        PFP2::REAL Bjk = Geom::angle((vj-p),(vk-p));
        PFP2::REAL Bij = Geom::angle((vi-p),(vj-p));
        PFP2::REAL Bki = Geom::angle((vk-p),(vi-p));

        PFP2::VEC3 ei = (vi-p)/((vi-p).norm());
        PFP2::VEC3 ej = (vj-p)/((vj-p).norm());
        PFP2::VEC3 ek = (vk-p)/((vk-p).norm());

        PFP2::VEC3 nij = (ei^ej)/((ei^ej).norm());
        PFP2::VEC3 njk = (ej^ek)/((ej^ek).norm());
        PFP2::VEC3 nki = (ek^ei)/((ek^ei).norm());

        PFP2::REAL ui= (Bjk + (Bij*(nij*njk)) + (Bki*(nki*njk)))/(2.0f*ei*njk);

        sumU+=ui;

        it = cage->phi<21>(it);
    }
    while(it!=vertex);

    return (1.0f/r)*sumU;
}


#ifndef DEBUG
Q_EXPORT_PLUGIN2(Surface_DeformationCage_Plugin, Surface_DeformationCage_Plugin)
#else
Q_EXPORT_PLUGIN2(Surface_DeformationCage_PluginD, Surface_DeformationCage_Plugin)
#endif

} // namespace SCHNApps

} // namespace CGoGN

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
            for(AttributeSet::const_iterator attribute = attributes.constBegin(); attribute != attributes.constEnd(); ++attribute)
            {
                //On considère la carte modifiée comme étant la cage
                MapParameters& p = h_parameterSet[mhg_current->getName()+attribute.key()+mhg_modified->getName()+nameAttr];
                if(p.m_initialized && !p.m_toComputeMVC)
                {   //Si des coordonnées ont été calculées
                    if(p.m_linked)
                    {   //Si les deux cartes sont actuellement liées
                        CGoGNout << "Attribute modified catched" << CGoGNendl;
                        moveObjectsPointsFromCageMovement(mhg_current, mhg_modified, attribute.key(), nameAttr);
                    }
                    else
                    {
                        p.m_toComputeMVC = true;
                    }
                }

                //On considère la carte modifiée comme étant l'objet
                p = h_parameterSet[mhg_modified->getName()+nameAttr+mhg_current->getName()+attribute.key()];
                if(p.m_initialized && !p.m_toComputeMVC)
                {   //Si des coordonnées ont été calculées
                    if(p.m_linked)
                    {   //Si les deux cartes sont actuellement liées
                        CGoGNout << "Attribute modified catched" << CGoGNendl;
                        moveObjectsPointsFromCageMovement(mhg_modified, mhg_current, nameAttr, attribute.key());
                    }
                    else
                    {
                        p.m_toComputeMVC = true;
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
    VertexAttribute<MVCCoordinates> objectCoordinates = object->getAttribute<MVCCoordinates, VERTEX>("MVCCoordinates");

    VertexAttribute<PFP2::VEC3> cagePosition = cage->getAttribute<PFP2::VEC3, VERTEX>(cageNameAttr.toStdString());

    TraversorV<PFP2::MAP> trav_vert_object(*object);
    for(Dart d = trav_vert_object.begin(); d!=trav_vert_object.end(); d = trav_vert_object.next())
    {   //Pour chaque sommet de l'objet
        objectPosition[d] = PFP2::VEC3(0);
        unsigned int j = 0;
        TraversorV<PFP2::MAP> trav_vert_cage(*cage);
        for(Dart dd = trav_vert_cage.begin(); dd!=trav_vert_cage.end(); dd = trav_vert_cage.next())
        {
            objectPosition[d] += cagePosition[dd]*objectCoordinates[d][j];
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

        VertexAttribute<PFP2::VEC3> positionCage = cage->getAttribute<PFP2::VEC3, VERTEX>(cageNameAttr.toStdString());
        if(!positionCage.isValid())
        {
            CGoGNout << "Position attribute chosen for the cage isn't valid" << CGoGNendl;
            return;
        }

        VertexAttribute<PFP2::VEC3> positionObject = object->getAttribute<PFP2::VEC3, VERTEX>(objectNameAttr.toStdString());
        if(!positionObject.isValid())
        {
            CGoGNout << "Position attribute chosen for the object isn't valid" << CGoGNendl;
            return;
        }

        VertexAttribute<MVCCoordinates> coordinates = object->getAttribute<MVCCoordinates, VERTEX>("MVCCoordinates");
        if(!coordinates.isValid())
        {
            coordinates = object->addAttribute<MVCCoordinates, VERTEX>("MVCCoordinates");
        }

        Utils::Chrono chrono;
        chrono.start();

        TraversorV<PFP2::MAP> trav_vert_object(*object);
        for(Dart d = trav_vert_object.begin(); d!=trav_vert_object.end(); d = trav_vert_object.next())
        {
            computePointMVCFromCage(positionObject[d], d, objectName, cageName, positionCage, coordinates);
        }

        CGoGNout << "Temps de calcul des coordonnées MVC : " << chrono.elapsed() << " ms." << CGoGNendl;

        p.m_toComputeMVC = false;
    }
}

/*
  * Fonction qui calcule les coordonnées MVC d'un point par rapport à une cage
  */
void Surface_DeformationCage_Plugin::computePointMVCFromCage(PFP2::VEC3 pt, Dart vertex, const QString& objectName, const QString& cageName,
                                                             VertexAttribute<PFP2::VEC3> position, VertexAttribute<MVCCoordinates> coordinates)
{
    MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(m_schnapps->getMap(cageName));
    PFP2::MAP* cage = mh_cage->getMap();

    PFP2::REAL c;
    PFP2::REAL sumMVC(0);
    unsigned int j=0;

    coordinates[vertex].reserve(cage->getNbOrbits<VERTEX>());

    TraversorV<PFP2::MAP> trav_vert_cage(*cage);
    for(Dart d = trav_vert_cage.begin(); d!=trav_vert_cage.end(); d = trav_vert_cage.next())
    {   //On calcule les coordonnées par rapport à chaque sommet de la cage
        c = computeMVC(pt, d, cage, position);
        coordinates[vertex].push_back(c);
        sumMVC += c;
        ++j;
    }

    for(unsigned int i=0; i<j; ++i)
    {
        coordinates[vertex][i] /= sumMVC;
    }
}

PFP2::REAL Surface_DeformationCage_Plugin::computeMVC(PFP2::VEC3 pt, Dart vertex, PFP2::MAP* cage, VertexAttribute<PFP2::VEC3> position)
{
    PFP2::REAL r = (position[vertex]-pt).norm();

    PFP2::REAL sumU(0.);
    Dart it = vertex;
    do
    {
        PFP2::VEC3 vi = position[it];
        PFP2::VEC3 vj = position[cage->phi1(it)];
        PFP2::VEC3 vk = position[cage->phi_1(it)];

        PFP2::REAL Bjk = Geom::angle((vj-pt),(vk-pt));
        PFP2::REAL Bij = Geom::angle((vi-pt),(vj-pt));
        PFP2::REAL Bki = Geom::angle((vk-pt),(vi-pt));

        PFP2::VEC3 ei = (vi-pt)/((vi-pt).norm());
        PFP2::VEC3 ej = (vj-pt)/((vj-pt).norm());
        PFP2::VEC3 ek = (vk-pt)/((vk-pt).norm());

        PFP2::VEC3 eiej = ei^ej;
        PFP2::VEC3 ejek = ej^ek;
        PFP2::VEC3 ekei = ek^ei;

        PFP2::VEC3 nij = eiej/(eiej.norm());
        PFP2::VEC3 njk = ejek/(ejek.norm());
        PFP2::VEC3 nki = ekei/(ekei.norm());

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

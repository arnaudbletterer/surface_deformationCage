#include "surface_deformationCage.h"

namespace CGoGN
{

namespace SCHNApps
{

MapParameters::MapParameters() :
    m_initialized(false),
    m_linked(false),
    m_toComputeMVC(true)
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
    if(orbit==VERTEX)
    {
        MapHandlerGen* mhg_modified = static_cast<MapHandlerGen*>(QObject::sender());
        if(h_cageParameters.contains(mhg_modified))
        {
            //Si la carte venant d'être modifiée est une cage
            CageParameters& p = h_cageParameters[mhg_modified];
            if(p.cagePosition.name()==nameAttr.toStdString())
            {   //Si l'attribut venant d'être modifié est celui qui avait été utilisé lors de la liaison avec l'objet

                int i = 0;
                MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(mhg_modified);
                PFP2::MAP* cage = mh_cage->getMap();

                TraversorV<PFP2::MAP> trav_vert_cage(*cage);
                for(Dart d = trav_vert_cage.begin(); d!=trav_vert_cage.end(); d= trav_vert_cage.next())
                {
                    p.cagePositionEigen(i, 0) = p.cagePosition[d][0];
                    p.cagePositionEigen(i, 1) = p.cagePosition[d][1];
                    p.cagePositionEigen(i, 2) = p.cagePosition[d][2];
                    ++i;
                }

                MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(p.controlledObject);
                PFP2::MAP* object = mh_object->getMap();

                p.objectPositionEigen = p.coordinatesEigen*p.cagePositionEigen;

                i = 0;
                TraversorV<PFP2::MAP> trav_vert_object(*object);
                for(Dart d = trav_vert_object.begin(); d!=trav_vert_object.end(); d= trav_vert_object.next())
                {
                    p.controlledObjectPosition[d][0] = p.objectPositionEigen(i, 0);
                    p.controlledObjectPosition[d][1] = p.objectPositionEigen(i, 1);
                    p.controlledObjectPosition[d][2] = p.objectPositionEigen(i, 2);
                    ++i;
                }

                mh_object->updateBB(p.controlledObjectPosition);
                mh_object->notifyAttributeModification(p.controlledObjectPosition);
                mh_object->notifyConnectivityModification();
            }
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

void setProgressBarValue(int value, QProgressBar* progress)
{
    progress->setValue(value);
}

void Surface_DeformationCage_Plugin::computeAllPointsFromObject(const QString& objectName, const QString& cageName, const QString& objectNameAttr, const QString& cageNameAttr)
{
    if(!h_cageParameters.contains(m_schnapps->getMap(cageName)))
    {   //Si la carte n'est pas encore liée à un objet
        CageParameters& p = h_cageParameters[m_schnapps->getMap(cageName)];
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

        p.cagePosition = positionCage;

        VertexAttribute<PFP2::VEC3> positionObject = object->getAttribute<PFP2::VEC3, VERTEX>(objectNameAttr.toStdString());
        if(!positionObject.isValid())
        {
            CGoGNout << "Position attribute chosen for the object isn't valid" << CGoGNendl;
            return;
        }

        p.controlledObject = m_schnapps->getMap(objectName);
        p.controlledObjectPosition = positionObject;

        Utils::Chrono chrono;
        chrono.start();

        unsigned int cageNbV = cage->getNbOrbits<VERTEX>();

        unsigned int objectNbV = object->getNbOrbits<VERTEX>();

        p.coordinatesEigen.resize(objectNbV, cageNbV);
        p.cagePositionEigen.resize(cageNbV,3);
        p.objectPositionEigen.resize(objectNbV,3);

        int i = 0;
        TraversorV<PFP2::MAP> trav_vert_cage(*cage);
        for(Dart d = trav_vert_cage.begin(); d != trav_vert_cage.end(); d = trav_vert_cage.next())
        {
            p.cagePositionEigen(i, 0) = positionCage[d][0];
            p.cagePositionEigen(i, 1) = positionCage[d][1];
            p.cagePositionEigen(i, 2) = positionCage[d][2];
            ++i;
        }

        i = 0;
        TraversorV<PFP2::MAP> trav_vert_object(*object);
        float incr = 100/objectNbV;
        float total = 0.;

        QFuture<void> future = QtConcurrent::run(setProgressBarValue, m_deformationCageDialog->progress_link->value()+(int)total, m_deformationCageDialog->progress_link);

        for(Dart d = trav_vert_object.begin(); d!=trav_vert_object.end(); d = trav_vert_object.next())
        {
            p.objectPositionEigen(i, 0) = positionObject[d][0];
            p.objectPositionEigen(i, 1) = positionObject[d][1];
            p.objectPositionEigen(i, 2) = positionObject[d][2];
            computePointMVCFromCage(positionObject[d], cage, cageNbV, positionCage, p.coordinatesEigen, i);
            m_deformationCageDialog->progress_link->setValue(m_deformationCageDialog->progress_link->value()+(int)total);
            m_deformationCageDialog->update();
            total += incr;
            ++i;
        }

        CGoGNout << "Temps de calcul des coordonnées MVC : " << chrono.elapsed() << " ms." << CGoGNendl;
    }
}

/*
  * Fonction qui calcule les coordonnées MVC d'un point par rapport à une cage
  */
    void Surface_DeformationCage_Plugin::computePointMVCFromCage(const PFP2::VEC3& pt, PFP2::MAP* cage, unsigned int cageNbV,
                                                                 const VertexAttribute<PFP2::VEC3>& position, Eigen::MatrixXf& coordinates, int index)
{
    PFP2::REAL c;
    PFP2::REAL sumMVC(0);

    int i = 0;
    TraversorV<PFP2::MAP> trav_vert_cage(*cage);
    for(Dart d = trav_vert_cage.begin(); d != trav_vert_cage.end(); d = trav_vert_cage.next())
    {
        //On calcule les coordonnées par rapport à chaque sommet de la cage
        c = computeMVC(pt, d, cage, position);
        coordinates(index, i) = c;
        sumMVC += c;
        ++i;
    }

    for(unsigned int i=0; i<cageNbV; ++i)
    {
        coordinates(index, i) /= sumMVC;
    }
}

PFP2::REAL Surface_DeformationCage_Plugin::computeMVC(const PFP2::VEC3& pt, Dart vertex, PFP2::MAP* cage, const VertexAttribute<PFP2::VEC3>& position)
{
    PFP2::REAL r = (position[vertex]-pt).norm();

    PFP2::REAL sumU(0.);
    Dart it = vertex;
    do
    {
        PFP2::VEC3 vi = position[it];
        PFP2::VEC3 vj = position[cage->phi1(it)];
        PFP2::VEC3 vk = position[cage->phi_1(it)];

        std::vector<PFP2::VEC3> tmp(2);

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

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

    m_colorPerVertexShader = new CGoGN::Utils::ShaderColorPerVertex();
    registerShader(m_colorPerVertexShader);

    m_positionVBO = new Utils::VBO();
    m_colorVBO = new Utils::VBO();

    m_toDraw = false;

    connect(m_deformationCageAction, SIGNAL(triggered()), this, SLOT(openDeformationCageDialog()));

    connect(m_schnapps, SIGNAL(mapAdded(MapHandlerGen*)), this, SLOT(mapAdded(MapHandlerGen*)));
    connect(m_schnapps, SIGNAL(mapRemoved(MapHandlerGen*)), this, SLOT(mapRemoved(MapHandlerGen*)));

    foreach(MapHandlerGen* map, m_schnapps->getMapSet().values())
        mapAdded(map);

    return true;
}

void Surface_DeformationCage_Plugin::disable()
{
    delete m_colorPerVertexShader;
    delete m_positionVBO;
    delete m_colorVBO;
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
        if(h_cageParameters.contains(mhg_modified))
        {
            //Si la carte venant d'être modifiée est une cage
            CageParameters& p = h_cageParameters[mhg_modified];
            if(p.cagePosition.name()==nameAttr.toStdString())
            {
                //Si l'attribut venant d'être modifié est celui qui avait été utilisé lors de la liaison avec l'objet
                MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(mhg_modified);
                PFP2::MAP* cage = mh_cage->getMap();

                int i = 0;
                TraversorV<PFP2::MAP> trav_vert_cage(*cage);
                for(Dart d = trav_vert_cage.begin(); d!=trav_vert_cage.end(); d= trav_vert_cage.next())
                {
                    p.cagePositionEigen(i, 0) = p.cagePosition[d][0];
                    p.cagePositionEigen(i, 1) = p.cagePosition[d][1];
                    p.cagePositionEigen(i++, 2) = p.cagePosition[d][2];
                }

                p.objectPositionEigen = p.coordinatesEigen*p.cagePositionEigen;
                MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(p.controlledObject);

                i = 0;
                Dart d;
                for(int i=0; i<p.dartObjectIndicesEigen.rows(); ++i)
                {
                    d = p.dartObjectIndicesEigen(i, 0);
                    p.controlledObjectPosition[d][0] = p.objectPositionEigen(i, 0);
                    p.controlledObjectPosition[d][1] = p.objectPositionEigen(i, 1);
                    p.controlledObjectPosition[d][2] = p.objectPositionEigen(i++, 2);
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
    if(!h_cageParameters.contains(m_schnapps->getMap(cageName))) {
        //Si la carte n'est pas encore liée à un objet

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
        p.dartObjectIndicesEigen.resize(objectNbV,1);

        int i = 0;
        TraversorV<PFP2::MAP> trav_vert_cage(*cage);
        for(Dart d = trav_vert_cage.begin(); d != trav_vert_cage.end(); d = trav_vert_cage.next())
        {
            p.cagePositionEigen(i, 0) = positionCage[d][0];
            p.cagePositionEigen(i, 1) = positionCage[d][1];
            p.cagePositionEigen(i++, 2) = positionCage[d][2];
        }

        VertexAttribute<VCage> vCage= object->getAttribute<VCage, VERTEX>("VCage");

        i = 0;
        TraversorV<PFP2::MAP> trav_vert_object(*object);
        float incr = 100/objectNbV;
        float total = 0.;

        for(Dart d = trav_vert_object.begin(); d!=trav_vert_object.end(); d = trav_vert_object.next())
        {
            p.objectPositionEigen(i, 0) = positionObject[d][0];
            p.objectPositionEigen(i, 1) = positionObject[d][1];
            p.objectPositionEigen(i, 2) = positionObject[d][2];
            computePointMVCFromCage(d, positionObject, positionCage, p.coordinatesEigen, i, vCage[d].getCage(), cage);
            m_deformationCageDialog->progress_link->setValue(m_deformationCageDialog->progress_link->value()+(int)total);
            m_deformationCageDialog->update();
            total += incr;
            p.dartObjectIndicesEigen(i++, 0) = d;
        }

        CGoGNout << "Temps de calcul des coordonnées MVC : " << chrono.elapsed() << " ms." << CGoGNendl;

        CGoGNout << "Calcul de la fonction de poids .." << CGoGNflush;

        i = 0;
        for(Dart d = trav_vert_object.begin(); d!=trav_vert_object.end(); d = trav_vert_object.next())
        {
            smoothingFunction(boundaryWeightFunction(vCage[d].getCage(), cage, p.coordinatesEigen, i++));
        }

        CGoGNout << ".. fait" << CGoGNendl;
    }
}

/*
  * Fonction qui calcule les coordonnées MVC d'un point par rapport à une cage
  */
    void Surface_DeformationCage_Plugin::computePointMVCFromCage(Dart vertex, const VertexAttribute<PFP2::VEC3>& positionObject,
                                                                 const VertexAttribute<PFP2::VEC3>& positionCage, Eigen::MatrixXf& coordinates, int index,
                                                                 const std::vector<Dart>& vCage, PFP2::MAP* cage)
{
    PFP2::REAL c, sumMVC(0.);
    int i = 0;

    TraversorV<PFP2::MAP> trav_vert_cage(*cage);

    for(Dart d = trav_vert_cage.begin(); d != trav_vert_cage.end(); d = trav_vert_cage.next())
    {
        if(std::find(vCage.begin(), vCage.end(), d) != vCage.end())
        {
            c = computeMVC2D(positionObject[vertex], d, cage, positionCage);
        }
        else
        {
            c = 0.;
        }
        coordinates(index, i++) = c;
        sumMVC += c;
    }

    while(i>0)
    {
        coordinates(index, --i) /= sumMVC;
    }
}

PFP2::REAL Surface_DeformationCage_Plugin::computeMVC(const PFP2::VEC3& pt, Dart vertex, PFP2::MAP* cage, const VertexAttribute<PFP2::VEC3>& positionCage)
{
    PFP2::REAL r = (positionCage[vertex]-pt).norm();

    PFP2::REAL sumU(0.);
    Dart it = vertex;
    do
    {
        PFP2::VEC3 vi = positionCage[it];
        PFP2::VEC3 vj = positionCage[cage->phi1(it)];
        PFP2::VEC3 vk = positionCage[cage->phi_1(it)];

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

PFP2::REAL Surface_DeformationCage_Plugin::computeMVC2D(const PFP2::VEC3& pt, Dart vertex, PFP2::MAP* cage, const VertexAttribute<PFP2::VEC3>& positionCage)
{
    PFP2::REAL res;

    PFP2::VEC3 vi = positionCage[vertex];
    PFP2::VEC3 vj = positionCage[cage->phi1(vertex)];
    PFP2::VEC3 vk = positionCage[cage->phi_1(vertex)];

    PFP2::REAL Bij = Geom::angle((vi-pt), (vj-pt));
    PFP2::REAL Bki = Geom::angle((vk-pt), (vi-pt));

    res = (tan(Bki/2) + tan(Bij/2)) / ((vi-pt).norm());

    return res;
}


PFP2::REAL Surface_DeformationCage_Plugin::boundaryWeightFunction(const std::vector<Dart>& vCage, PFP2::MAP* cage,
                                                                  const Eigen::MatrixXf& coordinatesEigen, int index)
{
    PFP2::REAL res(0);

    unsigned int i=0;
    PFP2::REAL sumCur(0);
    std::vector<Dart> boundaryVertices;
    std::vector<Dart>::iterator it;
    TraversorV<PFP2::MAP> trav_vert_cage(*cage);
    DartMarker boundaryMarker(*cage);

    //On récupère les sommets bordure
    Dart d2;
    for(i=0; i<vCage.size(); ++i)
    {
        d2 = cage->phi2(vCage[i]);
        if(d2 != vCage[i])
        {
            //Si la cage est collée à une autre cage le long de cette arête
            boundaryVertices.push_back(vCage[i]);
            boundaryVertices.push_back(d2);
        }
    }

    while(boundaryVertices.size()>0)
    {
        //S'il existe des sommets appartenant à la bordure de la cage
        boundaryMarker.unmarkAll();
        sumCur = 0;

        boundaryMarker.markOrbit<VERTEX>(boundaryVertices[0]);
        boundaryMarker.markOrbit<VERTEX>(boundaryVertices[1]);

        i=0;
        for(Dart d = trav_vert_cage.begin(); d != trav_vert_cage.end(); d = trav_vert_cage.next())
        {
            if(boundaryMarker.isMarked(d))
            {
                //S'il fait partie de la bordure actuellement traitée
                if(coordinatesEigen(index, i)>FLT_EPSILON || coordinatesEigen(index, i)<-FLT_EPSILON)
                {
                    //Si la coordonnée n'est pas nulle
                    sumCur += coordinatesEigen(index, i);
                }
                if(cage->getEmbedding<VERTEX>(d)==cage->getEmbedding<VERTEX>(boundaryVertices[0]))
                {
                    boundaryVertices.erase(boundaryVertices.begin());
                }
                else
                {
                    boundaryVertices.erase(++boundaryVertices.begin());
                }
                boundaryMarker.unmarkOrbit<VERTEX>(d);
            }
            ++i;
        }
        res *= 1-sumCur;
    }

    return res;
}

PFP2::REAL Surface_DeformationCage_Plugin::smoothingFunction(const PFP2::REAL& x, const PFP2::REAL& h)
{
    if(h>FLT_EPSILON)
    {
        return 1/2*sin(M_PI*(x/h-1/2));
    }
    else
    {
        return 0.;
    }
}


#ifndef DEBUG
Q_EXPORT_PLUGIN2(Surface_DeformationCage_Plugin, Surface_DeformationCage_Plugin)
#else
Q_EXPORT_PLUGIN2(Surface_DeformationCage_PluginD, Surface_DeformationCage_Plugin)
#endif

} // namespace SCHNApps

} // namespace CGoGN

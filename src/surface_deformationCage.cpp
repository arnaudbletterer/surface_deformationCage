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

void Surface_DeformationCage_Plugin::drawMap(View *view, MapHandlerGen *map)
{
    if(m_toDraw)
    {
        //If VBO are initialized
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_LIGHTING);
        glEnable(GL_POLYGON_OFFSET_FILL);
        m_colorPerVertexShader->setAttributePosition(m_positionVBO);
        m_colorPerVertexShader->setAttributeColor(m_colorVBO);
        m_colorPerVertexShader->setOpacity(1.);
        map->draw(m_colorPerVertexShader, CGoGN::Algo::Render::GL2::TRIANGLES);
        glDisable(GL_POLYGON_OFFSET_FILL);
    }
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

                MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(p.controlledObject);

                p.objectPositionEigen = p.coordinatesEigen*p.cagePositionEigen;

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

        VertexAttribute <PFP2::VEC4> colorObject = object->getAttribute<PFP2::VEC4, VERTEX>("color");
        if(!colorObject.isValid())
        {
            colorObject = object->addAttribute<PFP2::VEC4, VERTEX>("color");
            mh_object->registerAttribute(colorObject);
        }

        PFP2::REAL res;
        for(Dart d = trav_vert_object.begin(); d!=trav_vert_object.end(); d = trav_vert_object.next())
        {
            res = smoothingFunction(boundaryWeightFunction(vCage[d].getCage(), vCage[d].getCageId(), cage, p.coordinatesEigen, i++));
            colorObject[d] = PFP2::VEC4(1-res, 0.f, res, 1.f);
        }

        m_positionVBO->updateData(positionObject);
        m_colorVBO->updateData(colorObject);
        m_toDraw = true;

        CGoGNout << ".. fait" << CGoGNendl;

        m_schnapps->getSelectedView()->updateGL();
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


PFP2::REAL Surface_DeformationCage_Plugin::boundaryWeightFunction(const std::vector<Dart>& vCage, const std::vector<int>& vCageId, PFP2::MAP* cage,
                                                                  const Eigen::MatrixXf& coordinatesEigen, int index)
{
    PFP2::REAL res(1);

    unsigned int i=0;
    PFP2::REAL sumCur(0);
    TraversorV<PFP2::MAP> trav_vert_cage(*cage);
    DartMarker cageMarker(*cage);
    DartMarker boundaryMarker(*cage);
    int currentCage;
    std::vector<int> cagesCalculated;

    VertexAttribute<VCage> vCageCage = cage->getAttribute<VCage, VERTEX>("VCage");

    //On marque les sommets de la cage
    Dart d2;
    for(i=0; i<vCage.size(); ++i)
    {
        d2 = cage->phi2(vCage[i]);
        if(cage->isBoundaryEdge(vCage[i]))
        {
            cageMarker.mark(vCage[i]);
            cageMarker.mark(d2);
        }
    }

    do
    {
        //Tant qu'il y a des sommets de la cage qui n'ont pas été traités

        currentCage = -1;
        for(Dart d  = cage->begin(); d != cage->end(); cage->next(d))
        {
            if(cageMarker.isMarked(d))
            {
                d2 = cage->phi2(d);
                if(currentCage != -1)
                {
                    for(i=0; i<vCageCage[d2].getNbId(); ++i)
                    {
                        if(vCageCage[d2].getCageId()[i] == currentCage)
                        {
                            //Si le sommet trouvé fait aussi partie de la cage courante
                            boundaryMarker.markOrbit<VERTEX>(d);
                            boundaryMarker.markOrbit<VERTEX>(d2);
                            break;
                        }
                    }
                }
                else
                {
                    if(cagesCalculated.size()==0)
                    {
                        for(i=0; i<vCageCage[d2].getNbId() && currentCage == -1; ++i)
                        {
                            if(std::find(cagesCalculated.begin(), cagesCalculated.end(), vCageCage[d2].getCageId()[i]) == cagesCalculated.end())
                            {
                                boundaryMarker.markOrbit<VERTEX>(d);
                                boundaryMarker.markOrbit<VERTEX>(d2);
                                currentCage = vCageCage[d2].getCageId()[i];
                            }
                        }
                    }
                }
            }
        }

        if(currentCage != -1)
        {
            sumCur = 0;
            i=0;
            for(Dart d = trav_vert_cage.begin(); d != trav_vert_cage.end() && !boundaryMarker.isAllUnmarked(); d = trav_vert_cage.next())
            {
                if(boundaryMarker.isMarked(d))
                {
                    //S'il fait partie de la bordure actuellement traitée
                    sumCur += coordinatesEigen(index, i);
                    boundaryMarker.unmarkOrbit<VERTEX>(d);
                    boundaryMarker.unmarkOrbit<VERTEX>(cage->phi2(d));
                }
                ++i;
            }
            res *= 1-sumCur;
            cagesCalculated.push_back(currentCage);
        }
    } while(currentCage != -1);

    return res;
}

PFP2::REAL degreeToRadian(const PFP2::REAL& deg)
{
    return deg*M_PI/180.0;
}

PFP2::REAL Surface_DeformationCage_Plugin::smoothingFunction(const PFP2::REAL& x, const PFP2::REAL& h)
{
    if(x<=h)
    {
        if(h>FLT_EPSILON)
        {
            return (1/2. * std::sin(degreeToRadian(M_PI*(x/h-1/2.))) + 1/2.);
            //return -2*(x/h)*(x/h)*(x/h) + 3*(x/h)*(x/h);
            //return -8*(x/h)*(x/h)*(x/h)*(x/h)*(x/h) + 20*(x/h)*(x/h)*(x/h)*(x/h);
        }
        else
        {
            return 0.;
        }
    }
    else
    {
        return 1.;
    }
}


#ifndef DEBUG
Q_EXPORT_PLUGIN2(Surface_DeformationCage_Plugin, Surface_DeformationCage_Plugin)
#else
Q_EXPORT_PLUGIN2(Surface_DeformationCage_PluginD, Surface_DeformationCage_Plugin)
#endif

} // namespace SCHNApps

} // namespace CGoGN

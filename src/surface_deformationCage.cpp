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
    if(orbit == VERTEX)
    {
        MapHandlerGen* mhg_modified = static_cast<MapHandlerGen*>(QObject::sender());
        MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(mhg_modified);
        PFP2::MAP* cage = mh_cage->getMap();

        if(cage->isOrbitEmbedded<FACE>())
        {
            int index_cage;
            MapHandler<PFP2>* mh_object;
            TraversorF<PFP2::MAP> trav_face_cage(*cage);
            for(Dart d = trav_face_cage.begin(); d != trav_face_cage.end(); d = trav_face_cage.next())
            {
                index_cage = cage->getEmbedding<FACE>(d);
                if(h_cageParameters.contains(index_cage))
                {
                    //Si la carte venant d'être modifiée est une cage
                    CageParameters& p = h_cageParameters[index_cage];
                    if(p.cagePosition.name() == nameAttr.toStdString())
                    {
                        //Si l'attribut venant d'être modifié est celui qui avait été utilisé lors de la liaison avec l'objet
                        int i = 0;
                        Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, p.beginningDart);
                        for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
                        {
                            p.cagePositionEigen(i, 0) = p.cagePosition[dd][0];
                            p.cagePositionEigen(i, 1) = p.cagePosition[dd][1];
                            p.cagePositionEigen(i, 2) = p.cagePosition[dd][2];
                            ++i;
                        }

                        p.objectPositionEigen = p.coordinatesEigen*p.cagePositionEigen;

                        for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
                        {
                            p.controlledObjectPosition[dd][0] = p.objectPositionEigen(i, 0);
                            p.controlledObjectPosition[dd][1] = p.objectPositionEigen(i, 1);
                            p.controlledObjectPosition[dd][2] = p.objectPositionEigen(i, 2);
                        }

                        mh_object = static_cast<MapHandler<PFP2>*>(p.controlledObject);

                        mh_object->updateBB(p.controlledObjectPosition);
                        mh_object->notifyAttributeModification(p.controlledObjectPosition);
                        mh_object->notifyConnectivityModification();
                    }
                }
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
    MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(m_schnapps->getMap(cageName));
    MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(m_schnapps->getMap(objectName));

    if(mh_cage && mh_object)
    {
        PFP2::MAP* cage = mh_cage->getMap();
        PFP2::MAP* object = mh_object->getMap();

        VertexAttribute <PFP2::VEC4> colorObject = object->getAttribute<PFP2::VEC4, VERTEX>("color");
        if(!colorObject.isValid())
        {
            colorObject = object->addAttribute<PFP2::VEC4, VERTEX>("color");
            mh_object->registerAttribute(colorObject);
        }

        VertexAttribute<PFP2::VEC3> positionObject = object->getAttribute<PFP2::VEC3, VERTEX>(objectNameAttr.toStdString());
        if(!positionObject.isValid())
        {
            CGoGNout << "Position attribute chosen for the object isn't valid" << CGoGNendl;
            return;
        }

        int index_cage;
        int i;

        TraversorV<PFP2::MAP> trav_vert_object(*object);

        object->enableQuickTraversal<VERTEX>();
        cage->enableQuickTraversal<FACE>();

        unsigned int cageNbV = 0;
        unsigned int objectNbV = 0;

        TraversorF<PFP2::MAP> trav_face_cage(*cage);
        for(Dart d = trav_face_cage.begin(); d != trav_face_cage.end(); d = trav_face_cage.next())
        {
            index_cage = cage->getEmbedding<FACE>(d);
            if((!h_cageParameters.contains(index_cage)) && index_cage < 4)
            {
                objectNbV = 0;
                cageNbV = 0;
                CageParameters& p = h_cageParameters[index_cage];

                VertexAttribute<PFP2::VEC3> positionCage = cage->getAttribute<PFP2::VEC3, VERTEX>(cageNameAttr.toStdString());
                if(!positionCage.isValid())
                {
                    CGoGNout << "Position attribute chosen for the cage isn't valid" << CGoGNendl;
                    return;
                }

                p.cagePosition = positionCage;

                p.controlledObject = m_schnapps->getMap(objectName);
                p.controlledObjectPosition = positionObject;

                i = 0;
                p.beginningDart = d;
                p.min = positionCage[p.beginningDart];
                p.max = positionCage[p.beginningDart];

                Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, d);
                for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
                {
                    if(positionCage[dd][0] < p.min[0])
                    {
                        p.min[0] = positionCage[dd][0];
                    }
                    if(positionCage[dd][1] < p.min[1])
                    {
                        p.min[1] = positionCage[dd][1];
                    }
                    if(positionCage[dd][0] > p.max[0])
                    {
                        p.max[0] = positionCage[dd][0];
                    }
                    if(positionCage[dd][1] > p.max[1])
                    {
                        p.max[1] = positionCage[dd][1];
                    }
                    ++cageNbV;
                }

                for(Dart dd = trav_vert_object.begin(); dd != trav_vert_object.end(); dd = trav_vert_object.next())
                {
                    if(isInCage(positionObject[dd], p.min, p.max))
                    {
                        ++objectNbV;
                    }
                }

                p.coordinatesEigen.resize(objectNbV, cageNbV);
                p.cagePositionEigen.resize(cageNbV,3);
                p.objectPositionEigen.resize(objectNbV,3);
                p.boundaryWeightsEigen.resize(objectNbV,1);

                for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
                {
                    p.cagePositionEigen(i, 0) = positionCage[dd][0];
                    p.cagePositionEigen(i, 1) = positionCage[dd][1];
                    p.cagePositionEigen(i, 2) = positionCage[dd][2];
                    ++i;
                }

                i = 0;

                //Calcul des coordonnées
                for(Dart dd = trav_vert_object.begin(); dd != trav_vert_object.end(); dd = trav_vert_object.next())
                {
                    if(isInCage(positionObject[dd], p.min, p.max))
                    {
                        p.objectPositionEigen(i, 0) = positionObject[dd][0];
                        p.objectPositionEigen(i, 1) = positionObject[dd][1];
                        p.objectPositionEigen(i, 2) = positionObject[dd][2];
                        computePointMVCFromCage(dd, positionObject, positionCage, p.coordinatesEigen, i, cage, p.beginningDart);
                        ++i;
                    }
                }

                i = 0;
            }
        }

        for(Dart d = trav_face_cage.begin(); d != trav_face_cage.end(); d = trav_face_cage.next())
        {
            CGoGNout << "-------" << CGoGNendl;
            index_cage = cage->getEmbedding<FACE>(d);
            if(h_cageParameters.contains(index_cage))
            {
                CageParameters& p = h_cageParameters[index_cage];

                i = 0;

                //Calcul de la fonction de poids de bordure
                for(Dart dd = trav_vert_object.begin(); dd != trav_vert_object.end(); dd = trav_vert_object.next())
                {
                    if(isInCage(positionObject[dd], p.min, p.max))
                    {
                        p.boundaryWeightsEigen(i, 0) = smoothingFunction(boundaryWeightFunction(p.coordinatesEigen, p.beginningDart, cage, i));
                        colorObject[dd] = PFP2::VEC4(p.boundaryWeightsEigen(i, 0), 0.f, 1.f - p.boundaryWeightsEigen(i, 0), 1.f);
                        ++i;
                    }
                }
            }
        }

        m_positionVBO->updateData(positionObject);
        m_colorVBO->updateData(colorObject);
        m_toDraw = true;
        m_schnapps->getSelectedView()->updateGL();
    }
}

/*
  * Fonction qui calcule les coordonnées MVC d'un point par rapport à une cage
  */
void Surface_DeformationCage_Plugin::computePointMVCFromCage(Dart vertex, const VertexAttribute<PFP2::VEC3>& positionObject,
                                                             const VertexAttribute<PFP2::VEC3>& positionCage,
                                                             Eigen::MatrixXf& coordinates, int index, PFP2::MAP* cage, Dart beginningDart)
{
    PFP2::REAL sumMVC(0.);
    int i = 0;

    Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, beginningDart);
    for(Dart d = trav_vert_face_cage.begin(); d != trav_vert_face_cage.end(); d = trav_vert_face_cage.next())
    {
        coordinates(index, i) = computeMVC2D(positionObject[vertex], d, cage, positionCage);
        sumMVC += coordinates(index, i);
        ++i;
    }

    while(i>0)
    {
        --i;
        coordinates(index, i) /= sumMVC;
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

    res = (tan(Bki/2) + tan(Bij/2)) / ((pt-vi).norm());

    return res;
}

PFP2::REAL Surface_DeformationCage_Plugin::boundaryWeightFunction(const Eigen::MatrixXf& coordinates, Dart beginningDart, PFP2::MAP* cage, int index)
{
    PFP2::REAL res(1.), sumCur(0.);

    DartMarker marker(*cage);
    Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, beginningDart);

    int currentFace = -1;
    Dart d2;
    int i = 0;

    do
    {
        currentFace = -1;
        sumCur = 0.;
        i = 0;

        //On recherche les sommets appartenant au prochain bord
        for(Dart d = trav_vert_face_cage.begin(); d != trav_vert_face_cage.end(); d = trav_vert_face_cage.next())
        {
            if(cage->vertexDegree(d)>2)
            {
                //Si le sommet fait partie de plus d'une cage
                d2 = cage->phi2(d);
                if(currentFace == -1)
                {
                    if(!marker.isMarked(d2))
                    {
                        //Si la face n'a pas encore été traitée
                        currentFace = cage->getEmbedding<FACE>(d2);
                        sumCur += coordinates(index, i);
                        marker.markOrbit<FACE>(d2);
                    }
                }
                else
                {
                    if(currentFace == cage->getEmbedding<FACE>(d2))
                    {
                        sumCur += coordinates(index, i);
                    }
                }
            }
            else
            {
                CGoGNout << "Sommet : " << d << CGoGNendl;
            }
            ++i;
        }

        res *= 1-sumCur;

    } while(currentFace != -1);

    return res;
}

PFP2::REAL degreeToRadian(const PFP2::REAL& deg)
{
    return deg*M_PI/180.0;
}

PFP2::REAL Surface_DeformationCage_Plugin::smoothingFunction(const PFP2::REAL& x, const PFP2::REAL& h)
{
        if(h>FLT_EPSILON)
        {
            //return (1/2. * std::sin(degreeToRadian(M_PI*(x/h-1/2.))) + 1/2.);
            return -2*(x/h)*(x/h)*(x/h) + 3*(x/h)*(x/h);
            //return - 8*(x/h)*(x/h)*(x/h)*(x/h)*(x/h) + 20*(x/h)*(x/h)*(x/h)*(x/h) - 18*(x/h)*(x/h)*(x/h) + 7*(x/h)*(x/h);
        }
        else
        {
            return 0.;
        }
}

bool Surface_DeformationCage_Plugin::isInCage(PFP2::VEC3 point, PFP2::VEC3 min, PFP2::VEC3 max)
{
    if(min[0] <= point[0] && min[1] <= point[1]
            && max[0] >= point[0] && max[1] >= point[1])
    {
        return true;
    }
//    else if(almostEqual2sComplement(min[0], point[0]) || almostEqual2sComplement(min[1], point[1])
//                && max[0] >= point[0] && max[1] >= point[1])
//        {
//            return true;
//        }
//    else if(almostEqual2sComplement(max[0], point[0]) || almostEqual2sComplement(max[1], point[1])
//                && min[0] <= point[0] && min[1] <= point[1])
//    {
//        return true;
//    }
    return false;
}


// http://www.cygnus-software.com/papers/comparingfloats/Comparing%20floating%20point%20numbers.htm
bool Surface_DeformationCage_Plugin::almostEqual2sComplement(PFP2::REAL A, PFP2::REAL B)
{
    int maxUlps = 10;

    int aInt = *(int*)&A;
    // Make aInt lexicographically ordered as a twos-complement int
    if (aInt < 0)
        aInt = 0x80000000 - aInt;
    // Make bInt lexicographically ordered as a twos-complement int
    int bInt = *(int*)&B;
    if (bInt < 0)
        bInt = 0x80000000 - bInt;
    int intDiff = abs(aInt - bInt);
    if (intDiff <= maxUlps)
        return true;
    return false;
}

#ifndef DEBUG
Q_EXPORT_PLUGIN2(Surface_DeformationCage_Plugin, Surface_DeformationCage_Plugin)
#else
Q_EXPORT_PLUGIN2(Surface_DeformationCage_PluginD, Surface_DeformationCage_Plugin)
#endif

} // namespace SCHNApps

} // namespace CGoGN

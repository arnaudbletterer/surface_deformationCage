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
    connect(m_deformationCageDialog->slider_boundary, SIGNAL(valueChanged(int)), this, SLOT(boundarySliderValueChanged(int)));

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
    if(m_toDraw && m_schnapps->getSelectedView() == view)
    {
        //If VBO are initialized
        glPolygonMode(GL_FRONT, GL_FILL);
        glEnable(GL_LIGHTING);
        glEnable(GL_POLYGON_OFFSET_FILL);
        m_colorPerVertexShader->setAttributePosition(m_positionVBO);
        m_colorPerVertexShader->setAttributeColor(m_colorVBO);
        m_colorPerVertexShader->setOpacity(1.);
        map->draw(m_colorPerVertexShader, CGoGN::Algo::Render::GL2::TRIANGLES);
        glDisable(GL_POLYGON_OFFSET_FILL);
    }
}

void Surface_DeformationCage_Plugin::boundarySliderValueChanged(int value)
{
    MapHandlerGen* mhg_object = m_schnapps->getMap("Model");
    MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(mhg_object);
    PFP2::MAP* object = mh_object->getMap();
    MapHandlerGen* mhg_cage = m_schnapps->getMap("Cages");
    MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(mhg_cage);
    PFP2::MAP* cage = mh_cage->getMap();

    PFP2::REAL h = value/100.f;

    computeBoundaryWeights(cage, object, h, false);

    m_deformationCageDialog->label_boundary->setText(QString::number(h, 'g', 2));
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
        MapHandler<PFP2>* mh_object = NULL;
        PFP2::MAP* object = NULL;
        VertexAttribute<PFP2::VEC3> positionObject;

        if(cage->isOrbitEmbedded<FACE>())
        {
            int index_cage;

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
                        mh_object = static_cast<MapHandler<PFP2>*>(p.controlledObject);
                        if(mh_object)
                        {
                            int i = 0;
                            Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, p.beginningDart);
                            for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
                            {
                                p.cagePositionEigen(i, 0) = p.cagePosition[dd][0];
                                p.cagePositionEigen(i, 1) = p.cagePosition[dd][1];
                                ++i;
                            }

                            object = mh_object->getMap();

                            p.objectPositionEigen = p.coordinatesCageEigen*p.cagePositionEigen;

                            VertexAttribute<Dart> indexCageObject = object->getAttribute<Dart, VERTEX>("indexCage");
                            if(!indexCageObject.isValid())
                            {
                                CGoGNout << "indexCage attribute chosen for the object isn't valid" << CGoGNendl;
                                return;
                            }

                            i = 0;
                            TraversorV<PFP2::MAP> trav_vert_vert_object(*object);
                            for(Dart dd = trav_vert_vert_object.begin(); dd != trav_vert_vert_object.end(); dd = trav_vert_vert_object.next())
                            {
                                if(indexCageObject[dd] == p.beginningDart)
                                {
                                    p.controlledObjectPosition[dd][0] = p.objectPositionEigen(i,0);
                                    p.controlledObjectPosition[dd][1] = p.objectPositionEigen(i,1);
                                    ++i;
                                }
                            }

                            positionObject = p.controlledObjectPosition;
                        }
                    }
                }
            }

            if(mh_object)
            {
                m_positionVBO->updateData(positionObject);
                mh_object->updateBB(positionObject);
                mh_object->notifyAttributeModification(positionObject);
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

        VertexAttribute<Dart> indexCageObject = object->getAttribute<Dart, VERTEX>("indexCage");
        if(!indexCageObject.isValid())
        {
            indexCageObject = object->addAttribute<Dart, VERTEX>("indexCage");
            mh_object->registerAttribute(indexCageObject);
        }

        int index_cage;
        int i;

        TraversorV<PFP2::MAP> trav_vert_object(*object);

        unsigned int cageNbV = 0;
        unsigned int objectNbV = 0;

        TraversorF<PFP2::MAP> trav_face_cage(*cage);
        for(Dart d = trav_face_cage.begin(); d != trav_face_cage.end(); d = trav_face_cage.next())
        {
            index_cage = cage->getEmbedding<FACE>(d);
            if((!h_cageParameters.contains(index_cage)) && !cage->isBoundaryMarked2(d))
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

                p.beginningDart = d;
                p.min = positionCage[p.beginningDart];
                p.max = p.min;

                p.joinCage = findJoinCage(cage, p.beginningDart);

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
                    if(indexCageObject[dd]==EMBNULL && isInCage(positionObject[dd], p.min, p.max))
                    {
                        ++objectNbV;
                        indexCageObject[dd] = p.beginningDart;
                    }
                }

                p.coordinatesCageEigen.setZero(objectNbV, cageNbV);

                p.cagePositionEigen.resize(cageNbV,2);
                p.objectPositionEigen.resize(objectNbV, 2);

                p.boundaryWeightsEigen.setZero(objectNbV, cageNbV);
                p.smoothBoundaryWeightsEigen.setZero(objectNbV, cageNbV);

                i = 0;

                for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
                {
                    p.cagePositionEigen(i, 0) = positionCage[dd][0];
                    p.cagePositionEigen(i, 1) = positionCage[dd][1];
                    ++i;
                }

                i = 0;

                for(Dart dd = trav_vert_object.begin(); dd != trav_vert_object.end(); dd = trav_vert_object.next())
                {
                    if(p.beginningDart == indexCageObject[dd])
                    {
                        p.objectPositionEigen(i, 0) = positionObject[dd][0];
                        p.objectPositionEigen(i, 1) = positionObject[dd][1];
                        computePointMVCFromCage(dd, positionObject, positionCage, p.coordinatesCageEigen, i, cage, p.beginningDart, cageNbV);
                        ++i;
                    }
                }
            }
        }

        computeBoundaryWeights(cage, object);
        computeExternalCoordinates(cage, object);

        m_positionVBO->updateData(positionObject);
        m_toDraw = true;
    }
}

void Surface_DeformationCage_Plugin::computeBoundaryWeights(PFP2::MAP* cage, PFP2::MAP* object, PFP2::REAL h, bool first)
{
    int i = 0, j = 0, index_cage = -1;

    TraversorV<PFP2::MAP> trav_vert_object(*object);
    VertexAttribute<PFP2::VEC4> colorObject = object->getAttribute<PFP2::VEC4, VERTEX>("color");
    VertexAttribute<Dart> indexCageObject = object->getAttribute<Dart, VERTEX>("indexCage");

    PFP2::VEC3 color;

    TraversorF<PFP2::MAP> trav_face_cage(*cage);
    for(Dart d = trav_face_cage.begin(); d != trav_face_cage.end(); d = trav_face_cage.next())
    {
        index_cage = cage->getEmbedding<FACE>(d);
        if(h_cageParameters.contains(index_cage))
        {
            CageParameters& p = h_cageParameters[index_cage];

            i = 0;

            //Calcul de la fonction de poids de bordure
            for(Dart dd = trav_vert_object.begin(); dd != trav_vert_object.end(); dd = trav_vert_object.next())
            {
                if(indexCageObject[dd] == p.beginningDart)
                {
                    if(first)
                    {
                        boundaryWeightFunction(p.coordinatesCageEigen, p.beginningDart, p.boundaryWeightsEigen, cage, i);
                    }

                    j = 0;
                    PFP2::REAL value(1.);
                    Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, p.beginningDart);
                    for(Dart ddd = trav_vert_face_cage.begin(); ddd != trav_vert_face_cage.end(); ddd = trav_vert_face_cage.next())
                    {
                        p.smoothBoundaryWeightsEigen(i, j) = smoothingFunction(p.boundaryWeightsEigen(i, j), h);
                        //Uniquement pour avoir de la coloration
                        if(p.boundaryWeightsEigen(i, j) > 0)
                        {
                            value *= p.boundaryWeightsEigen(i, j);
                        }
                        ++j;
                    }

                    color = Utils::color_map_BCGYR(smoothingFunction(value, h));
                    colorObject[dd] = PFP2::VEC4(color[0], color[1], color[2], 1.f);
                    ++i;
                }
            }
        }
    }

    m_colorVBO->updateData(colorObject);
    m_schnapps->getSelectedView()->updateGL();
}

void Surface_DeformationCage_Plugin::computeExternalCoordinates(PFP2::MAP* cage, PFP2::MAP* object)
{
    int index_cage, i = 0, j = 0;

    VertexAttribute<PFP2::VEC3> positionCage = cage->getAttribute<PFP2::VEC3, VERTEX>("position");
    if(!positionCage.isValid())
    {
        CGoGNout << "Position attribute chosen for the cage isn't valid" << CGoGNendl;
        return;
    }

    VertexAttribute<PFP2::VEC3> positionObject = cage->getAttribute<PFP2::VEC3, VERTEX>("position");
    if(!positionObject.isValid())
    {
        CGoGNout << "Position attribute chosen for the cage isn't valid" << CGoGNendl;
        return;
    }

    TraversorF<PFP2::MAP> trav_face_cage(*cage);
    TraversorV<PFP2::MAP> trav_vert_object(*object);
    for(Dart d = trav_face_cage.begin(); d != trav_face_cage.end(); d = trav_face_cage.next())
    {
        index_cage = cage->getEmbedding<FACE>(d);
        if(h_cageParameters.contains(index_cage))
        {
            CageParameters& p = h_cageParameters[index_cage];

            j = 0;
            for(Dart dd = trav_vert_object.begin(); dd != trav_vert_object.end(); dd = trav_vert_object.next())
            {
                ++j;
            }

            ++i;
        }
    }
}

/*
  * Fonction qui calcule les coordonnées MVC d'un point par rapport à une cage
  */
void Surface_DeformationCage_Plugin::computePointMVCFromCage(Dart vertex, const VertexAttribute<PFP2::VEC3>& positionObject,
                                                             const VertexAttribute<PFP2::VEC3>& positionCage,
                                                             Eigen::MatrixXf& coordinates, int index, PFP2::MAP* cage, Dart beginningDart,
                                                             int cageNbV)
{
    PFP2::REAL sumMVC(0.);
    unsigned int i = 0;

    bool stop = false;
    Dart next, prev;

    PFP2::REAL distance_next(0.f), distance_prev(0.f);

    Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, beginningDart);
    for(Dart d = trav_vert_face_cage.begin(); d != trav_vert_face_cage.end() && !stop; d = trav_vert_face_cage.next())
    {
        next = cage->phi1(d);
        prev = cage->phi_1(d);

        distance_next = Geom::squaredDistanceSeg2Point(positionCage[next], positionCage[d]-positionCage[next],
                                                       (positionCage[d]-positionCage[next]).norm2(),
                                                       positionObject[vertex]);

        distance_prev = Geom::squaredDistanceSeg2Point(positionCage[prev], positionCage[d]-positionCage[prev],
                                                       (positionCage[d]-positionCage[prev]).norm2(),
                                                       positionObject[vertex]);

        if(distance_next < FLT_EPSILON && distance_prev < FLT_EPSILON)
        {
            //Le sommet de l'objet se situe sur le sommet courant de la cage
            coordinates.row(index).setZero();

            coordinates(index, i) = 1.f;    //Le sommet de l'objet est entièrement dépendant du sommet courant de la cage

            stop = true;
        }
        else if(distance_next < FLT_EPSILON)
        {
            //Le sommet de l'objet est sur [cur;next]
            coordinates.row(index).setZero();

            PFP2::REAL w = sqrt((positionObject[vertex]-positionCage[d]).norm2()
                                / (positionCage[next]-positionCage[d]).norm2());

            coordinates(index, (i+1)%cageNbV) = w;
            coordinates(index, i) = 1.f-w;

            stop = true;
        }
        else if(distance_prev < FLT_EPSILON)
        {
            //Le sommet de l'objet est sur [cur;prev]
            coordinates.row(index).setZero();

            PFP2::REAL w = sqrt((positionObject[vertex]-positionCage[d]).norm2()
                                / (positionCage[prev]-positionCage[d]).norm2());

            if(i==0)
            {
                coordinates(index, cageNbV-1) = w;
            }
            else
            {
                coordinates(index, i-1) = w;
            }
            coordinates(index, i) = 1.f-w;

            stop = true;
        }
        else
        {
            //On calcule les coordonnées de façon normale
            coordinates(index, i) = computeMVC2D(positionObject[vertex], d, next, prev, positionCage);
            sumMVC += coordinates(index, i);
        }
        ++i;
    }

    if(!stop)
    {
        for(i=0; i<coordinates.cols(); ++i)
        {
            coordinates(index, i) /= sumMVC;
        }
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

PFP2::REAL Surface_DeformationCage_Plugin::computeMVC2D(const PFP2::VEC3& pt, Dart current, Dart next, Dart previous,
                                                        const VertexAttribute<PFP2::VEC3>& positionCage)
{
    PFP2::REAL res;

    PFP2::VEC3 vi = positionCage[current];
    PFP2::VEC3 vj = positionCage[next];
    PFP2::VEC3 vk = positionCage[previous];

    bool positiveAngle_prev = Geom::testOrientation2D(pt, vk, vi) == Geom::LEFT;
    bool positiveAngle_next = Geom::testOrientation2D(pt, vi, vj) == Geom::LEFT;

    PFP2::REAL Bij = Geom::angle((vi-pt), (vj-pt));
    PFP2::REAL Bki = Geom::angle((vk-pt), (vi-pt));

    if(!positiveAngle_prev)
    {
        Bij *= -1;
    }
    if(!positiveAngle_next)
    {
        Bki *= -1;
    }

    res = (tan(Bki/2.f) + (tan(Bij/2.f))) /((pt-vi).norm());

    return res;
}

void Surface_DeformationCage_Plugin::boundaryWeightFunction(const Eigen::MatrixXf& coordinates, Dart beginningDart,
                                                            Eigen::MatrixXf& boundaryWeights, PFP2::MAP* cage, int index)
{
    PFP2::REAL sumCur(0.);

    Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, beginningDart);

    int i = 0, j = 0;

    DartMarker marker(*cage);

    Dart d2;
    int researchedVertex = -1;

    bool stop = false, restart = false;

    do
    {
        sumCur = 0.;
        stop = false;
        restart = false;
        j = 0;
        researchedVertex = -1;

        for(Dart d = trav_vert_face_cage.begin(); d != trav_vert_face_cage.end() && !stop;)
        {
            if(cage->vertexDegree(d) > 2)
            {
                //Si le sommet est incident à plus d'une face
                d2 = cage->phi2(d);
                if(researchedVertex == -1)
                {
                    //On cherche une autre bordure commune
                    if(!cage->isBoundaryMarked2(d2))
                    {
                        if(!marker.isMarked(d2))
                        {
                            //Si la bordure courante n'a pas encore été considérée
                            researchedVertex = cage->getEmbedding<VERTEX>(d2);
                            marker.markOrbit<FACE>(d2);
                            sumCur += coordinates(index, j);
                            restart = true;
                        }
                    }
                }
                else
                {
                    if(cage->getEmbedding<VERTEX>(d) == researchedVertex)
                    {
                        //On a trouvé le deuxième sommet composant l'arête
                        sumCur += coordinates(index, j);
                        stop = true;
                    }
                }
            }

            if(restart)
            {
                d = trav_vert_face_cage.begin();
                restart = false;
                j = 0;
            }
            else
            {
                d = trav_vert_face_cage.next();
                ++j;
            }
        }

        if(researchedVertex != -1)
        {
            boundaryWeights(index, i) = 1 - sumCur;
        }
        ++i;
    } while(researchedVertex != -1);
}

PFP2::REAL Surface_DeformationCage_Plugin::smoothingFunction(const PFP2::REAL& x, const PFP2::REAL& h)
{
    if(x >= h)
    {
        return 1.f;
    }

    if(h > FLT_EPSILON*10000)
    {
        return (1/2. * std::sin(M_PI*(x/h-1/2.)) + 1/2.);
        //return -2*(x/h)*(x/h)*(x/h) + 3*(x/h)*(x/h);
        //return -8*(x/h)*(x/h)*(x/h)*(x/h)*(x/h) + 20*(x/h)*(x/h)*(x/h)*(x/h) - 18*(x/h)*(x/h)*(x/h) + 7*(x/h)*(x/h);
    }

    return 0.;
}

std::vector<Dart> Surface_DeformationCage_Plugin::findJoinCage(PFP2::MAP* cage, Dart beginningDart)
{
    DartMarker markerJoinCage(*cage);
    Dart startingDart, currentDart;

    std::vector<Dart> joinCage;

    markerJoinCage.markOrbit<FACE>(beginningDart);

//    Traversor2FFaV<PFP2::MAP> trav_ffav_cage(*cage, beginningDart);
//    for(Dart d = trav_ffav_cage.begin(); d != trav_ffav_cage.end(); d = trav_ffav_cage.next())
//    {
//        if(!cage->isBoundaryMarked2(d))
//        {
//            CGoGNout << "face marquée" << CGoGNendl;
//            markerJoinCage.markOrbit<FACE>(d);
//        }
//    }

    startingDart = beginningDart;
    do
    {
        currentDart = startingDart;
        do
        {
            if(!cage->isBoundaryMarked2(currentDart))
            {
                markerJoinCage.markOrbit<FACE>(currentDart);
            }
            currentDart = cage->phi<21>(currentDart);
        } while(currentDart != startingDart);

        startingDart = cage->phi1(startingDart);
    } while(startingDart != beginningDart);

    if(!markerJoinCage.isMarked(cage->phi2(beginningDart)))
    {
        //On se trouve sur le bord
        startingDart = beginningDart;
    }
    else
    {
        //On cherche un brin sur le bord
        startingDart = cage->phi2(beginningDart);
        do
        {
            startingDart = cage->phi1(startingDart);
        } while(markerJoinCage.isMarked(cage->phi2(startingDart)));
    }

    currentDart = cage->phi2(startingDart);

    do
    {
        if(!markerJoinCage.isMarked(currentDart))
        {
            currentDart = cage->phi2(currentDart);
            joinCage.push_back(currentDart);
        }
        currentDart = cage->phi<12>(currentDart);
    } while(currentDart != cage->phi2(startingDart));

    return joinCage;
}

bool Surface_DeformationCage_Plugin::isInCage(PFP2::VEC3 point, PFP2::VEC3 min, PFP2::VEC3 max)
{
    if(point[0]+(FLT_EPSILON*10000) > min[0] && point[1]+(FLT_EPSILON*10000) > min[1]
            && point[0]-(FLT_EPSILON*10000) < max[0] && point[1]-(FLT_EPSILON*10000) < max[1])
    {
        return true;
    }

    return false;
}

#ifndef DEBUG
Q_EXPORT_PLUGIN2(Surface_DeformationCage_Plugin, Surface_DeformationCage_Plugin)
#else
Q_EXPORT_PLUGIN2(Surface_DeformationCage_PluginD, Surface_DeformationCage_Plugin)
#endif

} // namespace SCHNApps

} // namespace CGoGN

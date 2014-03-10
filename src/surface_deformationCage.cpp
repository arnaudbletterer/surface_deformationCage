#include "surface_deformationCage.h"

namespace CGoGN
{

namespace SCHNApps
{

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
//    MapHandlerGen* mhg_object = m_schnapps->getMap("Model");
//    MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(mhg_object);
//    PFP2::MAP* object = mh_object->getMap();
//    MapHandlerGen* mhg_cage = m_schnapps->getMap("Cages");
//    MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(mhg_cage);
//    PFP2::MAP* cage = mh_cage->getMap();

//    PFP2::REAL h = value/100.f;

//    computeBoundaryWeights(cage, object, h, false);

//    m_deformationCageDialog->label_boundary->setText(QString::number(h, 'g', 2));
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
    MapHandlerGen* mhg_modified = static_cast<MapHandlerGen*>(QObject::sender());
    MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(mhg_modified);
    if(orbit == VERTEX && nameAttr=="position" && mh_cage->getName() == "Cages")
    {
        PFP2::MAP* cage = mh_cage->getMap();

        VertexAttribute<PFP2::VEC3> positionCage = cage->getAttribute<PFP2::VEC3, VERTEX>("position");

        MapHandlerGen* mhg_object = m_schnapps->getMap("Model");
        MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(mhg_object);
        PFP2::MAP* object = mh_object->getMap();

        VertexAttribute<PFP2::VEC3> positionObject = object->getAttribute<PFP2::VEC3, VERTEX>("position");
        VertexAttribute<SpacePoint> spacePointObject = object->getAttribute<SpacePoint, VERTEX>("SpacePoint");

        if(spacePointObject.isValid())
        {
            //Si les calculs de poids ont déjà été effectués
            TraversorV<PFP2::MAP> trav_vert_object(*object);
            for(Dart d = trav_vert_object.begin(); d != trav_vert_object.end(); d = trav_vert_object.next())
            {
                Eigen::Matrix<float, Eigen::Dynamic, 2> cageCoordinatesEigen;
                cageCoordinatesEigen.setZero(cage->faceDegree(spacePointObject[d].getCageDart()), 2);

                //On récupère les positions des sommets de la cage locale
                unsigned int i = 0;
                Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, spacePointObject[d].getCageDart());
                for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
                {
                    cageCoordinatesEigen(i, 0) = positionCage[dd][0];
                    cageCoordinatesEigen(i, 1) = positionCage[dd][1];
                    ++i;
                }

                Eigen::Matrix<float, 1, 2> objectPositionEigen;
                objectPositionEigen.setZero(1, 2);
                float totalBoundaries(0.);

                //On récupère les positions des sommets des cages adjacentes
                for(i = 0; i < spacePointObject[d].m_adjCagesDart.size(); ++i)
                {
                    Eigen::Matrix<float, Eigen::Dynamic, 2> adjCageCoordinatesEigen;
                    Eigen::Matrix<float, 1, Eigen::Dynamic> adjCageWeightsEigen;
                    adjCageCoordinatesEigen.setZero(spacePointObject[d].m_adjCagesWeights[i].cols(), 2);
                    adjCageWeightsEigen.setZero(1, spacePointObject[d].m_adjCagesWeights[i].cols());

                    int j = 0;

                    Traversor2FV<PFP2::MAP> trav_adj(*cage, spacePointObject[d].m_adjCagesDart[i]);
                    for(Dart dd = trav_adj.begin(); dd != trav_adj.end(); dd = trav_adj.next())
                    {
                        adjCageWeightsEigen(0, j) = spacePointObject[d].m_adjCagesWeights[i](0, j);
                        adjCageCoordinatesEigen(j, 0) = positionCage[dd][0];
                        adjCageCoordinatesEigen(j, 1) = positionCage[dd][1];
                        ++j;
                    }
                    objectPositionEigen += spacePointObject[d].m_cageBoundaryWeights[i] * (adjCageWeightsEigen * adjCageCoordinatesEigen);
                    totalBoundaries += spacePointObject[d].m_cageBoundaryWeights[i];
                }

                objectPositionEigen += (1 - totalBoundaries) * (spacePointObject[d].m_cageWeightsEigen * cageCoordinatesEigen);

                positionObject[d][0] = objectPositionEigen(0, 0);
                positionObject[d][1] = objectPositionEigen(0, 1);
            }

            if(mh_object)
            {
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

        VertexAttribute<PFP2::VEC3> positionCage = cage->getAttribute<PFP2::VEC3, VERTEX>(cageNameAttr.toStdString());
        if(!positionCage.isValid())
        {
            CGoGNout << "Position attribute chosen for the cage isn't valid" << CGoGNendl;
            return;
        }

        VertexAttribute<SpacePoint> spacePointObject = object->getAttribute<SpacePoint, VERTEX>("SpacePoint");
        if(!spacePointObject.isValid())
        {
            spacePointObject = object->addAttribute<SpacePoint, VERTEX>("SpacePoint");
            mh_object->registerAttribute(spacePointObject);
        }

//        CGoGNStream::Out fichier;
//        fichier.toFile("/home/bletterer/plot3d_coordinates_inside.gp");

        TraversorV<PFP2::MAP> trav_vert_object(*object);
        for(Dart d = trav_vert_object.begin(); d != trav_vert_object.end(); d = trav_vert_object.next())
        {
            int i = 0;
            TraversorF<PFP2::MAP> trav_face_cage(*cage);
            for(Dart dd = trav_face_cage.begin(); dd != trav_face_cage.end(); dd = trav_face_cage.next())
            {
                if(!cage->isBoundaryMarked2(dd))
                {
                    spacePointObject[d].setCage(dd);
                    spacePointObject[d].setCageNbV(cage->faceDegree(dd));
                    computePointMVCFromCage(d, positionObject, positionCage, spacePointObject[d].m_cageWeightsEigen, cage, dd, cage->faceDegree(dd));
//                    if(i==0)
//                    {
//                        fichier << positionObject[d][0] << " " << positionObject[d][1] << " " << spacePointObject[d].m_cageWeightsEigen(0, i) << CGoGNendl;
//                    }
                }
                ++i;
            }
        }

//        fichier.close();

//        fichier.toFile("/home/bletterer/plot3d_cage_inside.gp");

//        TraversorF<PFP2::MAP> trav_face_cage(*cage);
//        for(Dart d = trav_face_cage.begin(); d != trav_face_cage.end(); d = trav_face_cage.next())
//        {
//            if(!cage->isBoundaryMarked2(d))
//            {
//                int i = 0;
//                Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, d);
//                for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
//                {
//                    if(i==0)
//                        fichier << positionCage[dd][0] << " " << positionCage[dd][1] << " " << 1 << CGoGNendl;
//                    else
//                        fichier << positionCage[dd][0] << " " << positionCage[dd][1] << " " << 0 << CGoGNendl;
//                    ++i;
//                }
//                fichier << positionCage[trav_vert_face_cage.begin()][0] << " " << positionCage[trav_vert_face_cage.begin()][1] << " " << 1 << CGoGNendl;
//            }
//        }

//        fichier.close();

        mh_cage->notifyAttributeModification(positionCage);     //JUSTE POUR DEBUG SANS DEPLACER DE SOMMETS DE CAGE
        computeBoundaryWeights(cage, object);
    }
}

void Surface_DeformationCage_Plugin::computeBoundaryWeights(PFP2::MAP* cage, PFP2::MAP* object)
{
    TraversorV<PFP2::MAP> trav_vert_object(*object);
    VertexAttribute<SpacePoint> spacePointObject = object->getAttribute<SpacePoint, VERTEX>("SpacePoint");
    VertexAttribute<PFP2::VEC4> colorObject = object->getAttribute<PFP2::VEC4, VERTEX>("color");
    if(!colorObject.isValid())
    {
        colorObject = object->addAttribute<PFP2::VEC4, VERTEX>("color");
    }

    PFP2::VEC3 color;

    for(Dart d = trav_vert_object.begin(); d != trav_vert_object.end(); d = trav_vert_object.next())
    {
        boundaryWeightFunction(spacePointObject[d].m_cageWeightsEigen, spacePointObject[d].getCageDart(),
                               spacePointObject[d].m_cageBoundaryWeights, cage);
    }

    m_colorVBO->updateData(colorObject);

    m_toDraw = true;

    m_schnapps->getSelectedView()->updateGL();
}

/*
  * Fonction qui calcule les coordonnées MVC d'un point par rapport à une cage
  */
void Surface_DeformationCage_Plugin:: computePointMVCFromCage(Dart vertex, const VertexAttribute<PFP2::VEC3>& positionObject,
                                                             const VertexAttribute<PFP2::VEC3>& positionCage,
                                                             Eigen::Matrix<float, 1, Eigen::Dynamic>& weights,
                                                             PFP2::MAP* cage, Dart beginningDart, int cageNbV)
{
    PFP2::REAL sumMVC(0.);
    int i = 0;

    bool stop = false;
    Dart next, prev;

    PFP2::REAL distance_next(0.f), distance_prev(0.f);

    weights.setZero(1, cageNbV);

    Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, beginningDart);
    for(Dart d = trav_vert_face_cage.begin(); d != trav_vert_face_cage.end() && !stop; d = trav_vert_face_cage.next())
    {
        next = cage->phi1(d);
        prev = cage->phi_1(d);

//        distance_next = Geom::squaredDistanceSeg2Point(positionCage[next], positionCage[d]-positionCage[next],
//                                                       (positionCage[d]-positionCage[next]).norm2(),
//                                                       positionObject[vertex]);

//        distance_prev = Geom::squaredDistanceSeg2Point(positionCage[prev], positionCage[d]-positionCage[prev],
//                                                       (positionCage[d]-positionCage[prev]).norm2(),
//                                                       positionObject[vertex]);

//        if(distance_next < FLT_EPSILON && distance_prev < FLT_EPSILON)
//        {
//            //Le sommet de l'objet se situe sur le sommet courant de la cage
//            weights.setZero(1, cageNbV);

//            weights(0, i) = 1.f;    //Le sommet de l'objet est entièrement dépendant du sommet courant de la cage

//            stop = true;
//        }
//        else if(distance_next < FLT_EPSILON)
//        {
//            //Le sommet de l'objet est sur [cur;next]
//            weights.setZero(1, cageNbV);

//            PFP2::REAL w = sqrt((positionObject[vertex]-positionCage[d]).norm2()
//                                / (positionCage[next]-positionCage[d]).norm2());

//            weights(0, (i+1)%cageNbV) = w;
//            weights(0, i) = 1.f-w;

//            stop = true;
//        }
//        else if(distance_prev < FLT_EPSILON)
//        {
//            //Le sommet de l'objet est sur [cur;prev]
//            weights.setZero(1, cageNbV);

//            PFP2::REAL w = sqrt((positionObject[vertex]-positionCage[d]).norm2()
//                                / (positionCage[prev]-positionCage[d]).norm2());

//            if(i==0)
//            {
//                weights(0, cageNbV-1) = w;
//            }
//            else
//            {
//                weights(0, i-1) = w;
//            }
//            weights(0, i) = 1.f-w;

//            stop = true;
//        }
//        else
//        {
//            //On calcule les coordonnées de façon normale
//            weights(0, i) = computeMVC2D(positionObject[vertex], d, next, prev, positionCage);
//            sumMVC += weights(0, i);
//        }
        weights(0, i) = computeMVC2D(positionObject[vertex], d, next, prev, positionCage, cage);
        sumMVC += weights(0, i);
        ++i;
    }

//    if(!stop)
//    {
//        for(i=0; i<weights.cols(); ++i)
//        {
//            weights(0, i) /= sumMVC;
//        }
//    }

    for(i=0; i<weights.cols(); ++i)
    {
        weights(0, i) /= sumMVC;
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
                                                        const VertexAttribute<PFP2::VEC3>& positionCage, PFP2::MAP* cage)
{
    PFP2::REAL res(1.);

    const PFP2::VEC3 c = positionCage[current];
    const PFP2::VEC3 c_prev = positionCage[previous];
    const PFP2::VEC3 c_next = positionCage[next];

    bool positiveAngle_prev = Geom::testOrientation2D(pt, c_prev, c) == Geom::LEFT;
    bool positiveAngle_next = Geom::testOrientation2D(pt, c, c_next) == Geom::LEFT;

//    PFP2::VEC3 v = c - pt ;
//    PFP2::VEC3 v_prev = c_prev - pt ;
//    PFP2::VEC3 v_next = c_next - pt ;

//    PFP2::REAL B_prev = Geom::angle(v_prev, v);
//    PFP2::REAL B_next = Geom::angle(v, v_next);

//    if(!positiveAngle_prev)
//    {
//        B_prev *= -1;
//    }

//    if(!positiveAngle_next)
//    {
//        B_next *= -1;
//    }

//    res = (tan(B_prev/2.f) + (tan(B_next/2.f))) / v.norm();

    const PFP2::VEC3 di_prev = c_prev - pt;
    const PFP2::VEC3 di_next = c_next - pt;

    const PFP2::REAL ri_prev = di_prev.norm();
    const PFP2::REAL ri_next = di_next.norm();

    //res = sqrt( (2 * (ri_prev*ri_next - di_prev*di_next)) / ((ri_prev*ri + di_prev*di) * (ri*ri_next + di*di_next)) );

    if(!positiveAngle_prev || !positiveAngle_next)
    {
        res = - sqrt(ri_prev*ri_next - di_prev*di_next);
    }
    else
    {
        res = sqrt((ri_prev*ri_next - di_prev*di_next));
    }

    Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, current);

    for(Dart d = trav_vert_face_cage.begin(); d != trav_vert_face_cage.end(); d = trav_vert_face_cage.next())
    {
        if(d!=current && d!=previous)
        {
            PFP2::VEC3 dj = positionCage[d] - pt;
            PFP2::VEC3 dj_next = positionCage[cage->phi1(d)] - pt;

            PFP2::REAL rj = dj.norm();
            PFP2::REAL rj_next = dj_next.norm();
            res *= sqrt(rj*rj_next + dj*dj_next);
        }
    }

    return res;
}

void Surface_DeformationCage_Plugin::boundaryWeightFunction(const Eigen::Matrix<float, 1, Eigen::Dynamic>& weights, Dart beginningDart,
                                                            std::vector<PFP2::REAL>& boundaryWeights, PFP2::MAP* cage)
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
                //Si le sommet est incident à plus de 3 arêtes (plus d'une face)
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
                            sumCur += weights(0, j);
                            restart = true;
                        }
                    }
                }
                else
                {
                    if(cage->getEmbedding<VERTEX>(d) == researchedVertex)
                    {
                        //On a trouvé le deuxième sommet composant l'arête de la bordure courante
                        sumCur += weights(0, j);
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
            boundaryWeights[i] = smoothingFunction(1 - sumCur);
        }
        ++i;
    } while(researchedVertex != -1);
}

PFP2::REAL Surface_DeformationCage_Plugin::smoothingFunction(const PFP2::REAL x, const PFP2::REAL h)
{
    if(x >= h)
    {
        return 0.f;
    }

    if(h > FLT_EPSILON*10000)
    {
        return (1/4. * std::cos(M_PI*(x/h)) + 1/4.);
        //return (1/4. * std::sin(M_PI*(x/h-1/2.)) + 1/4.);
        //return -2*(x/h)*(x/h)*(x/h) + 3*(x/h)*(x/h);
        //return -8*(x/h)*(x/h)*(x/h)*(x/h)*(x/h) + 20*(x/h)*(x/h)*(x/h)*(x/h) - 18*(x/h)*(x/h)*(x/h) + 7*(x/h)*(x/h);
    }

    return 0.;
}

bool Surface_DeformationCage_Plugin::isInCage(const PFP2::VEC3& point, const PFP2::VEC3& min, const PFP2::VEC3& max)
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

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

    m_movingVertices = false;
    m_movingVerticesInitiated = false;

    connect(m_deformationCageAction, SIGNAL(triggered()), this, SLOT(openDeformationCageDialog()));
    connect(m_deformationCageDialog->slider_boundary, SIGNAL(valueChanged(int)), this, SLOT(boundarySliderValueChanged(int)));

    m_deformationCageDialog->slider_boundary->setMinimum(1);
    m_deformationCageDialog->slider_boundary->setMaximum(100);

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
    if(m_toDraw && m_schnapps->getSelectedView() == view && map->getName()=="Model")
    {
        //If VBO are initialized
        glPolygonMode(GL_FRONT, GL_FILL);
        glEnable(GL_LIGHTING);
        glEnable(GL_POLYGON_OFFSET_FILL);
        m_colorPerVertexShader->setAttributePosition(m_positionVBO);
        m_colorPerVertexShader->setAttributeColor(m_colorVBO);
        m_colorPerVertexShader->setOpacity(1.0);
        map->draw(m_colorPerVertexShader, CGoGN::Algo::Render::GL2::TRIANGLES);
        glDisable(GL_POLYGON_OFFSET_FILL);
    }
}

void Surface_DeformationCage_Plugin::keyPress(View* view, QKeyEvent* event)
{
    if(m_schnapps->getSelectedView() == view)
    {
        if(QApplication::keyboardModifiers() == Qt::ControlModifier)
        {
            switch(event->key())
            {
            case Qt::Key_R :
                resetWeightsCalculated();
                CGoGNout << "Coordonnées réinitialisées" << CGoGNendl;
                break;
            default:
                break;
            }
        }
        else
        {
            switch(event->key())
            {
            case Qt::Key_D :
                if(!m_movingVertices)
                {
                    CGoGNout << "--- DEBUT DEPLACEMENT DE SOMMETS ---" << CGoGNendl;
                    m_movingVertices = true;
                    m_movingVerticesInitiated = false;
                    view->setMouseTracking(true);
                }
                else
                {
                    m_movingVertices = false;
                    m_movingVerticesInitiated = false;
                    CGoGNout << "--- FIN DEPLACEMENT DE SOMMETS ---" << CGoGNendl;
                    view->setMouseTracking(false);
                }
                break;
            default:
                break;
            }
        }
    }
}

void Surface_DeformationCage_Plugin::mouseMove(View* view, QMouseEvent* event)
{
    if(m_movingVertices && m_schnapps->getSelectedMap()->getName().compare("Cages") == 0)
    {
        if(m_movingVerticesInitiated)
        {
            qglviewer::Vec currentPixel(event->x(), event->y(), 0.5);
            qglviewer::Vec currentPoint = view->camera()->unprojectedCoordinatesOf(currentPixel);
            PFP2::VEC3 deplacement = PFP2::VEC3(currentPoint[0] - m_lastMousePosition[0], currentPoint[1] - m_lastMousePosition[1], 0.f);

            MapHandlerGen* mhg_selected = m_schnapps->getSelectedMap();
            MapHandler<PFP2>* mh_selected = static_cast<MapHandler<PFP2>*>(mhg_selected);
            PFP2::MAP* selected = mh_selected->getMap();

            VertexAttribute<PFP2::VEC3> positionSelectedMap = selected->getAttribute<PFP2::VEC3, VERTEX>("position");
            if(!positionSelectedMap.isValid())
            {
                CGoGNout << "Position attribute isn't valid" << CGoGNendl;
                exit(-1);
            }

            CellSelectorGen* selectedSelector = m_schnapps->getSelectedSelector(VERTEX);
            if(selectedSelector)
            {
                CellSelector<VERTEX>* selectedVerticesSelector = mhg_selected->getCellSelector<VERTEX>(selectedSelector->getName());

                const std::vector<Dart>& selectedDarts = selectedVerticesSelector->getSelectedCells();

                for(std::vector<Dart>::const_iterator it = selectedDarts.begin(); it != selectedDarts.end(); ++it)
                {
                    positionSelectedMap[*it] += deplacement;
                }
                m_lastMousePosition = currentPoint;

                mh_selected->notifyAttributeModification(positionSelectedMap);
                mh_selected->updateBB(positionSelectedMap);

                view->updateGL();
            }
        }
        else
        {
            qglviewer::Vec currentPixel(event->x(), event->y(), 0.5);
            m_lastMousePosition = view->camera()->unprojectedCoordinatesOf(currentPixel);
            m_movingVerticesInitiated = true;
        }
    }
}

void Surface_DeformationCage_Plugin::boundarySliderValueChanged(int value)
{
    MapHandlerGen* mhg_object = m_schnapps->getMap("Model");
    MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(mhg_object);

    MapHandlerGen* mhg_cage = m_schnapps->getMap("Cages");
    MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(mhg_cage);

    PFP2::REAL h = value/10.f;

    computeBoundaryWeights(mh_cage, mh_object, h, false);

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
    MapHandlerGen* mhg_modified = static_cast<MapHandlerGen*>(QObject::sender());
    MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(mhg_modified);
    if(orbit == VERTEX && nameAttr=="position" && mh_cage)
    {
        if(mh_cage->getName() == "VCages")
        {
            //Quand la cage virtuelle est déplacée par la cage
            PFP2::MAP* vcages = mh_cage->getMap();

            VertexAttribute<PFP2::VEC3> positionVCages = vcages->getAttribute<PFP2::VEC3, VERTEX>("position");

            MapHandlerGen* mhg_object = m_schnapps->getMap("Model");
            MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(mhg_object);
            PFP2::MAP* object = mh_object->getMap();

            VertexAttribute<PFP2::VEC3> positionObject = object->getAttribute<PFP2::VEC3, VERTEX>("position");
            VertexAttribute<SpacePoint> spacePointObject = object->getAttribute<SpacePoint, VERTEX>("SpacePoint");

            VertexAttribute<PFP2::VEC3> firstPositionVCages = vcages->getAttribute<PFP2::VEC3, VERTEX>("FirstPosition");

            if(spacePointObject.isValid())
            {
                //Si les calculs de poids ont déjà été effectués
                TraversorV<PFP2::MAP> trav_vert_object(*object);
                for(Dart d = trav_vert_object.begin(); d != trav_vert_object.end(); d = trav_vert_object.next())
                {
                    if(spacePointObject[d].isInitialized())
                    {
                        Eigen::Matrix<PFP2::REAL, 1, 2> objectPositionEigen, objectFirstPositionEigen;
                        objectPositionEigen.setZero(1, 2);
                        objectFirstPositionEigen.setZero(1, 2);

                        PFP2::REAL sumTot(0.f);

                        for(int i = 0; i < spacePointObject[d].getNbAssociatedCages(); ++i)
                        {
                            sumTot += smoothingFunction(spacePointObject[d].getCageBoundaryWeight(i), spacePointObject[d].getCageHParameter(i));
                        }

                        for(int i = 0; i < spacePointObject[d].getNbAssociatedCages(); ++i)
                        {

                            PFP2::REAL sumCur(0.f);

                            sumCur = smoothingFunction(spacePointObject[d].getCageBoundaryWeight(i), spacePointObject[d].getCageHParameter(i));

                            PFP2::REAL sumCurNorm = sumCur/sumTot;  //Somme normalisée pour le mélange des différentes déformations appliquées

                            Eigen::Matrix<PFP2::REAL, 1, Eigen::Dynamic>* cageWeights = spacePointObject[d].getCageWeights(i);

                            int j = 0;
                            Traversor2FV<PFP2::MAP> trav_vert_face_cage(*vcages, spacePointObject[d].getCageDart(i));
                            for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
                            {
                                objectPositionEigen(0, 0) += (*cageWeights)(0, j) * positionVCages[dd][0] * sumCurNorm * sumCur;
                                objectPositionEigen(0, 1) += (*cageWeights)(0, j) * positionVCages[dd][1] * sumCurNorm * sumCur;
                                objectFirstPositionEigen(0, 0) += (*cageWeights)(0, j) * firstPositionVCages[dd][0] * sumCurNorm * (1.f-sumCur);
                                objectFirstPositionEigen(0, 1) += (*cageWeights)(0, j) * firstPositionVCages[dd][1] * sumCurNorm * (1.f-sumCur);
                                ++j;
                            }
                        }

                        positionObject[d][0] = objectPositionEigen(0, 0) + objectFirstPositionEigen(0, 0);
                        positionObject[d][1] = objectPositionEigen(0, 1) + objectFirstPositionEigen(0, 1);
                    }
                }

                if(mh_object)
                {
                    mh_object->updateBB(positionObject);
                    mh_object->notifyAttributeModification(positionObject);
                }
            }
        }
        else if(mh_cage->getName() == "Cages")
        {
            //Quand la cage avec laquelle l'utilisateur interagit est modifiée
            PFP2::MAP* cage = mh_cage->getMap();

            VertexAttribute<PFP2::VEC3> positionCage = cage->getAttribute<PFP2::VEC3, VERTEX>("position");
            if(!positionCage.isValid())
            {
                CGoGNout << "Position attribute chosen for the cage isn't valid" << CGoGNendl;
                exit(-1);
            }

            MapHandlerGen* mhg_vcages = m_schnapps->getMap("VCages");
            MapHandler<PFP2>* mh_vcages = static_cast<MapHandler<PFP2>*>(mhg_vcages);

            if(mh_vcages)
            {
                PFP2::MAP* vcages = mh_vcages->getMap();

                VertexAttribute<PFP2::VEC3> positionVCages = vcages->getAttribute<PFP2::VEC3, VERTEX>("position");
                VertexAttribute<SpacePoint> spacePointVCages = vcages->getAttribute<SpacePoint, VERTEX>("SpacePoint");
                VertexAttribute<PFP2::VEC3> linkCagesVCages = vcages->getAttribute<PFP2::VEC3, VERTEX>("LinkCages");

                if(positionVCages.isValid() && spacePointVCages.isValid() && linkCagesVCages.isValid())
                {
                    //Si les calculs de poids ont déjà été effectués
                    TraversorV<PFP2::MAP> trav_vert_vcages(*vcages);
                    for(Dart d = trav_vert_vcages.begin(); d != trav_vert_vcages.end(); d = trav_vert_vcages.next())
                    {
                        positionVCages[d] = positionCage[cage->phi_1(d)]+linkCagesVCages[d];
                    }
                    mh_vcages->updateBB(positionVCages);
                    mh_vcages->notifyAttributeModification(positionVCages);
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

void Surface_DeformationCage_Plugin::computeAllPointsFromObject(const QString& objectName, const QString& cageName,
                                                                const QString& objectNameAttr, const QString& cageNameAttr, bool onlyInside)
{
    MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(m_schnapps->getMap(cageName));
    MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(m_schnapps->getMap(objectName));

    if(mh_cage && mh_object)
    {
        PFP2::MAP* cage = mh_cage->getMap();
        PFP2::MAP* object = mh_object->getMap();

        VertexAttribute<PFP2::VEC3> colorObject = object->getAttribute<PFP2::VEC3, VERTEX>("color");
        if(!colorObject.isValid())
        {
            colorObject = object->addAttribute<PFP2::VEC3, VERTEX>("color");
        }

        VertexAttribute<PFP2::VEC3> positionObject = object->getAttribute<PFP2::VEC3, VERTEX>(objectNameAttr.toStdString());
        if(!positionObject.isValid())
        {
            CGoGNout << "Position attribute chosen for the object isn't valid" << CGoGNendl;
            exit(-1);
        }

        VertexAttribute<SpacePoint> spacePointObject = object->getAttribute<SpacePoint, VERTEX>("SpacePoint");
        if(!spacePointObject.isValid())
        {
            spacePointObject = object->addAttribute<SpacePoint, VERTEX>("SpacePoint");
            mh_object->registerAttribute(spacePointObject);
        }

        VertexAttribute<PFP2::VEC3> positionCage = cage->getAttribute<PFP2::VEC3, VERTEX>(cageNameAttr.toStdString());
        if(!positionCage.isValid())
        {
            CGoGNout << "Position attribute chosen for the cage isn't valid" << CGoGNendl;
            exit(-1);
        }

        VertexAttribute<PFP2::VEC3> firstPositionCage = cage->getAttribute<PFP2::VEC3, VERTEX>("FirstPosition");
        if(!firstPositionCage.isValid() && onlyInside)
        {
            firstPositionCage = cage->addAttribute<PFP2::VEC3, VERTEX>("FirstPosition");
            mh_cage->registerAttribute(firstPositionCage);
        }

        TraversorF<PFP2::MAP> trav_face_cage(*cage);
        for(Dart d = trav_face_cage.begin(); d != trav_face_cage.end(); d = trav_face_cage.next())
        {
            if(!cage->isBoundaryMarked2(d))
            {
                if(onlyInside)
                {
                    //Lors de l'association des points de l'espace avec la VCage
                    Geom::BoundingBox<PFP2::VEC3> bb;

                    //On enregistre la position initiale de chaque sommet de la cage
                    Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, d);
                    for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
                    {
                        firstPositionCage[dd] = positionCage[dd];
                        bb.addPoint(positionCage[dd]);
                    }

                    TraversorV<PFP2::MAP> trav_vert_object(*object);
                    for(Dart dd = trav_vert_object.begin(); dd != trav_vert_object.end(); dd = trav_vert_object.next())
                    {
                        if(Algo::Surface::Geometry::isPointInConvexFace2D<PFP2>(*cage, d, positionCage, positionObject[dd]))
                        {
                            spacePointObject[dd].addCage(d, cage->faceDegree(d));
                            computePointMVCFromCage(dd, positionObject, positionCage,
                                                    spacePointObject[dd].getCageWeights(spacePointObject[dd].getNbAssociatedCages()-1),
                                                    cage, d, cage->faceDegree(d));
                            spacePointObject[dd].addCageHParameter(M_H);
                        }
                    }
                }
            }
        }

        if(onlyInside)
        {
            computeBoundaryWeights(mh_cage, mh_object);
        }
        mh_cage->notifyAttributeModification(positionCage);     //JUSTE POUR DEBUG SANS DEPLACER DE SOMMETS DE CAGE
    }
}

void Surface_DeformationCage_Plugin::computeBoundaryWeights(MapHandler<PFP2>* mh_cage, MapHandler<PFP2>* mh_object, const PFP2::REAL h, bool recalcul)
{
    PFP2::MAP* cage = mh_cage->getMap();
    PFP2::MAP* object = mh_object->getMap();

    TraversorV<PFP2::MAP> trav_vert_object(*object);
    VertexAttribute<SpacePoint> spacePointObject = object->getAttribute<SpacePoint, VERTEX>("SpacePoint");
    if(!spacePointObject.isValid())
    {
        CGoGNout << "SpacePointObject attribute is not valid" << CGoGNendl;
        exit(-1);
    }

    VertexAttribute<PFP2::VEC3> colorObject = object->getAttribute<PFP2::VEC3, VERTEX>("color");
    if(!colorObject.isValid())
    {
        colorObject = object->addAttribute<PFP2::VEC3, VERTEX>("color");
    }

    PFP2::VEC3 color;

    for(Dart d = trav_vert_object.begin(); d != trav_vert_object.end(); d = trav_vert_object.next())
    {
        if(spacePointObject[d].isInitialized())
        {
            for(int i = 0; i < spacePointObject[d].getNbAssociatedCages(); ++i)
            {
                if(recalcul)
                {
                    spacePointObject[d].setCageBoundaryWeight(i, boundaryWeightFunction(spacePointObject[d].getCageWeights(i)));
                }
                else
                {
                    spacePointObject[d].setCageHParameter(i, h);
                }
//                color = Utils::color_map_BCGYR(smoothingFunction(spacePointObject[d].getCageBoundaryWeight(i), spacePointObject[d].getCageHParameter(i)));
//                colorObject[d] = PFP2::VEC3(color[0], color[1], color[2]);
            }
        }
    }

    m_colorVBO->updateData(colorObject);

    m_toDraw = true;

    m_schnapps->getSelectedView()->updateGL();
}

void Surface_DeformationCage_Plugin:: computePointMVCFromCage(Dart vertex, const VertexAttribute<PFP2::VEC3>& positionObject,
                                                             const VertexAttribute<PFP2::VEC3>& positionCage,
                                                             Eigen::Matrix<PFP2::REAL, 1, Eigen::Dynamic>* weights,
                                                             PFP2::MAP* cage, Dart beginningDart, int cageNbV)
{
    PFP2::REAL sumMVC(0.f);
    int i = 0;

    bool stop = false;
    Dart next, prev;

    weights->setZero(1, cageNbV);

    Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, beginningDart);
    for(Dart d = trav_vert_face_cage.begin(); d != trav_vert_face_cage.end() && !stop; d = trav_vert_face_cage.next())
    {
        next = cage->phi1(d);
        prev = cage->phi_1(d);

        (*weights)(0, i) = computeMVC2D(positionObject[vertex], d, next, prev, positionCage, cage);
        sumMVC += (*weights)(0, i);
        ++i;
    }

    //Coordinates normalization
    for(i=0; i<weights->cols(); ++i)
    {
        (*weights)(0, i) /= sumMVC;
    }
}

PFP2::REAL Surface_DeformationCage_Plugin::modifyingFunction(const PFP2::REAL x)
{
    if(x < 0.f)
    {
        return 0.f;
    }
    else if(x < 1.f)
    {
        return -2.f*x*x*(x-1.5f);
    }
    else if(x < 1.5f)
    {
        return 1.f + (x - 1.f) * (x - 1.f);
    }
    else
    {
        return x - 0.25f;
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

        PFP2::REAL ui= (Bjk + (Bij*(nij*njk)) + (Bki*(nki*njk)))/(2.f*ei*njk);

        sumU+=ui;

        it = cage->phi<21>(it);
    }
    while(it!=vertex);

    return (1.0f/r)*sumU;
}

void Surface_DeformationCage_Plugin::resetWeightsCalculated()
{
    MapHandlerGen* mhg_model = m_schnapps->getMap("Model");
    MapHandler<PFP2>* mh_model = static_cast<MapHandler<PFP2>*>(mhg_model);
    PFP2::MAP* model = mh_model->getMap();

    VertexAttribute<SpacePoint> spacePointModel = model->getAttribute<SpacePoint, VERTEX>("SpacePoint");
    if(spacePointModel.isValid())
    {
        for(unsigned int i = spacePointModel.begin(); i != spacePointModel.end(); spacePointModel.next(i))
        {
            spacePointModel[i].removeCages();
        }
    }
}

PFP2::REAL Surface_DeformationCage_Plugin::computeMVC2D(const PFP2::VEC3& pt, Dart current, Dart next, Dart previous,
                                                        const VertexAttribute<PFP2::VEC3>& positionCage, PFP2::MAP* cage)
{
    PFP2::REAL res(1.f);

    const PFP2::VEC3 c = positionCage[current];
    const PFP2::VEC3 c_prev = positionCage[previous];
    const PFP2::VEC3 c_next = positionCage[next];

    bool positiveAngle_prev = Geom::testOrientation2D(pt, c_prev, c) == Geom::LEFT;
    bool positiveAngle_next = Geom::testOrientation2D(pt, c, c_next) == Geom::LEFT;

    PFP2::VEC3 v = c - pt ;
    PFP2::VEC3 v_prev = c_prev - pt ;
    PFP2::VEC3 v_next = c_next - pt ;

    PFP2::REAL b_prev = Geom::angle(v_prev, v);
    PFP2::REAL b_next = Geom::angle(v, v_next);

    if(!positiveAngle_prev)
    {
        b_prev *= -1;
    }

    if(!positiveAngle_next)
    {
        b_next *= -1;
    }

    res = (tan(b_prev/2.f) + tan(b_next/2.f)) / v.norm();

    return res;
}

PFP2::REAL Surface_DeformationCage_Plugin::boundaryWeightFunction(const Eigen::Matrix<PFP2::REAL, 1, Eigen::Dynamic>* weights)
{
    PFP2::REAL res = 1.f;
    for(int i = 0; i < weights->cols(); ++i)
    {
        res *= (1.f - ((*weights)(0, i) + (*weights)(0, (i+1)%weights->cols())));
    }
    return res/std::pow(2.f/weights->cols(), weights->cols());
}

PFP2::REAL Surface_DeformationCage_Plugin::smoothingFunction(const PFP2::REAL x, const PFP2::REAL h)
{
    return (std::cos(M_PI*(x/h + 1.f))+1.f)*0.5f;
}

bool Surface_DeformationCage_Plugin::isInCage(const PFP2::VEC3& point, const PFP2::VEC3& min, const PFP2::VEC3& max)
{
    if(point[0] > min[0] && point[1] > min[1]
            && point[0] < max[0] && point[1] < max[1])
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

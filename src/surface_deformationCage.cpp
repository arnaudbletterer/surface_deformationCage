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

    m_clearingCages = false;

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
            case Qt::Key_R:
                resetWeightsCalculated();
                m_clearingCages = true;
                clearCages();
                m_clearingCages = false;
                //CGoGNout << "Coordonnées réinitialisées" << CGoGNendl;
                view->updateGL();
                break;
            case Qt::Key_A:
                //CGoGNout << "Association de la cage au modèle .." << CGoGNflush;
                resetWeightsCalculated();
                computeAllPointsFromObject("VCages", "Cages", "position", "position");
                computeAllPointsFromObject("Model", "VCages", "position", "position", true);
                //CGoGNout << ".. fait" << CGoGNendl;
            default:
                break;
            }
        }
        else
        {
            switch(event->key())
            {
            case Qt::Key_D:
                if(!m_movingVertices)
                {
                    //CGoGNout << "--- DEBUT DEPLACEMENT DE SOMMETS ---" << CGoGNendl;
                    m_movingVertices = true;
                    m_movingVerticesInitiated = false;
                    view->setMouseTracking(true);
                }
                else
                {
                    m_movingVertices = false;
                    m_movingVerticesInitiated = false;
                    //CGoGNout << "--- FIN DEPLACEMENT DE SOMMETS ---" << CGoGNendl;
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

            VertexAttribute<PFP2::VEC3, PFP2::MAP> positionSelectedMap = mh_selected->getAttribute<PFP2::VEC3, VERTEX>("position");
            if(!positionSelectedMap.isValid())
            {
                //CGoGNout << "Position attribute isn't valid" << CGoGNendl;
                exit(-1);
            }

            CellSelectorGen* selectedSelector = m_schnapps->getSelectedSelector(VERTEX);
            if(selectedSelector && mh_selected->getGenericMap()->getNbCells(VERTEX) > 0)
            {
                CellSelector<PFP2::MAP, VERTEX>* selectedVerticesSelector = static_cast<CellSelector<PFP2::MAP, VERTEX>*>(selectedSelector);

                const std::vector<Cell<VERTEX> >& selectedDarts = selectedVerticesSelector->getSelectedCells();

                for(std::vector<Cell<VERTEX> >::const_iterator it = selectedDarts.begin(); it != selectedDarts.end(); ++it)
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
    //    MapHandlerGen* mhg_object = m_schnapps->getMap("Model");
    //    MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(mhg_object);

    //    MapHandlerGen* mhg_cage = m_schnapps->getMap("Cages");
    //    MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(mhg_cage);

    //    PFP2::REAL h = value/10.f;

    //    computeBoundaryWeights(mh_object, false);

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
    if(!m_clearingCages)
    {
        MapHandlerGen* mhg_modified = static_cast<MapHandlerGen*>(QObject::sender());
        MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(mhg_modified);
        if(orbit == VERTEX && nameAttr=="position" && mh_cage)
        {
            if(mh_cage->getName() == "VCages")
            {
                //Quand la cage virtuelle est déplacée par la cage
                PFP2::MAP* vcages = mh_cage->getMap();

                VertexAttribute<PFP2::VEC3, PFP2::MAP> positionVCages = mh_cage->getAttribute<PFP2::VEC3, VERTEX>("position");
                VertexAttribute<PFP2::VEC3, PFP2::MAP> firstPositionVCages = mh_cage->getAttribute<PFP2::VEC3, VERTEX>("FirstPosition");
                FaceAttribute<PFP2::REAL, PFP2::MAP> minBoundaryWeightVCages = mh_cage->getAttribute<PFP2::REAL, FACE>("MinBoundaryWeight");

                MapHandlerGen* mhg_object = m_schnapps->getMap("Model");
                MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(mhg_object);
                PFP2::MAP* object = mh_object->getMap();

                VertexAttribute<PFP2::VEC3, PFP2::MAP> positionObject = mh_object->getAttribute<PFP2::VEC3, VERTEX>("position");
                VertexAttribute<SpacePoint, PFP2::MAP> spacePointObject = mh_object->getAttribute<SpacePoint, VERTEX>("SpacePoint");

                if(spacePointObject.isValid())
                {
                    //Si les calculs de poids ont déjà été effectués
                    TraversorV<PFP2::MAP> trav_vert_object(*object);
                    for(Dart d = trav_vert_object.begin(); d != trav_vert_object.end(); d = trav_vert_object.next())
                    {
                        if(spacePointObject[d].isInitialized())
                        {
                            Eigen::Matrix<PFP2::REAL, 1, 2> objectPositionEigen, currentObjectPositionEigen;
                            objectPositionEigen.setZero(1, 2);
                            currentObjectPositionEigen.setZero(1, 2);

                            PFP2::REAL sumTot(0.f);

                            for(int i = 0; i < spacePointObject[d].getNbAssociatedCages(); ++i)
                            {
                                sumTot += smoothingFunction(spacePointObject[d].getCageBoundaryWeight(i),
                                                            minBoundaryWeightVCages[spacePointObject[d].getCageDart(i)]);
                            }

                            for(int i = 0; i < spacePointObject[d].getNbAssociatedCages(); ++i)
                            {

                                PFP2::REAL sumCur(0.f);

                                sumCur = smoothingFunction(spacePointObject[d].getCageBoundaryWeight(i),
                                                           minBoundaryWeightVCages[spacePointObject[d].getCageDart(i)]);

                                PFP2::REAL sumCurNorm = sumCur/sumTot; //Somme normalisée pour le mélange des différentes déformations appliquées

                                Eigen::Matrix<PFP2::REAL, 1, Eigen::Dynamic>* cageWeights = spacePointObject[d].getCageWeights(i);

                                currentObjectPositionEigen.setZero(1, 2);

                                int j = 0;
                                Traversor2FV<PFP2::MAP> trav_vert_face_cage(*vcages, spacePointObject[d].getCageDart(i));
                                for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
                                {
                                    currentObjectPositionEigen(0, 0) += ((*cageWeights)(0, j) * positionVCages[dd][0] * sumCur
                                            + (*cageWeights)(0, j) * firstPositionVCages[dd][0] * (1.f-sumCur));
                                    currentObjectPositionEigen(0, 1) += ((*cageWeights)(0, j) * positionVCages[dd][1] * sumCur
                                            + (*cageWeights)(0, j) * firstPositionVCages[dd][1] * (1.f-sumCur));
                                    ++j;
                                }

                                objectPositionEigen += currentObjectPositionEigen * sumCurNorm;
                            }

                            positionObject[d][0] = objectPositionEigen(0, 0);
                            positionObject[d][1] = objectPositionEigen(0, 1);
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

                VertexAttribute<PFP2::VEC3, PFP2::MAP> positionCage = mh_cage->getAttribute<PFP2::VEC3, VERTEX>("position");
                if(!positionCage.isValid())
                {
                    //CGoGNout << "Position attribute chosen for the cage isn't valid" << CGoGNendl;
                    exit(-1);
                }

                MapHandlerGen* mhg_vcages = m_schnapps->getMap("VCages");
                MapHandler<PFP2>* mh_vcages = static_cast<MapHandler<PFP2>*>(mhg_vcages);

                if(mh_vcages)
                {
                    PFP2::MAP* vcages = mh_vcages->getMap();

                    VertexAttribute<PFP2::VEC3, PFP2::MAP> positionVCages = mh_vcages->getAttribute<PFP2::VEC3, VERTEX>("position");
                    VertexAttribute<PFP2::VEC3, PFP2::MAP> linkCagesVCages = mh_vcages->getAttribute<PFP2::VEC3, VERTEX>("LinkCages");

                    if(positionVCages.isValid() && linkCagesVCages.isValid() && vcages->getNbCells(VERTEX) > 0)
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
    //CGoGNout << "computeAllPointsFromObject" << CGoGNendl;
    MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(m_schnapps->getMap(cageName));
    MapHandler<PFP2>* mh_object = static_cast<MapHandler<PFP2>*>(m_schnapps->getMap(objectName));

    if(mh_cage && mh_object && mh_cage->getGenericMap()->getNbCells(VERTEX) > 0 && mh_object->getGenericMap()->getNbCells(VERTEX) > 0)
    {
        PFP2::MAP* cage = mh_cage->getMap();
        PFP2::MAP* object = mh_object->getMap();

        VertexAttribute<PFP2::VEC3, PFP2::MAP> positionObject = mh_object->getAttribute<PFP2::VEC3, VERTEX>(objectNameAttr);
        if(!positionObject.isValid())
        {
            //CGoGNout << "Position attribute chosen for the object isn't valid" << CGoGNendl;
            exit(-1);
        }

        VertexAttribute<PFP2::VEC3, PFP2::MAP> positionCage = mh_cage->getAttribute<PFP2::VEC3, VERTEX>(cageNameAttr);
        if(!positionCage.isValid())
        {
            //CGoGNout << "Position attribute chosen for the cage isn't valid" << CGoGNendl;
            exit(-1);
        }

        if(onlyInside)
        {
            //Lors de l'association des points de l'espace avec la VCage
            VertexAttribute<PFP2::VEC3, PFP2::MAP> firstPositionCage = mh_cage->getAttribute<PFP2::VEC3, VERTEX>("FirstPosition");
            if(!firstPositionCage.isValid() && onlyInside)
            {
                firstPositionCage = mh_cage->addAttribute<PFP2::VEC3, VERTEX>("FirstPosition");
            }

            VertexAttribute<SpacePoint, PFP2::MAP> spacePointObject = mh_object->getAttribute<SpacePoint, VERTEX>("SpacePoint");
            if(!spacePointObject.isValid())
            {
                spacePointObject = mh_object->addAttribute<SpacePoint, VERTEX>("SpacePoint");
            }

            TraversorF<PFP2::MAP> trav_face_cage(*cage);
            for(Dart d = trav_face_cage.begin(); d != trav_face_cage.end(); d = trav_face_cage.next())
            {
                if(!cage->isBoundaryMarked<2>(d))
                {
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
                        }
                    }
                }
            }
            computeBoundaryWeights(mh_object);
            //            computeColorMap(mh_object, mh_cage);

            //            CGoGNStream::Out fichier;
            //            fichier.toFile("/home/bletterer/plot3d_MVC_withFunction.gp");

            //            TraversorV<PFP2::MAP> trav_vert_object(*object);
            //            for(Dart dd = trav_vert_object.begin(); dd != trav_vert_object.end(); dd = trav_vert_object.next())
            //            {
            //                if(fabs(positionObject[dd][1]) < FLT_EPSILON)
            //                {
            //                    if(spacePointObject[dd].getNbAssociatedCages()>0)
            //                    {
            ////                        fichier << positionObject[dd][0] << " " << spacePointObject[dd].getCageBoundaryWeight(0)*16 << CGoGNendl;
            //                        ////CGoGNout << spacePointObject[dd].getCageBoundaryWeight(0) << CGoGNendl;
            //                        fichier << positionObject[dd][0] << " " << smoothingFunction(spacePointObject[dd].getCageBoundaryWeight(0), 1.f/16) << CGoGNendl;
            //                    }
            //                    else
            //                    {
            //                        fichier << positionObject[dd][0] << " " << 0. << CGoGNendl;
            //                    }
            //                }
            //            }

            //            fichier.close();

            VertexAttribute<PFP2::VEC3, PFP2::MAP> colorObject = mh_object->getAttribute<PFP2::VEC3, VERTEX>("color");

            m_colorVBO->updateData(colorObject);

            m_toDraw = true;

            CGoGNout << "Avant" << CGoGNendl;

            m_schnapps->getSelectedView()->updateGL();

            CGoGNout << "Apres" << CGoGNendl;
        }
        else
        {
            //Lors de l'association des cages de contrôle avec les cages d'influence


            FaceAttribute<PFP2::REAL, PFP2::MAP> minBoundaryWeightObject = mh_object->getAttribute<PFP2::REAL, FACE>("MinBoundaryWeight");
            if(!minBoundaryWeightObject.isValid())
            {
                minBoundaryWeightObject = mh_object->addAttribute<PFP2::REAL, FACE>("MinBoundaryWeight");
            }

            FaceAttribute<int, PFP2::MAP> idCageObject = mh_object->getAttribute<int, FACE>("IdCage");
            if(!idCageObject.isValid())
            {
                idCageObject = mh_object->addAttribute<int, FACE>("IdCage");
            }

            FaceAttribute<int, PFP2::MAP> idCageCage = mh_cage->getAttribute<int, FACE>("IdCage");
            if(!idCageCage.isValid())
            {
                idCageCage = mh_cage->addAttribute<int, FACE>("IdCage");
            }

            VertexAttribute<SpacePoint, PFP2::MAP> spacePointCage = mh_cage->getAttribute<SpacePoint, VERTEX>("SpacePoint");
            if(!spacePointCage.isValid())
            {
                spacePointCage = mh_cage->addAttribute<SpacePoint, VERTEX>("SpacePoint");
            }

            TraversorF<PFP2::MAP> trav_face_object(*object);
            for(Dart d =  trav_face_object.begin(); d != trav_face_object.end(); d = trav_face_object.next())
            {
                if(!object->isBoundaryMarked<2>(d))
                {
                    TraversorV<PFP2::MAP> trav_vert_cage(*cage);
                    for(Dart dd = trav_vert_cage.begin(); dd != trav_vert_cage.end(); dd = trav_vert_cage.next())
                    {
                        if(idCageObject[d] == idCageCage[dd])
                        {
                            spacePointCage[dd].addCage(d, object->faceDegree(d));
                            computePointMVCFromCage(dd, positionCage, positionObject,
                                                    spacePointCage[dd].getCageWeights(0),
                                                    object, dd, object->faceDegree(d));
                        }
                    }
                }
            }

            computeBoundaryWeights(mh_cage);

            for(Dart d =  trav_face_object.begin(); d != trav_face_object.end(); d = trav_face_object.next())
            {
                if(!object->isBoundaryMarked<2>(d))
                {
                    PFP2::REAL minBoundaryWeight(2.f); //Valeur supérieure à n'importe quelle poids de bordure (0 <= x <= 1)

                    TraversorV<PFP2::MAP> trav_vert_cage(*cage);
                    for(Dart dd = trav_vert_cage.begin(); dd != trav_vert_cage.end(); dd = trav_vert_cage.next())
                    {
                        if(idCageCage[dd] == idCageObject[d])
                        {
                            if(spacePointCage[dd].getCageBoundaryWeight(0) < minBoundaryWeight)
                            {
                                minBoundaryWeight = spacePointCage[dd].getCageBoundaryWeight(0);
                            }
                        }
                    }

                    minBoundaryWeightObject[d] = minBoundaryWeight;
                }
            }
        }
    }
}

void Surface_DeformationCage_Plugin::computeBoundaryWeights(MapHandler<PFP2>* mh_object)
{
    PFP2::MAP* object = mh_object->getMap();

    VertexAttribute<SpacePoint, PFP2::MAP> spacePointObject = mh_object->getAttribute<SpacePoint, VERTEX>("SpacePoint");
    if(!spacePointObject.isValid())
    {
        //CGoGNout << "SpacePointObject attribute for the object is not valid" << CGoGNendl;
        exit(-1);
    }

    TraversorV<PFP2::MAP> trav_vert_object(*object);
    for(Dart d = trav_vert_object.begin(); d != trav_vert_object.end(); d = trav_vert_object.next())
    {
        if(spacePointObject[d].isInitialized())
        {
            for(int i = 0; i < spacePointObject[d].getNbAssociatedCages(); ++i)
            {
                spacePointObject[d].setCageBoundaryWeight(i, boundaryWeightFunction(spacePointObject[d].getCageWeights(i)));
            }
        }
    }
}

void Surface_DeformationCage_Plugin::computeColorMap(MapHandler<PFP2>* mh_object, MapHandler<PFP2>* mh_cage)
{
    PFP2::MAP* object = mh_object->getMap();

    VertexAttribute<PFP2::VEC3, PFP2::MAP> colorObject = mh_object->getAttribute<PFP2::VEC3, VERTEX>("color");
    if(!colorObject.isValid())
    {
        colorObject = mh_object->addAttribute<PFP2::VEC3, VERTEX>("color");
    }

    VertexAttribute<SpacePoint, PFP2::MAP> spacePointObject = mh_object->getAttribute<SpacePoint, VERTEX>("SpacePoint");
    if(!spacePointObject.isValid())
    {
        spacePointObject = mh_object->addAttribute<SpacePoint, VERTEX>("SpacePoint");
    }

    FaceAttribute<PFP2::REAL, PFP2::MAP> minBoundaryWeightCage = mh_cage->getAttribute<PFP2::REAL, FACE>("MinBoundaryWeight");
    if(!minBoundaryWeightCage.isValid())
    {
        //CGoGNout << "Color attribute for the cage is not valid" << CGoGNendl;
        exit(-1);
    }

    PFP2::VEC3 color;

    TraversorV<PFP2::MAP> trav_vert_object(*object);
    for(Dart d = trav_vert_object.begin(); d != trav_vert_object.end(); d = trav_vert_object.next())
    {
        if(spacePointObject[d].getNbAssociatedCages() > 0)
        {
            for(int i = 0; i < spacePointObject[d].getNbAssociatedCages(); ++i)
            {
                PFP2::REAL weight(smoothingFunction(spacePointObject[d].getCageBoundaryWeight(i), minBoundaryWeightCage[spacePointObject[d].getCageDart(i)]));
                color = Utils::color_map_BCGYR(weight);
                colorObject[d][0] = color[0];
                colorObject[d][1] = color[1];
                colorObject[d][2] = color[2];
            }
        }
        else
        {
            colorObject[d] = PFP2::VEC3(0.f, 0.f, 1.f);
        }
    }

    m_colorVBO->updateData(colorObject);
}

void Surface_DeformationCage_Plugin:: computePointMVCFromCage(Dart vertex, const VertexAttribute<PFP2::VEC3, PFP2::MAP>& positionObject,
                                                              const VertexAttribute<PFP2::VEC3, PFP2::MAP>& positionCage,
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

PFP2::REAL Surface_DeformationCage_Plugin::computeMVC(const PFP2::VEC3& pt, Dart vertex, PFP2::MAP* cage, const VertexAttribute<PFP2::VEC3, PFP2::MAP>& positionCage)
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

    VertexAttribute<SpacePoint, PFP2::MAP> spacePointModel = mh_model->getAttribute<SpacePoint, VERTEX>("SpacePoint");
    if(spacePointModel.isValid())
    {
        for(unsigned int i = spacePointModel.begin(); i != spacePointModel.end(); spacePointModel.next(i))
        {
            spacePointModel[i].removeCages();
        }
    }
}

void Surface_DeformationCage_Plugin::clearCages()
{
    MapHandlerGen* mhg_tmp = m_schnapps->getMap("Cages");
    MapHandler<PFP2>* mh_tmp = static_cast<MapHandler<PFP2>*>(mhg_tmp);

    CellSelector<PFP2::MAP, VERTEX>* cs = static_cast<CellSelector<PFP2::MAP, VERTEX>*>(m_schnapps->getSelectedSelector(VERTEX));
    if(cs)
    {
        cs->unselect(cs->getSelectedCells());
    }

    mh_tmp->clear(false);

    mh_tmp->updateBB(Geom::BoundingBox<PFP2::VEC3>(PFP2::VEC3(0.f, 0.f, 0.f)));

    mhg_tmp = m_schnapps->getMap("VCages");
    mh_tmp = static_cast<MapHandler<PFP2>*>(mhg_tmp);

    mh_tmp->clear(false);

    mh_tmp->updateBB(Geom::BoundingBox<PFP2::VEC3>(PFP2::VEC3(0.f, 0.f, 0.f)));

    //    CellSelectorSet cs_set = mh_tmp->getCellSelectorSet(VERTEX);

    //    //CGoGNout << cs_set.size() << CGoGNendl;

    //    for(CellSelectorSet::ConstIterator it = cs_set.constBegin(); it != cs_set.constEnd(); ++it)
    //    {
    //        CellSelector<PFP2::MAP, VERTEX>* cs = static_cast<CellSelector<PFP2::MAP, VERTEX>*>(it.value());
    //        //CGoGNout << "Avant : " << cs->getNbSelectedCells() << CGoGNendl;
    //        cs->unselect(cs->getSelectedCells());
    //        //CGoGNout << "Après : " << cs->getNbSelectedCells() << CGoGNendl;
    //    }

    mhg_tmp = m_schnapps->getMap("Model");
    mh_tmp = static_cast<MapHandler<PFP2>*>(mhg_tmp);

    VertexAttribute<PFP2::VEC3, PFP2::MAP> colorTmp = mh_tmp->getAttribute<PFP2::VEC3, VERTEX>("color");
    if(!colorTmp.isValid())
    {
        //CGoGNout << "Color attribute is not valid" << CGoGNendl;
        exit(-1);
    }

    //    TraversorV<PFP2::MAP> trav_vert_tmp(*tmp);
    //    for(Dart d = trav_vert_tmp.begin(); d != trav_vert_tmp.end(); d = trav_vert_tmp.next())
    //    {
    //        colorTmp[d] = PFP2::VEC3(0.f, 1.f, 0.f);
    //    }

    //    m_colorVBO->updateData(colorTmp);

}

PFP2::REAL Surface_DeformationCage_Plugin::computeMVC2D(const PFP2::VEC3& pt, Dart current, Dart next, Dart previous,
                                                        const VertexAttribute<PFP2::VEC3, PFP2::MAP>& positionCage, PFP2::MAP* cage)
{
    PFP2::REAL res(1.f);

    const PFP2::VEC3 c = positionCage[current];
    const PFP2::VEC3 c_prev = positionCage[previous];
    const PFP2::VEC3 c_next = positionCage[next];

    bool positiveAngle_prev = Geom::testOrientation2D(pt, c_prev, c) == Geom::LEFT;
    bool positiveAngle_next = Geom::testOrientation2D(pt, c, c_next) == Geom::LEFT;

    PFP2::VEC3 v = c - pt;
    PFP2::VEC3 v_prev = c_prev - pt;
    PFP2::VEC3 v_next = c_next - pt;

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

    return res;
}

PFP2::REAL Surface_DeformationCage_Plugin::smoothingFunction(const PFP2::REAL x, const PFP2::REAL h)
{
    if(x < h)
    {
        return 0.5f * std::sin(M_PI*(x/h - 0.5f)) + 0.5f;
    }

    return 1.f;
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

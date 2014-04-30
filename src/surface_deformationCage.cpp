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
    if(orbit == VERTEX && nameAttr=="position")
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
                            sumTot += smoothingFunction(spacePointObject[d].getCageBoundaryWeight(i));
                        }

                        for(int i = 0; i < spacePointObject[d].getNbAssociatedCages(); ++i)
                        {

                            PFP2::REAL sumCur(0.f);

                            sumCur = smoothingFunction(spacePointObject[d].getCageBoundaryWeight(i));

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

            MapHandlerGen* mhg_vcages = m_schnapps->getMap("VCages");
            MapHandler<PFP2>* mh_vcages = static_cast<MapHandler<PFP2>*>(mhg_vcages);
            PFP2::MAP* vcages = mh_vcages->getMap();

            VertexAttribute<PFP2::VEC3> positionVCages = vcages->getAttribute<PFP2::VEC3, VERTEX>("position");
            VertexAttribute<SpacePoint> spacePointVCages = vcages->getAttribute<SpacePoint, VERTEX>("SpacePoint");

            if(spacePointVCages.isValid())
            {
                //Si les calculs de poids ont déjà été effectués
                TraversorV<PFP2::MAP> trav_vert_vcages(*vcages);
                for(Dart d = trav_vert_vcages.begin(); d != trav_vert_vcages.end(); d = trav_vert_vcages.next())
                {
                    if(spacePointVCages[d].isInitialized())
                    {
                        Eigen::Matrix<PFP2::REAL, 1, 2> vCagesPositionEigen;
                        vCagesPositionEigen.setZero(1, 2);

                        Eigen::Matrix<PFP2::REAL, 1, Eigen::Dynamic>* cageWeights = spacePointVCages[d].getCageWeights(0);

                        int i = 0;
                        Traversor2FV<PFP2::MAP> trav_vert_face_cage(*vcages, spacePointVCages[d].getCageDart(0));
                        for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
                        {
                            vCagesPositionEigen(0, 0) += (*cageWeights)(0, i) * positionCage[dd][0];
                            vCagesPositionEigen(0, 1) += (*cageWeights)(0, i) * positionCage[dd][1];
                            ++i;
                        }

                        positionVCages[d][0] = vCagesPositionEigen(0, 0);
                        positionVCages[d][1] = vCagesPositionEigen(0, 1);
                    }
                }

                if(mh_vcages)
                {
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
            return;
        }

        VertexAttribute<PFP2::VEC3> firstPositionCage = cage->getAttribute<PFP2::VEC3, VERTEX>("FirstPosition");
        if(!firstPositionCage.isValid() && onlyInside)
        {
            firstPositionCage = cage->addAttribute<PFP2::VEC3, VERTEX>("FirstPosition");
            mh_cage->registerAttribute(firstPositionCage);
        }

//        FaceAttribute<int> idCageCage, idCageObject;

//        if(!onlyInside)
//        {

//        }

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
                        if(bb.contains(positionObject[dd]))
                        {
                            CGoGNout << "yes" << CGoGNendl;
                            spacepointobject[dd].addCage(d, cage->faceDegree(d));
                            computePointMVCFromCage(dd, positionObject, positionCage,
                                                    spacePointObject[dd].getCageWeights(spacePointObject[dd].getNbAssociatedCages()-1),
                                                    cage, d, cage->faceDegree(d));
                        }
                    }
                }
                else
                {
                    //Lors de l'association de la Cage avec la VCage

                    FaceAttribute<int> idCageCage = cage->getAttribute<int, FACE>("IdCage");
                    if(!idCageCage.isValid())
                    {
                        CGoGNout << "IdCage attribute chosen for the cage isn't valid" << CGoGNendl;
                        return;
                    }

                    FaceAttribute<int> idCageObject = object->getAttribute<int, FACE>("IdCage");
                    if(!idCageObject.isValid())
                    {
                        CGoGNout << "IdCage attribute chosen for the object isn't valid" << CGoGNendl;
                        return;
                    }

                    TraversorV<PFP2::MAP> trav_vert_object(*object);
                    for(Dart dd = trav_vert_object.begin(); dd != trav_vert_object.end(); dd = trav_vert_object.next())
                    {
                        if(idCageObject[dd] == idCageCage[d])
                        {
                            //Si la VCage courante corrspond bien à celle devant être associée à la Cage courante
                            spacePointObject[dd].addCage(d, cage->faceDegree(d));
                            computePointMVCFromCage(dd, positionObject, positionCage,
                                                    spacePointObject[dd].getCageWeights(0), cage, d, cage->faceDegree(d));
                        }
                    }
                }
            }
        }

        if(onlyInside)
        {
            computeBoundaryWeights(cage, object);
        }
        mh_cage->notifyAttributeModification(positionCage);     //JUSTE POUR DEBUG SANS DEPLACER DE SOMMETS DE CAGE
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

void Surface_DeformationCage_Plugin::computeFirstDerivative(PFP2::MAP* cage)
{
    Eigen::Matrix<PFP2::REAL, 2, Eigen::Dynamic> vertices, derivatives;
    Eigen::MatrixXf coefficients;
    int nbV = 0;

    VertexAttribute<PFP2::VEC3> positionCage = cage->getAttribute<PFP2::VEC3, VERTEX>("position");
    if(!positionCage.isValid())
    {
        CGoGNout << "Position attribute chosen for the cage isn't valid" << CGoGNendl;
    }

    FaceAttribute<FirstDerivative> firstDerivative = cage->getAttribute<FirstDerivative, FACE>("FirstDerivative");
    if(!firstDerivative.isValid())
    {
        firstDerivative = cage->addAttribute<FirstDerivative, FACE>("FirstDerivative");
    }

    TraversorF<PFP2::MAP> trav_face_cage(*cage);
    for(Dart d = trav_face_cage.begin(); d != trav_face_cage.end(); d = trav_face_cage.next())
    {
        if(!cage->isBoundaryMarked2(d))
        {
            nbV = cage->faceDegree(d);
            vertices.setZero(2, nbV);
            derivatives.setZero(2, nbV);
            coefficients.setZero(nbV, nbV);
            firstDerivative[d].setBeginningDart(d);
            firstDerivative[d].setNbVertices(nbV);

//            //Initialization of coefficients matrix
//            int i = 0;
//            for(i = 0; i < nbV; ++i)
//            {
//                coefficients(i, i) = 4.f;
//                coefficients(i, (i+1)%nbV) = 1.f;
//                if(i==0)
//                {
//                    coefficients(i, nbV-1) = 1.f;
//                }
//                else
//                {
//                    coefficients(i, i-1) = 1.f;
//                }
//            }

//            //Initialization of vertices matrix
//            i = 0;
//            Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, d);
//            for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
//            {
//                vertices(0, i) = (3 * (positionCage[cage->phi1(dd)][0] - positionCage[cage->phi_1(dd)][0]));
//                vertices(1, i) = (3 * (positionCage[cage->phi1(dd)][1] - positionCage[cage->phi_1(dd)][1]));
//                ++i;
//            }

//            //Forward elimination (Gaussian elimination)
//            //First we want only "zeros" on the left hand side of the diagonal
//            //To do so we substract the row above from the current row

//            for(i = 1; i < nbV-1; ++i)
//            {
//                for(int j = 0; j < nbV; ++j)
//                {
//                    coefficients(i, j) = (coefficients(i-1, j) * (-coefficients(i,i-1) / coefficients(i-1, i-1))) + coefficients(i, j);
//                }
//                vertices(0, i) = (vertices(0, i-1) * (-coefficients(i, i-1) / coefficients(i-1, i-1))) + vertices(0, i);
//                vertices(1, i) = (vertices(1, i-1) * (-coefficients(i, i-1) / coefficients(i-1, i-1))) + vertices(1, i);
//            }

//            //For the last row we have to successively remove the leftmost non-zero value
//            for(i = 0; i < nbV-1; ++i)
//            {
//                for(int j = 0; j < nbV; ++j)
//                {
//                    coefficients(nbV-1, j) = (coefficients(i,j) * (-coefficients(nbV-1,i) / coefficients(i, i))) + coefficients(nbV-1, j);
//                }

//                vertices(0, nbV-1) = (vertices(0, i) * (-coefficients(nbV-1, i) / coefficients(i, i))) + vertices(0, nbV-1);
//                vertices(1, nbV-1) = (vertices(1, i) * (-coefficients(nbV-1, i) / coefficients(i, i))) + vertices(1, nbV-1);

//                //We also set the value of the element of the current row 'i' on the diagonal to be 1
//                PFP2::REAL rowDiagonalValue = coefficients(i ,i);

//                vertices(0, i) /= rowDiagonalValue;
//                vertices(1, i) /= rowDiagonalValue;
//                for(int j = 0; j < nbV; ++j)
//                {
//                    coefficients(i, j) /= rowDiagonalValue;
//                }
//            }

//            vertices(0, nbV-1) /= coefficients(nbV-1, nbV-1);
//            vertices(1, nbV-1) /= coefficients(nbV-1, nbV-1);

//            //We set the value of the element of the last row on the diagonal to be 1 too
//            for(i = 0; i < nbV; ++i)
//            {
//                coefficients(nbV-1, i) /= coefficients(nbV-1, nbV-1);
//            }

            //Backward substitution
            //Now we simply have to successively resolve the equations by starting from the bottom of the matrix
            //(Because the answer of the last equation is trivial)

//            firstDerivative[d].m_verticesDerivatives(0, nbV-1) = vertices(0, nbV-1);
//            firstDerivative[d].m_verticesDerivatives(1, nbV-1) = vertices(0, nbV-1);

//            for(i = nbV-2; i > 0; --i)
//            {
//                firstDerivative[d].m_verticesDerivatives(0, i) = vertices(0, i)
//                        - coefficients(i, i+1) * firstDerivative[d].m_verticesDerivatives(0, i+1);
//                firstDerivative[d].m_verticesDerivatives(1, i) = vertices(1, i)
//                        - coefficients(i, i+1) * firstDerivative[d].m_verticesDerivatives(1, i+1);
//            }

//            //For the first row of the matrix, we need to consider the last column too, as it has a non-zero value
//            firstDerivative[d].m_verticesDerivatives(0, 0) = vertices(0, 0)
//                    - coefficients(0, nbV-1) * firstDerivative[d].m_verticesDerivatives(0, nbV-1)
//                    - coefficients(0, 1) * firstDerivative[d].m_verticesDerivatives(0, 1);

//            firstDerivative[d].m_verticesDerivatives(1, 0) = vertices(1, 0)
//                    - coefficients(0, nbV-1) * firstDerivative[d].m_verticesDerivatives(1, nbV-1)
//                    - coefficients(0, 1) * firstDerivative[d].m_verticesDerivatives(1, 1);

//            CGoGNout << "--Matrice des coefficients --" << CGoGNendl;
//            for(i = 0; i < nbV; ++i)
//            {
//                for(int j = 0; j < nbV; ++j)
//                {
//                    CGoGNout << coefficients(i,j) << ", " << CGoGNflush;
//                }
//                CGoGNout << CGoGNendl;
//            }

//            for(i = 0; i < nbV; ++i)
//            {
//                CGoGNout << "Tangente : [" << firstDerivative[d].m_verticesDerivatives(0, i) << ","
//                         <<  firstDerivative[d].m_verticesDerivatives(1, i) << "]" << CGoGNendl;
//            }

            int i = 0;
            Traversor2FV<PFP2::MAP> trav_vert_face_cage(*cage, d);
            for(Dart dd = trav_vert_face_cage.begin(); dd != trav_vert_face_cage.end(); dd = trav_vert_face_cage.next())
            {
                for(int j = 0; j < 2 ; ++j)
                {
                    firstDerivative[d].m_verticesDerivatives[i](j, j) = 1.f;
                }
                ++i;
            }
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
    return res;
}

PFP2::REAL Surface_DeformationCage_Plugin::smoothingFunction(const PFP2::REAL x, const PFP2::REAL h)
{
    return (std::cos(M_PI*(x*10.f + 1.f))+1.f)*0.5;
//    if(x >= h)
//    {
//        return 0.f;
//    }

//    if(h > FLT_EPSILON*10000)
//    {
//        //return (1/4. * std::cos(M_PI*(x/h)) + 1/4.);
//        return (std::cos(M_PI*(x/h + 1.f))+1.f)*0.5;
//        //return (1/4. * std::sin(M_PI*(x/h-1/2.)) + 1/4.);
//        //return -2*(x/h)*(x/h)*(x/h) + 3*(x/h)*(x/h);
//        //return -8*(x/h)*(x/h)*(x/h)*(x/h)*(x/h) + 20*(x/h)*(x/h)*(x/h)*(x/h) - 18*(x/h)*(x/h)*(x/h) + 7*(x/h)*(x/h);
//    }

//    return 0.;
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

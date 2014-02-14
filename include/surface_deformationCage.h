#ifndef _SURFACE_DEFORMATIONCAGE_PLUGIN_H_
#define _SURFACE_DEFORMATIONCAGE_PLUGIN_H_

#include "plugin_interaction.h"

#include "Utils/colorMaps.h"

#include <cmath>

#include "dialog_deformationCage.h"

#include "mapHandler.h"

#include "Eigen/Geometry"

#include "spacePoint.h"

namespace CGoGN
{

namespace SCHNApps
{

#define M_H 1.f

class Surface_DeformationCage_Plugin : public PluginInteraction
{
	Q_OBJECT
	Q_INTERFACES(CGoGN::SCHNApps::Plugin)

public:
    Surface_DeformationCage_Plugin()
	{}

    ~Surface_DeformationCage_Plugin()
	{}

	virtual bool enable();
	virtual void disable();

    virtual void draw(View* view) {}
    virtual void drawMap(View* view, MapHandlerGen* map);

    virtual void keyPress(View* view, QKeyEvent* event) {}
    virtual void keyRelease(View* view, QKeyEvent* event) {}
    virtual void mousePress(View* view, QMouseEvent* event) {}
    virtual void mouseRelease(View* view, QMouseEvent* event) {}
    virtual void mouseMove(View* view, QMouseEvent* event) {}
    virtual void wheelEvent(View* view, QWheelEvent* event) {}

    virtual void viewLinked(View* view) {}
    virtual void viewUnlinked(View* view) {}

    void computeMVCFromDialog();

private :
    void computeBoundaryWeights(PFP2::MAP* cage, PFP2::MAP* object);

    void computePointMVCFromCage(Dart vertex, const VertexAttribute<PFP2::VEC3>& positionObject,
                                 const VertexAttribute<PFP2::VEC3>& positionCage,
                                 Eigen::Matrix<PFP2::REAL, Eigen::Dynamic, 1>& weights,
                                 PFP2::MAP* cage, Dart beginningDart, int cageNbV);

    PFP2::REAL computeMVC(const PFP2::VEC3& pt, Dart vertex, PFP2::MAP* cage,
                          const VertexAttribute<PFP2::VEC3>& positionCage);
    PFP2::REAL computeMVC2D(const PFP2::VEC3& pt, Dart current, Dart next, Dart previous,
                            const VertexAttribute<PFP2::VEC3>& positionCage);

    /*
     *Fonctions de l'article
     */
    void boundaryWeightFunction(const Eigen::Matrix<PFP2::REAL, Eigen::Dynamic, 1>& coordinates, Dart beginningDart,
                                std::vector<PFP2::REAL>& boundaryWeights, PFP2::MAP* cage, int nbAdjCages);

    PFP2::REAL smoothingFunction(const PFP2::REAL& x, const PFP2::REAL& h = M_H);

    std::vector<Dart> findJoinCage(PFP2::MAP* cage, Dart beginningDart);

    bool isInCage(PFP2::VEC3 point, PFP2::VEC3 min, PFP2::VEC3 max);

private slots:
	void mapAdded(MapHandlerGen* map);
	void mapRemoved(MapHandlerGen* map);

    void attributeModified(unsigned int orbit, QString nameAttr);

    void openDeformationCageDialog();

    void boundarySliderValueChanged(int value);

public slots:
    void computeAllPointsFromObject(const QString& objectName, const QString& cageName, const QString& objectNameAttr, const QString& cageNameAttr);

private:
    Dialog_DeformationCage* m_deformationCageDialog;
    QAction* m_deformationCageAction;

protected:
    CGoGN::Utils::ShaderColorPerVertex* m_colorPerVertexShader;
    Utils::VBO* m_positionVBO;
    Utils::VBO* m_colorVBO;

    bool m_toDraw;
};

} // namespace SCHNApps

} // namespace CGoGN

#endif


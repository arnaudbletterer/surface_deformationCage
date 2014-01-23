#ifndef _SURFACE_DEFORMATIONCAGE_PLUGIN_H_
#define _SURFACE_DEFORMATIONCAGE_PLUGIN_H_

#include "plugin_interaction.h"

#include <cmath>

#include "Algo/Modelisation/voxellisation.h"
#include "Algo/Modelisation/triangulation.h"
#include "Algo/MC/marchingcube.h"
#include "Algo/Import/import.h"

#include "Utils/chrono.h"

#include "dialog_deformationCage.h"

#include "mapHandler.h"

#include "MVCCoordinates.h"

#include "Algo/Modelisation/voxellisation.h"

#include "vCage.h"

namespace CGoGN
{

namespace SCHNApps
{

#define M_H 0.6

struct MapParameters
{
    MapParameters();
    ~MapParameters();

    void start();
    void stop();

    bool m_initialized;
    bool m_linked;
    bool m_toComputeMVC;
};

struct CageParameters
{
    VertexAttribute<PFP2::VEC3> cagePosition;
    MapHandlerGen* controlledObject;
    VertexAttribute<PFP2::VEC3> controlledObjectPosition;

    Eigen::MatrixXf coordinatesEigen;
    Eigen::Matrix<float, Eigen::Dynamic, 3> cagePositionEigen;
    Eigen::Matrix<float, Eigen::Dynamic, 3> objectPositionEigen;

    Eigen::Matrix<Dart, Eigen::Dynamic, 1> dartObjectIndicesEigen;
};

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

private slots:
	void mapAdded(MapHandlerGen* map);
	void mapRemoved(MapHandlerGen* map);

    void attributeModified(unsigned int orbit, QString nameAttr);

    void openDeformationCageDialog();

    void computePointMVCFromCage(Dart vertex, const VertexAttribute<PFP2::VEC3>& positionObject,
                                 const VertexAttribute<PFP2::VEC3>& positionCage,
                                 Eigen::MatrixXf& coordinates, int index,
                                 const std::vector<Dart>& vCage, PFP2::MAP* cage);
    PFP2::REAL computeMVC(const PFP2::VEC3& pt, Dart vertex, PFP2::MAP* cage,
                          const VertexAttribute<PFP2::VEC3>& positionCage);
    PFP2::REAL computeMVC2D(const PFP2::VEC3& pt, Dart vertex, PFP2::MAP* cage,
                            const VertexAttribute<PFP2::VEC3>& position);

    PFP2::REAL boundaryWeightFunction(const std::vector<Dart>& vCage, PFP2::MAP* cage,
                                      const Eigen::MatrixXf& coordinatesEigen, int index);
    PFP2::REAL smoothingFunction(const PFP2::REAL& x, const PFP2::REAL& h = M_H);

public slots:
    void computeAllPointsFromObject(const QString& objectName, const QString& cageName, const QString& objectNameAttr, const QString& cageNameAttr);

private:
    Dialog_DeformationCage* m_deformationCageDialog;
    QAction* m_deformationCageAction;

public:
    QHash<QString, MapParameters> h_parameterSet;
    QHash<MapHandlerGen*, CageParameters> h_cageParameters;

protected:
    CGoGN::Utils::ShaderColorPerVertex* m_colorPerVertexShader;
    Utils::VBO* m_positionVBO;
    Utils::VBO* m_colorVBO;

    bool m_toDraw;
};

} // namespace SCHNApps

} // namespace CGoGN

#endif


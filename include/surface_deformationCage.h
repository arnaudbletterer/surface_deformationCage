#ifndef _SURFACE_DEFORMATIONCAGE_PLUGIN_H_
#define _SURFACE_DEFORMATIONCAGE_PLUGIN_H_

#include "plugin_processing.h"

#include "Algo/Modelisation/voxellisation.h"
#include "Algo/Modelisation/triangulation.h"
#include "Algo/MC/marchingcube.h"
#include "Algo/Import/import.h"

#include "Utils/chrono.h"

#include "dialog_deformationCage.h"

#include "mapHandler.h"

#include "MVCCoordinates.h"

#include "Algo/Modelisation/voxellisation.h"

namespace CGoGN
{

namespace SCHNApps
{

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
};

class Surface_DeformationCage_Plugin : public PluginProcessing
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

    void computeMVCFromDialog();

private slots:
	void mapAdded(MapHandlerGen* map);
	void mapRemoved(MapHandlerGen* map);

    void attributeModified(unsigned int orbit, QString nameAttr);

    void openDeformationCageDialog();

    void computePointMVCFromCage(const PFP2::VEC3& pt, PFP2::MAP* cage, unsigned int cageNbV,
                                 const VertexAttribute<PFP_STANDARD::VEC3>& position, Eigen::MatrixXf& coordinates, int index);
    bool isInCage(const PFP2::VEC3& min, const PFP2::VEC3& max, const PFP2::VEC3& pt);
    PFP2::REAL computeMVC(const PFP2::VEC3& pt, Dart vertex, PFP2::MAP* cage, const VertexAttribute<PFP2::VEC3>& position);
    PFP2::REAL computeMVC2D(const PFP2::VEC3& pt, Dart vertex, PFP2::MAP* cage, const VertexAttribute<PFP2::VEC3>& position);

public slots:
    void computeAllPointsFromObject(const QString& objectName, const QString& cageName, const QString& objectNameAttr, const QString& cageNameAttr);

private:
    Dialog_DeformationCage* m_deformationCageDialog;
    QAction* m_deformationCageAction;

public:
    QHash<QString, MapParameters> h_parameterSet;
    QHash<MapHandlerGen*, CageParameters> h_cageParameters;
};

} // namespace SCHNApps

} // namespace CGoGN

#endif


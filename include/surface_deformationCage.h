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
    std::vector<PFP2::VEC3> m_coordinates;
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
    void moveObjectsPointsFromCageMovement(MapHandlerGen* o, MapHandlerGen* c, const QString& objectNameAttr, const QString& cageNameAttr);

    void openDeformationCageDialog();

    void computeAllPointsFromObject(const QString& objectName, const QString& cageName, const QString& objectNameAttr, const QString& cageNameAttr);
    void computePointMVCFromCage(Dart vertex, const QString& objectName, const QString& cageName, const QString& cageNameAttr,
                                 VertexAttribute<PFP_STANDARD::VEC3> position, PFP2::MAP* objectMap, PFP2::MAP* cageMap, VertexAttribute<MVCCoordinates>& coordinates);
    PFP2::REAL computeMVC(PFP2::VEC3 p, Dart vertex, PFP2::MAP* cage, const QString& cageNameAttr);

public slots:

private:
    Dialog_DeformationCage* m_deformationCageDialog;
    QAction* m_deformationCageAction;

public:
    QHash<QString, MapParameters> h_parameterSet;
};

} // namespace SCHNApps

} // namespace CGoGN

#endif


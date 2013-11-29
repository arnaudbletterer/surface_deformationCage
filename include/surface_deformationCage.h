#ifndef _SURFACE_DEFORMATIONCAGE_PLUGIN_H_
#define _SURFACE_DEFORMATIONCAGE_PLUGIN_H_

#include "plugin_processing.h"

#include "dialog_deformationCage.h"

#include "Algo/Modelisation/voxellisation.h"
#include "Algo/Modelisation/triangulation.h"
#include "Algo/MC/marchingcube.h"
#include "Algo/Import/import.h"

#include "Utils/chrono.h"

namespace CGoGN
{

namespace SCHNApps
{

struct MapCageParameters
{
    MapCageParameters();
    ~MapCageParameters();

    void start();
    void stop();

    bool m_initialized;
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

private slots:
	void mapAdded(MapHandlerGen* map);
	void mapRemoved(MapHandlerGen* map);
    void attributeModified(unsigned int orbit, QString nameAttr);

    void currentMapSelectedChangedFromDialog();
    void currentAttributeIndexChangedFromDialog(QString nameAttr);

    void openDeformationCageDialog();

public slots:

private:
    Dialog_DeformationCage* m_deformationCageDialog;
    QAction* m_deformationCageAction;

public:
    QHash<QString, MapCageParameters> h_parameterSet;
};

} // namespace SCHNApps

} // namespace CGoGN

#endif


#ifndef _DIALOG_DEFORMATIONCAGE_H_
#define _DIALOG_DEFORMATIONCAGE_H_

#include "ui_dialog_deformationCage.h"

#include "Geometry/vector_gen.h"

#include "mapHandler.h"

namespace CGoGN
{

namespace SCHNApps
{

class SCHNApps;
class MapHandlerGen;
class Surface_DeformationCage_Plugin;
struct MapParameters;

class Dialog_DeformationCage : public QDialog, public Ui::Dialog_DeformationCage
{
    Q_OBJECT

public:
    Dialog_DeformationCage(SCHNApps* s, Surface_DeformationCage_Plugin* p);

    void updateAppearanceFromPlugin();

private:
    SCHNApps* m_schnapps;
    MapHandlerGen* m_selectedObject;
    MapHandlerGen* m_selectedCage;
    Surface_DeformationCage_Plugin* m_plugin;

public slots:
    void addMapToLists(MapHandlerGen* m);
    void removeMapFromLists(MapHandlerGen* m);
    void addAttributeToObjectList(unsigned int orbit, const QString& nameAttr);
    void addAttributeToCageList(unsigned int orbit, const QString& nameAttr);

    void selectedObjectChanged();
    void selectedCageChanged();

    void linkStateClicked();

    MapHandlerGen* getSelectedObject();
    MapHandlerGen* getSelectedCage();
};

} // namespace SCHNApps

} // namespace CGoGN

#endif

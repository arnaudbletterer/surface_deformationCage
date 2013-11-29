#ifndef _DIALOG_DEFORMATIONCAGE_H_
#define _DIALOG_DEFORMATIONCAGE_H_

#include "ui_dialog_deformationCage.h"

#include "Geometry/vector_gen.h"

namespace CGoGN
{

namespace SCHNApps
{

class SCHNApps;
class MapHandlerGen;
class Surface_DeformationCage_Plugin;
class MapCageParameters;

class Dialog_DeformationCage : public QDialog, public Ui::Dialog_DeformationCage
{
    Q_OBJECT

public:
    Dialog_DeformationCage(SCHNApps* s);

    /*
      * Fonction qui met  jour l'apparence de l'interface en fonction de la configuration de chaque carte
      */
    void updateAppearanceFromPlugin(bool independant, bool initialized) {

    }

private:
    SCHNApps* m_schnapps;
    MapHandlerGen* m_selectedMap;

public slots:
    void addMapToList(MapHandlerGen* m);
    void removeMapFromList(MapHandlerGen* m);
    void addAttributeToList(unsigned int orbit, const QString& nameAttr);

    void selectedMapChanged();
};

} // namespace SCHNApps

} // namespace CGoGN

#endif

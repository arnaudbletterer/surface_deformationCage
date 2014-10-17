#include "dialog_deformationCage.h"
#include "surface_deformationCage.h"
#include "schnapps.h"

namespace CGoGN
{

namespace SCHNApps
{

Dialog_DeformationCage::Dialog_DeformationCage(SCHNApps* s, Surface_DeformationCage_Plugin* p) :
    m_schnapps(s),
    m_selectedObject(NULL),
    m_selectedCage(NULL),
    m_plugin(p)
{
    setupUi(this);

    connect(m_schnapps, SIGNAL(mapAdded(MapHandlerGen*)), this, SLOT(addMapToLists(MapHandlerGen*)));
    connect(m_schnapps, SIGNAL(mapRemoved(MapHandlerGen*)), this, SLOT(removeMapFromLists(MapHandlerGen*)));

    connect(list_objects, SIGNAL(itemSelectionChanged()), this, SLOT(selectedObjectChanged()));
    connect(list_cages, SIGNAL(itemSelectionChanged()), this, SLOT(selectedCageChanged()));

    connect(button_linkState, SIGNAL(clicked()), this, SLOT(linkStateClicked()));

    foreach(MapHandlerGen* map,  m_schnapps->getMapSet().values())
    {
        QListWidgetItem* item = new QListWidgetItem(map->getName(), list_objects);
        item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
        item = new QListWidgetItem(map->getName(), list_cages);
        item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
    }
}

void Dialog_DeformationCage::updateAppearanceFromPlugin()
{
    //    if(m_selectedObject && m_selectedCage)
    //    {
    //        MapHandler<PFP2>* mh_cage = static_cast<MapHandler<PFP2>*>(m_selectedCage);
    //        PFP2::MAP* cage = mh_cage->getMap();

    //        if(m_plugin->h_cageParameters.contains())
    //        {
    //            CageParameters& p = m_plugin->h_cageParameters[m_selectedCage];
    //            if(p.controlledObject==m_selectedObject)
    //            {
    //                group_linkState->setEnabled(true);
    //                button_linkState->setText(QString("Unlink"));
    //                progress_link->setValue(100);
    //            }
    //            else
    //            {
    //                group_linkState->setEnabled(false);
    //                button_linkState->setText(QString("Link"));
    //                progress_link->setValue(0);
    //            }
    //        }
    //        else
    //        {
    //            group_linkState->setEnabled(true);
    //            button_linkState->setText(QString("Link"));
    //            progress_link->setValue(0);
    //        }
    //    }
    //    else
    //    {
    //        button_linkState->setText(QString("Link"));
    //        group_linkState->setEnabled(false);
    //        progress_link->setValue(0);
    //    }
    //    combo_objectPositionAttribute->setEnabled(m_selectedObject);
    //    combo_cagePositionAttribute->setEnabled(m_selectedCage);
}

void Dialog_DeformationCage::addMapToLists(MapHandlerGen* m)
{
    QListWidgetItem* item = new QListWidgetItem(m->getName(), list_objects);
    item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
    item = new QListWidgetItem(m->getName(), list_cages);
    item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
}

void Dialog_DeformationCage::removeMapFromLists(MapHandlerGen* m)
{
    QList<QListWidgetItem*> items = list_objects->findItems(m->getName(), Qt::MatchExactly);
    if(!items.empty())
        delete items[0];

    items = list_cages->findItems(m->getName(), Qt::MatchExactly);
    if(!items.empty())
        delete items[0];

    if(m_selectedObject == m)
    {
        disconnect(m_selectedObject, SIGNAL(attributeAdded(unsigned int, const QString&)), this, SLOT(addAttributeToObjectList(unsigned int, const QString&)));
        m_selectedObject = NULL;
    }

    if(m_selectedCage == m)
    {
        disconnect(m_selectedCage, SIGNAL(attributeAdded(unsigned int, const QString&)), this, SLOT(addAttributeToCageList(unsigned int, const QString&)));
        m_selectedCage = NULL;
    }
}

void Dialog_DeformationCage::addAttributeToObjectList(unsigned int orbit, const QString& nameAttr)
{
    QString vec3TypeName = QString::fromStdString(nameOfType(PFP2::VEC3()));

    const QString& typeAttr = m_selectedObject->getAttributeTypeName(orbit, nameAttr);

    if(typeAttr == vec3TypeName)
    {   //On n'ajoute l'élment que s'il est de type Vec3
        //C'est-à-dire s'il peut être interprêté comme un point
        combo_objectPositionAttribute->addItem(nameAttr);
    }
}

void Dialog_DeformationCage::addAttributeToCageList(unsigned int orbit, const QString& nameAttr)
{
    QString vec3TypeName = QString::fromStdString(nameOfType(PFP2::VEC3()));

    const QString& typeAttr = m_selectedCage->getAttributeTypeName(orbit, nameAttr);

    if(typeAttr == vec3TypeName)
    {   //On n'ajoute l'élment que s'il est de type Vec3
        //C'est-à-dire s'il peut être interprêté comme un point
        combo_cagePositionAttribute->addItem(nameAttr);
    }
}

void Dialog_DeformationCage::selectedObjectChanged()
{
    if(m_selectedObject) {
        disconnect(m_selectedObject, SIGNAL(attributeAdded(unsigned int, const QString&)), this, SLOT(addAttributeToObjectList(unsigned int, const QString&)));
    }

    QList<QListWidgetItem*> currentItems = list_objects->selectedItems();
    if(!currentItems.empty())
    {
        combo_objectPositionAttribute->clear();

        const QString& mapname = currentItems[0]->text();
        MapHandlerGen* mh = m_schnapps->getMap(mapname);

        QString vec3TypeName = QString::fromStdString(nameOfType(PFP2::VEC3()));

        unsigned int j = 0;
        const AttributeSet& attribs = mh->getAttributeSet(VERTEX);
        for(AttributeSet::const_iterator i = attribs.constBegin(); i != attribs.constEnd(); ++i)
        {
            if(i.value() == vec3TypeName)
            {
                combo_objectPositionAttribute->addItem(i.key());
                ++j;
            }
        }

        m_selectedObject = mh;
        connect(m_selectedObject, SIGNAL(attributeAdded(unsigned int, const QString&)), this, SLOT(addAttributeToObjectList(unsigned int, const QString&)));

        if(m_selectedCage)
        {
            updateAppearanceFromPlugin();
        }

    }
    else
        m_selectedObject = NULL;
}

void Dialog_DeformationCage::selectedCageChanged()
{
    if(m_selectedCage)
    {
        disconnect(m_selectedCage, SIGNAL(attributeAdded(unsigned int, const QString&)), this, SLOT(addAttributeToCageList(unsigned int, const QString&)));
    }

    QList<QListWidgetItem*> currentItems = list_cages->selectedItems();
    if(!currentItems.empty())
    {
        combo_cagePositionAttribute->clear();

        const QString& mapname = currentItems[0]->text();
        MapHandlerGen* mh = m_schnapps->getMap(mapname);

        QString vec3TypeName = QString::fromStdString(nameOfType(PFP2::VEC3()));

        unsigned int j = 0;
        const AttributeSet& attribs = mh->getAttributeSet(VERTEX);
        for(AttributeSet::const_iterator i = attribs.constBegin(); i != attribs.constEnd(); ++i)
        {
            if(i.value() == vec3TypeName)
            {
                combo_cagePositionAttribute->addItem(i.key());
                ++j;
            }
        }

        m_selectedCage = mh;
        connect(m_selectedCage, SIGNAL(attributeAdded(unsigned int, const QString&)), this, SLOT(addAttributeToCageList(unsigned int, const QString&)));

        if(m_selectedObject)
        {
            updateAppearanceFromPlugin();
        }
    }
    else
        m_selectedCage = NULL;
}

MapHandlerGen* Dialog_DeformationCage::getSelectedObject()
{
    return m_selectedObject;
}

MapHandlerGen* Dialog_DeformationCage::getSelectedCage()
{
    return m_selectedCage;
}

void Dialog_DeformationCage::linkStateClicked()
{
    //    if(m_selectedObject && m_selectedCage)
    //    {
    //        if(!m_plugin->h_cageParameters.contains(m_selectedCage))
    //        {
    //            //S'il n'existait pas déjà un objet associé à la cage courante
    //            m_plugin->computeMVCFromDialog();
    //        }
    //        else
    //        {
    //            CageParameters p = m_plugin->h_cageParameters[m_selectedCage];
    //            if(p.controlledObject==m_selectedObject
    //                    && p.cagePosition.name()==combo_cagePositionAttribute->currentText().toStdString()
    //                    && p.controlledObjectPosition.name()==combo_objectPositionAttribute->currentText().toStdString())
    //            {
    //                //Si les deux éléments étaient actuellement liés, on les délie
    //                m_plugin->h_cageParameters.remove(m_selectedCage);
    //            }
    //        }
    //        updateAppearanceFromPlugin();
    //    }
}
} // namespace SCHNApps

} // namespace CGoGN

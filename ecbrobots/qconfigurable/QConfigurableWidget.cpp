/***************************************************************************
 *   Copyright (C) 2010 by                                                 *
 *   Research Network for Self-Organization of Robot Behavior              *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    wrabe@informatik.uni-leipzig.de                                      *
 *    Georg.Martius@mis.mpg.de                                             *
 *    ralfder@mis.mpg.de                                                   *
 *    frank@nld.ds.mpg.de                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.11  2010-12-15 17:26:28  wrabe
 *   - number of colums for tileWidgets and width of tileWidgets can
 *   now be changed (independently for each Configurable)
 *   - bugfixes
 *
 *   Revision 1.10  2010/12/15 11:00:06  wrabe
 *   -load/save multiple ConfigurableStates from one file
 *   -All current ConfigurableStates can be stored and loaded now via menu
 *   -loading a ConfigurableState for one Configurable from a file containing multiple ConfigurableStates allows to choose one desired ConfigurableState
 *
 *   Revision 1.9  2010/12/14 10:10:12  guettler
 *   -autoload/autosave now uses only one xml file
 *   -fixed getName of TileWidget which produced invisible widgets in xml files
 *
 *   Revision 1.8  2010/12/13 16:22:18  wrabe
 *   - autosave function rearranged
 *   - bugfixes
 *
 *   Revision 1.7  2010/12/09 17:00:08  wrabe
 *   - load / save function of ConfigurableState (configurable + GUI)
 *   - autoload / autosave function of ConfigurableState (configurable
 *     + GUI)
 *   - handling of equal Configurable names implemented for autoload
 *     and -save
 *   - bugfixing
 *
 *   Revision 1.6  2010/12/08 17:52:57  wrabe
 *   - bugfixing/introducing new feature:
 *   - folding of the ConfigurableWidgets now awailable
 *   - highlight the ConfigurableTile when hoovered by mouse
 *   - load/store of the state of a ConfigurableWidget to file
 *
 *   Revision 1.5  2010/12/03 11:11:53  wrabe
 *   - now handled paramVal, paramInt and paramBool, all the params are displayed
 *     as ConfigurableTiles witch can be show and hide seperatly or arranged by user
 *     (showHideDialog reacheble by contextMenu (right click an the Widget containing
 *     the tiles ), arrange the Tiles is can done by drag and drop (there is no history or
 *     storage implementet yet))
 *
 *   Revision 1.4  2010/11/30 17:19:03  wrabe
 *   - bugfix
 *
 *   Revision 1.3  2010/11/30 17:07:06  wrabe
 *   - new class QConfigurableTileShowHideDialog
 *   - try to introduce user-arrangeable QConfigurationTiles (current work, not finished)
 *
 *   Revision 1.2  2010/11/28 20:33:44  wrabe
 *   - current state of work: only paramval´s
 *   - construct a configurable as a tile containing a QSlider to change the value by drag with mouse as well as a QSpinBox to change the configurable by typing new values (mouse-scrolls are also supported)
 *   - minimum and maximum boundaries can´t be changed will be so far, only a change- dialog-dummy is reacable over the context-menu
 *
 *   Revision 1.1  2010/11/26 12:22:36  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *   - bugfixes
 *   - current development state of QConfigurable (Qt GUI)
 *
 *                                                                         *
 ***************************************************************************/

#include "QConfigurableWidget.h"
#include <QtGui>
#include <QtXml>
#include <QFile>

#include "QBoolConfigurableTileWidget.h"
#include "QIntConfigurableTileWidget.h"
#include "QValConfigurableTileWidget.h"
#include "QConfigurableTileShowHideDialog.h"
#include "QConfigurableLoadSaveDialog.h"
#include "QChangeNumberTileColumnsDialog.h"

namespace lpzrobots {
  
  QConfigurableWidget::QConfigurableWidget(Configurable* config, int nameIndex) :
    config(config), dragging(false), isCollapsed(false), configurableTile_dragging(0), nameIndex(nameIndex), numberTilesPerLine(3) {
    initBody();
    createConfigurableLines();
    setAcceptDrops(true);
  }

  QConfigurableWidget::~QConfigurableWidget() {
    foreach(QAbstractConfigurableTileWidget* tileWidget, configTileWidgetMap){
      disconnect(tileWidget, SIGNAL(sig_resize(QSize)));
      disconnect(this, SIGNAL(sig_tileWidgetResize(QSize)));
    }
  }

  void QConfigurableWidget::createConfigurableLines() {


    setLayout(&layout);
    int tileIndex = 0;

    Configurable::parammap valMap = config->getParamValMap();
    FOREACHC(Configurable::parammap, valMap, keyIt) {
      Configurable::paramkey key = (*keyIt).first;
      QAbstractConfigurableTileWidget* configTileWidget = new QValConfigurableTileWidget(config, key);
      configTileWidgetMap.insert(configTileWidget->getConfigurableName(), configTileWidget);
      configTileWidget->setTileIndex(tileIndex);
      connect(configTileWidget, SIGNAL(sig_resize(QSize)), this, SIGNAL(sig_tileWidgetResize(QSize)));
      connect(this, SIGNAL(sig_tileWidgetResize(QSize)), configTileWidget, SLOT(sl_resize(QSize)));
      //layout.addWidget(configTileWidget, numberWidgets / numberTilesPerLine, numberWidgets % numberTilesPerLine);
      tileIndex++;
    }

    Configurable::paramintmap intMap = config->getParamIntMap();
    FOREACHC(Configurable::paramintmap, intMap, keyIt) {
      Configurable::paramkey key = (*keyIt).first;
      QAbstractConfigurableTileWidget* configTileWidget = new QIntConfigurableTileWidget(config, key);
      configTileWidgetMap.insert(configTileWidget->getConfigurableName(), configTileWidget);
      configTileWidget->setTileIndex(tileIndex);
      connect(configTileWidget, SIGNAL(sig_resize(QSize)), this, SIGNAL(sig_tileWidgetResize(QSize)));
      connect(this, SIGNAL(sig_tileWidgetResize(QSize)), configTileWidget, SLOT(sl_resize(QSize)));
      //layout.addWidget(configTileWidget, numberWidgets / 3, numberWidgets % 3);
      tileIndex++;
    }
    Configurable::paramboolmap boolMap = config->getParamBoolMap();
    FOREACHC(Configurable::paramboolmap, boolMap, keyIt) {
      Configurable::paramkey key = (*keyIt).first;
      QAbstractConfigurableTileWidget* configTileWidget = new QBoolConfigurableTileWidget(config, key);
      configTileWidgetMap.insert(configTileWidget->getConfigurableName(), configTileWidget);
      configTileWidget->setTileIndex(tileIndex);
      connect(configTileWidget, SIGNAL(sig_resize(QSize)), this, SIGNAL(sig_tileWidgetResize(QSize)));
      connect(this, SIGNAL(sig_tileWidgetResize(QSize)), configTileWidget, SLOT(sl_resize(QSize)));
      //layout.addWidget(configTileWidget, numberWidgets / 3, numberWidgets % 3);
      tileIndex++;
    }

    arrangeConfigurableTiles();
    //body.setLayout(&layout);
  }

  void QConfigurableWidget::initBody() {
    //    setTitle(QString(config->getName().c_str()) + "  -  " + QString(config->getRevision().c_str()) + "  [" + QString::number(config->getId()) + "]");
    setTitle(QString(config->getName().c_str()) + " (" + QString::number(nameIndex) + ")");
    configName = QString(config->getName().c_str()) + "_" + QString::number(nameIndex);
    setFont(QFont("Courier", 11, QFont::Bold));

    QPalette pal = palette();
    pal.setColor(QPalette::AlternateBase, QColor(200, 210, 200));
    setPalette(pal);
    setBackgroundRole(QPalette::AlternateBase);
    setAutoFillBackground(true);

    // Prepare the context menu to show the configurable show and hide dialog
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(sl_execContextMenu(const QPoint &)));
    contextMenuShowHideDialog.addAction("set number of tile columns", this, SLOT(sl_changeNumberTileColumns()));
    contextMenuShowHideDialog.addAction("show/hide parameters", this, SLOT(sl_showAndHideParameters()));
    contextMenuShowHideDialog.addAction("load configurable state from file ...", this, SLOT(sl_loadConfigurableStateFromFile()));
    contextMenuShowHideDialog.addAction("save current configurable state to file ...", this, SLOT(sl_saveConfigurableStateToFile()));
  }

  void QConfigurableWidget::sl_execContextMenu(const QPoint & pos) {
    if (!isCollapsed) {
      contextMenuShowHideDialog.exec(this->mapToGlobal(pos));
    }
  }

  void QConfigurableWidget::sl_changeNumberTileColumns() {
    QChangeNumberTileColumnsDialog* dialog = new QChangeNumberTileColumnsDialog(&numberTilesPerLine);
    dialog->exec();
    arrangeConfigurableTiles();
  }

  void QConfigurableWidget::sl_loadConfigurableStateFromFile() {
    QFileDialog* fileDialog = new QFileDialog();
    fileDialog->setAcceptMode(QFileDialog::AcceptOpen);
    fileDialog->setFileMode(QFileDialog::ExistingFile);
    QString pathApplication = QCoreApplication::applicationDirPath() + "/*.xml";
    fileDialog->selectFile(pathApplication);
    fileDialog->setNameFilter(tr("Xml (*.xml)"));
    if (fileDialog->exec() == QDialog::Accepted) {
      QFile file(fileDialog->selectedFiles().at(0));
      if (!file.open(QIODevice::ReadOnly))
        return;

      QDomDocument doc("ConfigurableStateTypeDefinition");
      if (!doc.setContent(&file)) {
        file.close();
        return;
      }
      file.close();

      QDomElement qde_configurableStates = doc.documentElement();
      if (qde_configurableStates.tagName() != "ConfigurableStates")
        return;

      // generate qde_configurableStateMap
      QHash<QString,QDomElement> qde_configurableStateMap;
      QDomNodeList qdn_List = qde_configurableStates.elementsByTagName("ConfigurableState");
      for (int i = 0; i < qdn_List.size(); i++)
        qde_configurableStateMap.insert(qdn_List.at(i).toElement().attribute("name"), qdn_List.at(i).toElement());

      QMap<QString, QConfigurableWidget*> configurableWidgetMap;
      configurableWidgetMap.insert(getName(), this);

      QConfigurableLoadSaveDialog* dialog = new QConfigurableLoadSaveDialog(configurableWidgetMap, qde_configurableStateMap, QConfigurableLoadSaveDialog::ConfigurableLoadSingle);
      dialog->exec();
      //QMessageBox::warning(this, this->title(), tr("Configurable state could not be opened."), QMessageBox::Close);
    }
  }
  void QConfigurableWidget::sl_saveConfigurableStateToFile() {
    QFileDialog* fileDialog = new QFileDialog();
    fileDialog->setAcceptMode(QFileDialog::AcceptSave);
    //fileDialog->setFileMode(QFileDialog::AnyFile);
    fileDialog->setNameFilter(tr("Xml (*.xml)"));
    fileDialog->setDefaultSuffix("xml");
    QString pathApplication = QCoreApplication::applicationDirPath() + "/";
    QString fileNamePreference = pathApplication + QString(config->getName().c_str());
    fileDialog->selectFile(fileNamePreference);
    if (fileDialog->exec() == QDialog::Accepted) {
      QString fileName = fileDialog->selectedFiles().at(0);
      if (!saveConfigurableState(fileName)) {
        QMessageBox::warning(this, this->title(), tr("Configurable state could not be saved."), QMessageBox::Close);
      }
    }
  }


  int QConfigurableWidget::loadConfigurableState(const QString &fileName) {
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
      return -1;

    QDomDocument doc("ConfigurableStateTypeDefinition");
    if (!doc.setContent(&file)) {
      file.close();
      return -2;
    }
    file.close();

    QDomElement qde_configurableStates = doc.documentElement();
    if (qde_configurableStates.tagName() != "ConfigurableStates")
      return -3;

    return fromXml(qde_configurableStates.elementsByTagName("ConfigurableState").at(0).toElement());
  }

  int QConfigurableWidget::fromXml(const QDomElement &qde_configurableState) {
    // GUI-Elements...
    QDomElement qde_configurableWidget = qde_configurableState.firstChildElement("ConfigurableWidget");
    if (!qde_configurableWidget.isNull()) {
      // By default there exists only one node named "ConfigurableWidget"!
      QString collapse = qde_configurableWidget.attribute("isCollapsed", "true");
      numberTilesPerLine = qde_configurableWidget.attribute("numberTilesPerLine", "4").toInt();
      int tileWidgetWidth = qde_configurableWidget.attribute("tileWidgetWidth", QString::number(QAbstractConfigurableTileWidget::defaultWidgetSize.width())).toInt();
      QDomNode qdn_configurableTileWidgets = qde_configurableWidget.elementsByTagName("ConfigurableTileWidgets").at(0);
      // By default there will only one list of ConfigurableTileWidgets accepted!
      QDomElement qde_configurableTileWidget = qdn_configurableTileWidgets.firstChild().toElement();
      int tmpTileIndex = 0;
      while (!qde_configurableTileWidget.isNull()) {
        QString tileName = qde_configurableTileWidget.attribute("name", "???");
        int tileIndex = qde_configurableTileWidget.attribute("tileIndex", QString::number(tmpTileIndex++)).toInt();
        QString visible = qde_configurableTileWidget.attribute("isVisible", "true");
        QAbstractConfigurableTileWidget* tileWidget = configTileWidgetMap.value(tileName);
        if (tileWidget != 0) {
          tileWidget->setTileIndex(tileIndex);
          tileWidget->sl_resize(QSize(tileWidgetWidth,QAbstractConfigurableTileWidget::defaultWidgetSize.height()));
          if (visible.startsWith("true"))
            tileWidget->show();
          else
            tileWidget->hide();
        }
        qde_configurableTileWidget = qde_configurableTileWidget.nextSiblingElement();
      }
      arrangeConfigurableTiles();
      setFolding(collapse.startsWith("true"));
    } else
      return -4;

    // data...
    QDomElement qde_configurable = qde_configurableState.firstChildElement("Configurable");
    if (!qde_configurable.isNull()) {
      // By default there exists only one node named "Configurable"!
      // this node has attributes named: revision, name
      // this node has childs named <paramvals>, <paramints> and <parambools>
      QDomNode qdn_paramvals = qde_configurable.elementsByTagName("paramvals").at(0);
      // By default there exists only one node named "paramvals", meaning there is only one list ...
      QDomElement qde_paramval = qdn_paramvals.firstChild().toElement();
      while (!qde_paramval.isNull()) {
        QString key = qde_paramval.attribute("name", "???");
        QString desc = qde_paramval.attribute("description");
        double value = qde_paramval.attribute("value").toDouble();
        double minBound = qde_paramval.attribute("minBound").toDouble();
        double maxBound = qde_paramval.attribute("maxBound").toDouble();
        if (config->getParamValMap().find(key.toStdString()) != config->getParamValMap().end() && configTileWidgetMap.contains(key)) {
          config->setParamBounds(key.toStdString(), minBound, maxBound);
          config->setParam(key.toStdString(), value);
          config->setParamDescr(key.toStdString(), desc.toStdString());
          configTileWidgetMap.value(key)->reloadConfigurableData();
        }
        qde_paramval = qde_paramval.nextSiblingElement();
      }
      QDomNode qdn_paramints = qde_configurable.elementsByTagName("paramints").at(0);
      // By default there exists only one node named "paramints", meaning there is only one list ...
      QDomElement qde_paramint = qdn_paramints.firstChild().toElement();
      while (!qde_paramint.isNull()) {
        QString key = qde_paramint.attribute("name", "???");
        QString desc = qde_paramint.attribute("description");
        int value = qde_paramint.attribute("value").toInt();
        int minBound = qde_paramint.attribute("minBound").toInt();
        int maxBound = qde_paramint.attribute("maxBound").toInt();

        if (config->getParamIntMap().find(key.toStdString()) != config->getParamIntMap().end() && configTileWidgetMap.contains(key)) {
          config->setParamBounds(key.toStdString(), minBound, maxBound);
          config->setParam(key.toStdString(), value);
          config->setParamDescr(key.toStdString(), desc.toStdString());
          configTileWidgetMap.value(key)->reloadConfigurableData();
        }
        qde_paramint = qde_paramint.nextSiblingElement();
      }
      QDomNode qdn_parambools = qde_configurable.elementsByTagName("parambools").at(0);
      // By default there exists only one node named "parambools", meaning there is only one list ...
      QDomElement qde_parambool = qdn_parambools.firstChild().toElement();
      while (!qde_parambool.isNull()) {
        QString key = qde_parambool.attribute("name", "???");
        QString desc = qde_parambool.attribute("description");
        bool value = qde_parambool.attribute("value").toInt();
        if (config->getParamBoolMap().find(key.toStdString()) != config->getParamBoolMap().end() && configTileWidgetMap.contains(key)) {
          config->setParam(key.toStdString(), value);
          config->setParamDescr(key.toStdString(), desc.toStdString());
          configTileWidgetMap.value(key)->reloadConfigurableData();
        }
        qde_parambool = qde_parambool.nextSiblingElement();
      }
    } else
      return -5;
    return 0;
  }

  bool QConfigurableWidget::saveConfigurableState(const QString &fileName) {
    QDomDocument doc("ConfigurableStateTypeDefinition");
    // <ConfigurableStates>
    QDomElement nodeConfigurableStates = doc.createElement("ConfigurableStates");
    doc.appendChild(nodeConfigurableStates);
    nodeConfigurableStates.appendChild(toXml());
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
      return false;
    QTextStream ts(&file);
    ts << doc.toString();
    file.close();
    return true;
  }

  QDomElement QConfigurableWidget::toXml() {
    QDomDocument doc("ConfigurableStateTypeDefinition");
    // <ConfigurableState>

    QDomElement nodeConfigurableState = doc.createElement("ConfigurableState");
//    QDomElement nodeConfigurableState = new QDomElement();
//    nodeConfigurableState->setTagName("ConfigurableState");
    doc.appendChild(nodeConfigurableState);
    nodeConfigurableState.setAttribute("name", QString(config->getName().c_str()) + "_" + QString::number(nameIndex));

    // <ConfigurableState><ConfigurableWidget>
    QDomElement nodeConfigurableWidget = doc.createElement("ConfigurableWidget");
    nodeConfigurableState.appendChild(nodeConfigurableWidget);
    nodeConfigurableWidget.setAttribute("isCollapsed", isCollapsed ? "true" : "false");
    nodeConfigurableWidget.setAttribute("numberTilesPerLine", numberTilesPerLine);
    if (!configTileWidgetMap.isEmpty())
      nodeConfigurableWidget.setAttribute("tileWidgetWidth", configTileWidgetMap.values().at(0)->width());

    // <ConfigurableState><ConfigurableWidget><ConfigurableTileWidgets>
    QDomElement nodeConfigurableTileWidgets = doc.createElement("ConfigurableTileWidgets");
    nodeConfigurableWidget.appendChild(nodeConfigurableTileWidgets);
    // <ConfigurableState><ConfigurableWidget><ConfigurableTileWidgets><ConfigurableTileWidget name="..." tileIndex=xx isVisible={true|false} />
    foreach(QAbstractConfigurableTileWidget* configurableTile, configTileWidgetMap)
      {
        QDomElement nodeConfigurableTileWidget = doc.createElement("ConfigurableTileWidget");
        nodeConfigurableTileWidgets.appendChild(nodeConfigurableTileWidget);
        nodeConfigurableTileWidget.setAttribute("name", configurableTile->getName());
        nodeConfigurableTileWidget.setAttribute("tileIndex", configurableTile->getTileIndex());
        nodeConfigurableTileWidget.setAttribute("isVisible", configurableTile->isVisible()?"true":"false");
      }

    // <ConfigurableState><Configurable>
    QDomElement nodeConfigurable = doc.createElement("Configurable");
    nodeConfigurableState.appendChild(nodeConfigurable);
    nodeConfigurable.setAttribute("name", QString(config->getName().c_str()));
    nodeConfigurable.setAttribute("revision", QString(config->getRevision().c_str()));
    nodeConfigurable.setAttribute("id", config->getId());

    // <ConfigurableState><Configurable><paramvals>
    QDomElement nodeParamvals = doc.createElement("paramvals");
    nodeConfigurable.appendChild(nodeParamvals);
    // <ConfigurableState><Configurable><paramvals><paramval name="..." value=... minBound=... maxBound=... description="..." />
    int i = 0;
    foreach(Configurable::paramvalpair pair, config->getParamValMap())
      {
        Configurable::paramkey key = pair.first;
        QDomElement nodeParamval = doc.createElement("paramval");
        nodeParamvals.appendChild(nodeParamval);
        nodeParamval.setAttribute("name", QString(key.c_str()));
        nodeParamval.setAttribute("value", *(config->getParamValMap()[key]));
        nodeParamval.setAttribute("minBound", config->getParamvalBounds(key).first);
        nodeParamval.setAttribute("maxBound", config->getParamvalBounds(key).second);
        nodeParamval.setAttribute("description", QString(config->getParamDescr(key).c_str()));
        i++;
      }

    // <ConfigurableState><Configurable><paramints>
    QDomElement nodeParamints = doc.createElement("paramints");
    nodeConfigurable.appendChild(nodeParamints);
    // <ConfigurableState><Configurable><paramints><paramint name="..." value=... minBound=... maxBound=... description="..." />
    foreach(Configurable::paramintpair pair, config->getParamIntMap())
      {
        Configurable::paramkey key = pair.first;
        QDomElement nodeParamint = doc.createElement("paramint");
        nodeParamints.appendChild(nodeParamint);
        nodeParamint.setAttribute("name", QString(key.c_str()));
        nodeParamint.setAttribute("value", *(config->getParamIntMap()[key]));
        nodeParamint.setAttribute("minBound", config->getParamintBounds(key).first);
        nodeParamint.setAttribute("maxBound", config->getParamintBounds(key).second);
        nodeParamint.setAttribute("description", QString(config->getParamDescr(key).c_str()));
      }

    // <ConfigurableState><Configurable><parambools>
    QDomElement nodeParambools = doc.createElement("parambools");
    nodeConfigurable.appendChild(nodeParambools);
    // <ConfigurableState><Configurable><parambools><parambool name="..." value=... description="..." />
    foreach(Configurable::paramboolpair pair, config->getParamBoolMap())
      {
        Configurable::paramkey key = pair.first;
        QDomElement nodeParambool = doc.createElement("parambool");
        nodeParambools.appendChild(nodeParambool);
        nodeParambool.setAttribute("name", QString(key.c_str()));
        nodeParambool.setAttribute("value", *(config->getParamBoolMap()[key]));
        nodeParambool.setAttribute("description", QString(config->getParamDescr(key).c_str()));
      }

    return nodeConfigurableState;
  }

  void QConfigurableWidget::sl_showAndHideParameters() {
    QConfigurableTileShowHideDialog* dialog = new QConfigurableTileShowHideDialog(configTileWidgetMap, &layout);
    dialog->exec();
    delete (dialog);
    arrangeConfigurableTiles();
  }

  void QConfigurableWidget::arrangeConfigurableTiles() {
    foreach(QAbstractConfigurableTileWidget* configurableTile, configTileWidgetMap)
      {
        layout.removeWidget(configurableTile);
        layout.addWidget(configurableTile, configurableTile->getTileIndex() / numberTilesPerLine, configurableTile->getTileIndex() % numberTilesPerLine, Qt::AlignLeft);
      }
    layout.setColumnStretch(numberTilesPerLine+1, 100);
  }

  void QConfigurableWidget::setFolding(bool folding) {

    if (!folding) {
      foreach(QAbstractConfigurableTileWidget* configurableTile, configTiles_shownBeforeCollapse)
        {
          configurableTile->show();
        }
      arrangeConfigurableTiles();
      isCollapsed = false;
    } else {
      foreach(QAbstractConfigurableTileWidget* configurableTile, configTileWidgetMap)
        {
          if (configurableTile->isVisible()) {
            configTiles_shownBeforeCollapse.insert(configurableTile->getTileIndex(), configurableTile);
            configurableTile->hide();
          }
        }
      isCollapsed = true;
    }
  }

  void QConfigurableWidget::enterEvent(QEvent * event) {
    defaultPalette = palette();
    QPalette pal = QPalette(defaultPalette);
    pal.setColor(QPalette::AlternateBase, QColor(200, 200, 220));
    setPalette(pal);
    setBackgroundRole(QPalette::AlternateBase);
    setAutoFillBackground(true);
    update();
  }
  void QConfigurableWidget::leaveEvent(QEvent * event) {
    setPalette(defaultPalette);
    update();
  }
  void QConfigurableWidget::mousePressEvent(QMouseEvent * event) {
    foreach(QAbstractConfigurableTileWidget* configurableTile, configTileWidgetMap)
      {
        if (configurableTile->isVisible() && configurableTile->contains(event->pos())) {
          configurableTile_dragging = configurableTile;
          configruableTile_mousePressedOffset = event->pos() - configurableTile_dragging->pos();
          QMimeData *mimeData = new QMimeData;
          mimeData->setData("mimetype:" + QString::number(config->getId()), 0);

          QDrag *drag = new QDrag(this);
          drag->setMimeData(mimeData);
          drag->setPixmap(QPixmap::grabWidget(configurableTile_dragging));
          drag->setHotSpot(configruableTile_mousePressedOffset);
          configurableTile_dragging->toDummy(true);
          drag->exec(Qt::MoveAction);
        }
      }
  }
  void QConfigurableWidget::mouseDoubleClickEvent(QMouseEvent * event) {
    if (event->button() == Qt::LeftButton) {
      setFolding(!isCollapsed);
    }
  }

  void QConfigurableWidget::dragEnterEvent(QDragEnterEvent *event) {
    if (event->mimeData()->hasFormat("mimetype:" + QString::number(config->getId()))) {
      if (event->source() == this) {
        configurableTile_dragging->toDummy(true);
      } else {
        configurableTile_dragging = 0;
      }
      event->setDropAction(Qt::MoveAction);
      event->accept();
      return;
    }
    event->ignore();
  }

  void QConfigurableWidget::dragLeaveEvent(QDragLeaveEvent *event) {
    if (configurableTile_dragging != 0) {
      configurableTile_dragging->toDummy(false);
    }
    arrangeConfigurableTiles();
    event->ignore();
  }

  void QConfigurableWidget::dragMoveEvent(QDragMoveEvent *event) {
    if (event->mimeData()->hasFormat("mimetype:" + QString::number(config->getId()))) {
      event->setDropAction(Qt::MoveAction);
      event->accept();
      return;
    }
    event->ignore();
  }

  void QConfigurableWidget::dropEvent(QDropEvent *event) {
    if (event->mimeData()->hasFormat("mimetype:" + QString::number(config->getId()))) {
      event->accept();
      foreach(QAbstractConfigurableTileWidget* configurableTile, configTileWidgetMap)
        {
          if (configurableTile != configurableTile_dragging && configurableTile->contains(event->pos())) {

            // Ok, there is a ConfigurationTile under the mouseCursor
            int tmp_index = configurableTile->getTileIndex();
            if (tmp_index < configurableTile_dragging->getTileIndex()) {
              // die ConfigurableTiles müssen nach hinten verschoben werden
              // setze das abzulegende ConfigurableTile an diese Position, alle folgenden bis zur Lücke gehen einen Paltz weiter
              foreach(QAbstractConfigurableTileWidget* configurableTile_tmp, configTileWidgetMap)
                {
                  int i = configurableTile_tmp->getTileIndex();
                  if (i >= tmp_index && i < configurableTile_dragging->getTileIndex()) {
                    configurableTile_tmp->setTileIndex(i + 1);
                  }
                }

            } else {
              // setze das abzulegende ConfigurableTile an diese Position, alle vorhergehenden rücken auf
              foreach(QAbstractConfigurableTileWidget* configurableTile_tmp, configTileWidgetMap)
                {
                  int i = configurableTile_tmp->getTileIndex();
                  if (i > configurableTile_dragging->getTileIndex() && i <= tmp_index) {
                    configurableTile_tmp->setTileIndex(i - 1);
                  }
                }
            }
            configurableTile_dragging->setTileIndex(tmp_index);
            configurableTile_dragging->toDummy(false);
            arrangeConfigurableTiles();
            return;
          }
        }
      // There is no ConfigurationTile under the mouseCursor
      configurableTile_dragging->toDummy(false);
      arrangeConfigurableTiles();
      return;
    }
  }

} // namespace lpzrobots

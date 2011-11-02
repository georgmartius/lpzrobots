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
 *   Revision 1.24  2011-11-02 19:04:57  guettler
 *   crash fix: added removeCallback to destructor
 *
 *   Revision 1.23  2011/10/28 16:12:57  guettler
 *   smaller title font size
 *
 *   Revision 1.22  2011/03/22 16:38:13  guettler
 *   - adpaptions to enhanced configurable and inspectable interface:
 *   - qconfigurable is now restarted if initialization of agents is finished
 *
 *   Revision 1.21  2011/03/21 17:35:26  guettler
 *   - new autosave checkbox in context menu implemented and used
 *
 *   Revision 1.20  2011/02/11 12:12:11  guettler
 *   - UI: some seperators added
 *
 *   Revision 1.19  2011/01/28 12:15:37  guettler
 *   - restore of AutoSave File from a backup implemented
 *   - reset to original values, values AND bounds for Configurable implemented
 *   - reset to original values for tileWidgets implemented
 *
 *   Revision 1.18  2011/01/27 09:04:12  guettler
 *   - some preparations for checkbox in order to switch the autosave function
 *
 *   Revision 1.17  2011/01/24 18:40:48  guettler
 *   - autosave functionality now stores only values, bounds and descriptions of
 *   parameters if they differ from their original values
 *
 *   Revision 1.16  2011/01/05 13:28:45  guettler
 *   - bugfix: auto delete functionality of qt lead to SIGSEV by reason of wrong
 *     processing order of destructors from child (QAbstractConfigurableTileWidget) and
 *     parent (QConfigurableWidget) objects
 *
 *   Revision 1.15  2010/12/16 18:37:40  wrabe
 *   -added several tooltips
 *   -corrected sentences, notation, syntax for improved informational value
 *   -bugfix: if in collapsed mode, all tiles were stored as invisible
 *   -cosmetic ui changes
 *   -other minor things
 *
 *   Revision 1.14  2010/12/16 16:39:25  wrabe
 *   - drag&drop reworked: user can now drag a parameter to a any place
 *   - rearrangement of parameters now made only when user wants this
 *   - bugfixes
 *
 *   Revision 1.13  2010/12/15 18:28:34  wrabe
 *   -preparations for drag&drop of tileWidgets to empty places
 *
 *   Revision 1.12  2010/12/15 18:06:55  wrabe
 *   -regression fix: drag and drop of tileWidgets
 *
 *   Revision 1.11  2010/12/15 17:26:28  wrabe
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

using namespace std;

namespace lpzrobots {
  
  QConfigurableWidget::QConfigurableWidget(Configurable* config, int nameIndex) :
    config(config), dragging(false), isCollapsed(false), configurableTile_dragging(0), nameIndex(nameIndex),
        numberOfTilesPerRow(3), numberOfVisibleTiles(0) {
    initBody();
    createConfigurableLines();
    setAcceptDrops(true);
    setToolTip();
    config->addCallbackable(this, Configurable::CALLBACK_CONFIGURABLE_CHANGED);
  }

  QConfigurableWidget::~QConfigurableWidget() {
    foreach(QAbstractConfigurableTileWidget* tileWidget, configTileWidgetMap)
      {
        disconnect(tileWidget, SIGNAL(sig_resize(QSize)));
        disconnect(this, SIGNAL(sig_tileWidgetResize(QSize)));
        disconnect(tileWidget, SIGNAL(sig_mousePressEvent(QMouseEvent*)));
        delete tileWidget;
      }
    config->removeCallbackable(this, Configurable::CALLBACK_CONFIGURABLE_CHANGED);
  }

  void QConfigurableWidget::createConfigurableLines() {

    int tileIndex = 0;

    Configurable::parammap valMap = config->getParamValMap();
    FOREACHC(Configurable::parammap, valMap, keyIt) {
      Configurable::paramkey key = (*keyIt).first;
      QAbstractConfigurableTileWidget* configTileWidget = new QValConfigurableTileWidget(config, key,
          tileIndexConfigWidgetMap);
      configTileWidgetMap.insert(configTileWidget->getConfigurableName(), configTileWidget);
      configTileWidget->setGridPos(tileIndex / numberOfTilesPerRow, tileIndex % numberOfTilesPerRow);
      connect(configTileWidget, SIGNAL(sig_resize(QSize)), this, SIGNAL(sig_tileWidgetResize(QSize)));
      connect(this, SIGNAL(sig_tileWidgetResize(QSize)), configTileWidget, SLOT(sl_resize(QSize)));
      connect(configTileWidget, SIGNAL(sig_mousePressEvent(QMouseEvent*)), this, SLOT(sl_mousePressEvent(QMouseEvent*)));
      //layout.addWidget(configTileWidget, numberWidgets / numberOfTilesPerRow, numberWidgets % numberOfTilesPerRow);
      tileIndex++;
    }

    Configurable::paramintmap intMap = config->getParamIntMap();
    FOREACHC(Configurable::paramintmap, intMap, keyIt) {
      Configurable::paramkey key = (*keyIt).first;
      QAbstractConfigurableTileWidget* configTileWidget = new QIntConfigurableTileWidget(config, key,
          tileIndexConfigWidgetMap);
      configTileWidgetMap.insert(configTileWidget->getConfigurableName(), configTileWidget);
      configTileWidget->setGridPos(tileIndex / numberOfTilesPerRow, tileIndex % numberOfTilesPerRow);
      connect(configTileWidget, SIGNAL(sig_resize(QSize)), this, SIGNAL(sig_tileWidgetResize(QSize)));
      connect(this, SIGNAL(sig_tileWidgetResize(QSize)), configTileWidget, SLOT(sl_resize(QSize)));
      connect(configTileWidget, SIGNAL(sig_mousePressEvent(QMouseEvent*)), this, SLOT(sl_mousePressEvent(QMouseEvent*)));
      //layout.addWidget(configTileWidget, numberWidgets / 3, numberWidgets % 3);
      tileIndex++;
    }
    Configurable::paramboolmap boolMap = config->getParamBoolMap();
    FOREACHC(Configurable::paramboolmap, boolMap, keyIt) {
      Configurable::paramkey key = (*keyIt).first;
      QAbstractConfigurableTileWidget* configTileWidget = new QBoolConfigurableTileWidget(config, key,
          tileIndexConfigWidgetMap);
      configTileWidgetMap.insert(configTileWidget->getConfigurableName(), configTileWidget);
      configTileWidget->setGridPos(tileIndex / numberOfTilesPerRow, tileIndex % numberOfTilesPerRow);
      connect(configTileWidget, SIGNAL(sig_resize(QSize)), this, SIGNAL(sig_tileWidgetResize(QSize)));
      connect(this, SIGNAL(sig_tileWidgetResize(QSize)), configTileWidget, SLOT(sl_resize(QSize)));
      connect(configTileWidget, SIGNAL(sig_mousePressEvent(QMouseEvent*)), this, SLOT(sl_mousePressEvent(QMouseEvent*)));
      //layout.addWidget(configTileWidget, numberWidgets / 3, numberWidgets % 3);
      tileIndex++;
    }
    numberOfVisibleTiles = tileIndex;
    sl_rearrangeConfigurableTiles();
    //body.setLayout(&layout);
  }

  void QConfigurableWidget::initBody() {
    setLayout(&layout);

    // TODO: show checkbox to switch between autosave configurable values
    //setCheckable(true);
    connect(this, SIGNAL(toggled(bool)), this, SLOT(sl_toggled(bool)));

    //    setTitle(QString(config->getName().c_str()) + "  -  " + QString(config->getRevision().c_str()) + "  [" + QString::number(config->getId()) + "]");
    setTitle(QString(config->getName().c_str()) + " (" + QString::number(nameIndex) + ")");
    configName = QString(config->getName().c_str()) + "_" + QString::number(nameIndex);
    setFont(QFont("Courier", 10, QFont::Bold));

    QPalette pal = palette();
    pal.setColor(QPalette::AlternateBase, QColor(200, 210, 200));
    setPalette(pal);
    setBackgroundRole(QPalette::AlternateBase);
    setAutoFillBackground(true);

    // Prepare the context menu to show the configurable show and hide dialog
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(sl_execContextMenu(const QPoint &)));
    contextMenuShowHideDialog.addAction(tr("set number of parameter columns ..."), this,
        SLOT(sl_changeNumberTileColumns()));
    contextMenuShowHideDialog.addAction(tr("show/hide parameters ..."), this, SLOT(sl_showAndHideParameters()));
    contextMenuShowHideDialog.addAction(tr("rearrange parameters"), this, SLOT(sl_rearrangeConfigurableTiles()));
    contextMenuShowHideDialog.addSeparator();
    contextMenuShowHideDialog.addAction(tr("load configurable state from file ..."), this,
        SLOT(sl_loadConfigurableStateFromFile()));
    contextMenuShowHideDialog.addAction(tr("save current configurable state to file ..."), this,
        SLOT(sl_saveConfigurableStateToFile()));

    actionToggleAutoSave = new QAction(tr("enable autosave function for this Configurable"), this);
    actionToggleAutoSave->setCheckable(true);
    actionToggleAutoSave->setChecked(true); // default value
    contextMenuShowHideDialog.addAction(actionToggleAutoSave);

    contextMenuShowHideDialog.addSeparator();
    contextMenuShowHideDialog.addAction(tr("reset all parameters to original values"), this,
        SLOT(sl_resetToOriginalValues()));
    contextMenuShowHideDialog.addAction(tr("reset all parameters to original values AND bounds"), this,
        SLOT(sl_resetToOriginalValuesAndBounds()));
    layout.setColumnStretch(10, 100);
  }

  void QConfigurableWidget::sl_resetToOriginalValues() {
    foreach(QAbstractConfigurableTileWidget* configurableTile, configTileWidgetMap)
      {
        configurableTile->sl_resetToOriginalValues();
      }
  }

  void QConfigurableWidget::sl_resetToOriginalValuesAndBounds() {
    foreach(QAbstractConfigurableTileWidget* configurableTile, configTileWidgetMap)
      {
        configurableTile->sl_resetToOriginalValuesAndBounds();
      }
  }

  void QConfigurableWidget::sl_toggled(bool on) {
    foreach(QAbstractConfigurableTileWidget* configurableTile, configTileWidgetMap)
      {
        configurableTile->setEnabled(true);
      }
  }

  void QConfigurableWidget::sl_execContextMenu(const QPoint & pos) {
    if (!isCollapsed) {
      contextMenuShowHideDialog.exec(this->mapToGlobal(pos));
    }
  }

  void QConfigurableWidget::sl_changeNumberTileColumns() {
    int oldNumberOfTilesPerRow = numberOfTilesPerRow;
    QChangeNumberTileColumnsDialog* dialog = new QChangeNumberTileColumnsDialog(&numberOfTilesPerRow);
    dialog->exec();
    if (numberOfTilesPerRow < oldNumberOfTilesPerRow)
      sl_rearrangeConfigurableTiles();
    else
      arrangeConfigurableTiles();
  }

  void QConfigurableWidget::sl_loadConfigurableStateFromFile() {
    QFileDialog* fileDialog = new QFileDialog();
    fileDialog->setWindowTitle("Select the XML file containing the ConfigurableState(s)");
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
      QHash<QString, QDomElement> qde_configurableStateMap;
      QDomNodeList qdn_List = qde_configurableStates.elementsByTagName("ConfigurableState");
      for (int i = 0; i < qdn_List.size(); i++)
        qde_configurableStateMap.insert(qdn_List.at(i).toElement().attribute("name"), qdn_List.at(i).toElement());

      QMap<QString, QConfigurableWidget*> configurableWidgetMap;
      configurableWidgetMap.insert(getName(), this);

      QConfigurableLoadSaveDialog* dialog = new QConfigurableLoadSaveDialog(configurableWidgetMap,
          qde_configurableStateMap, QConfigurableLoadSaveDialog::ConfigurableLoadSingle);
      dialog->exec();
      //QMessageBox::warning(this, this->title(), tr("Configurable state could not be opened."), QMessageBox::Close);
    }
  }
  void QConfigurableWidget::sl_saveConfigurableStateToFile() {
    QFileDialog* fileDialog = new QFileDialog();
    fileDialog->setWindowTitle("Select the XML file to store the ConfigurableState");
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

    return fromXml(qde_configurableStates.elementsByTagName("ConfigurableState").at(0).toElement(), false);
  }

  int QConfigurableWidget::fromXml(const QDomElement &qde_configurableState, bool inAutoSaveMode) {
    // GUI-Elements...
    QDomElement qde_configurableWidget = qde_configurableState.firstChildElement("ConfigurableWidget");
    if (!qde_configurableWidget.isNull()) {
      // By default there exists only one node named "ConfigurableWidget"!
      QString collapse = qde_configurableWidget.attribute("isCollapsed", "true");
      numberOfTilesPerRow = qde_configurableWidget.attribute("numberOfTilesPerRow", "4").toInt();
      int tileWidgetWidth = qde_configurableWidget.attribute("tileWidgetWidth", QString::number(
          QAbstractConfigurableTileWidget::defaultWidgetSize.width())).toInt();
      QDomNode qdn_configurableTileWidgets = qde_configurableWidget.elementsByTagName("ConfigurableTileWidgets").at(0);
      // By default there will only one list of ConfigurableTileWidgets accepted!
      QDomElement qde_configurableTileWidget = qdn_configurableTileWidgets.firstChild().toElement();
      int tmpTileIndex = 0;
      numberOfVisibleTiles = 0;
      while (!qde_configurableTileWidget.isNull()) {
        QString tileName = qde_configurableTileWidget.attribute("name", "???");
        int gridColumn = qde_configurableTileWidget.attribute("gridColumn", QString::number(tmpTileIndex
            % numberOfTilesPerRow)).toInt();
        int gridRow = qde_configurableTileWidget.attribute("gridRow", QString::number(tmpTileIndex
            / numberOfTilesPerRow)).toInt();
        QString visible = qde_configurableTileWidget.attribute("isVisible", "true");
        QAbstractConfigurableTileWidget* tileWidget = configTileWidgetMap.value(tileName);
        if (tileWidget != 0) {
          tileWidget->setGridPos(gridRow, gridColumn);
          tileWidget->sl_resize(QSize(tileWidgetWidth, QAbstractConfigurableTileWidget::defaultWidgetSize.height()));
          if (visible.startsWith("true")) {
            numberOfVisibleTiles++;
            tileWidget->show();
          } else {
            tileWidget->hide();
          }
        }
        qde_configurableTileWidget = qde_configurableTileWidget.nextSiblingElement();
        tmpTileIndex++;
      }
      arrangeConfigurableTiles();
      setFolding(collapse.startsWith("true"));
    } else
      return -4;

    // data...
    QDomElement qde_configurable = qde_configurableState.firstChildElement("Configurable");
    if (!qde_configurable.isNull()) {
      // By default there exists only one node named "Configurable"!
      // this node has attributes named: revision, name, autosaveFunction
      // this node has childs named <paramvals>, <paramints> and <parambools>
      QString collapse = qde_configurableWidget.attribute("autosaveFunction", "true");
      actionToggleAutoSave->setChecked(collapse.startsWith("true"));
      QDomNode qdn_paramvals = qde_configurable.elementsByTagName("paramvals").at(0);
      // By default there exists only one node named "paramvals", meaning there is only one list ...
      QDomElement qde_paramval = qdn_paramvals.firstChild().toElement();
      while (!qde_paramval.isNull()) {
        QString key = qde_paramval.attribute("name", "???");
        string stdKey = key.toStdString();
        if (config->getParamValMap().find(stdKey) != config->getParamValMap().end()
            && configTileWidgetMap.contains(key)) {
          if ((inAutoSaveMode && actionToggleAutoSave->isChecked()) || !inAutoSaveMode) {
            QString desc = qde_paramval.attribute("description", QString(config->getParamDescr(stdKey).c_str()));
            double value = qde_paramval.attribute("value", QString::number(config->getParam(stdKey))).toDouble();
            double minBound =
                qde_paramval.attribute("minBound", QString::number(config->getParamvalBounds(stdKey).first)).toDouble();
            double maxBound = qde_paramval.attribute("maxBound",
                QString::number(config->getParamvalBounds(stdKey).second)).toDouble();
            config->setParamBounds(stdKey, minBound, maxBound);
            config->setParam(stdKey, value);
            config->setParamDescr(stdKey, desc.toStdString());
          }
          configTileWidgetMap.value(key)->reloadConfigurableData();
        }
        qde_paramval = qde_paramval.nextSiblingElement();
      }
      QDomNode qdn_paramints = qde_configurable.elementsByTagName("paramints").at(0);
      // By default there exists only one node named "paramints", meaning there is only one list ...
      QDomElement qde_paramint = qdn_paramints.firstChild().toElement();
      while (!qde_paramint.isNull()) {
        QString key = qde_paramint.attribute("name", "???");
        string stdKey = key.toStdString();
        if (config->getParamIntMap().find(stdKey) != config->getParamIntMap().end()
            && configTileWidgetMap.contains(key)) {
          if ((inAutoSaveMode && actionToggleAutoSave->isChecked()) || !inAutoSaveMode) {
            QString desc = qde_paramint.attribute("description", QString(config->getParamDescr(stdKey).c_str()));
            int value = qde_paramint.attribute("value", QString::number(config->getParam(stdKey))).toInt();
            int minBound =
                qde_paramint.attribute("minBound", QString::number(config->getParamintBounds(stdKey).first)).toInt();
            int maxBound =
                qde_paramint.attribute("maxBound", QString::number(config->getParamintBounds(stdKey).second)).toInt();
            config->setParamBounds(stdKey, minBound, maxBound);
            config->setParam(stdKey, value);
            config->setParamDescr(stdKey, desc.toStdString());
          }
          configTileWidgetMap.value(key)->reloadConfigurableData();
        }
        qde_paramint = qde_paramint.nextSiblingElement();
      }
      QDomNode qdn_parambools = qde_configurable.elementsByTagName("parambools").at(0);
      // By default there exists only one node named "parambools", meaning there is only one list ...
      QDomElement qde_parambool = qdn_parambools.firstChild().toElement();
      while (!qde_parambool.isNull()) {
        QString key = qde_parambool.attribute("name", "???");
        string stdKey = key.toStdString();
        if (config->getParamBoolMap().find(stdKey) != config->getParamBoolMap().end() && configTileWidgetMap.contains(
            key)) {
          if ((inAutoSaveMode && actionToggleAutoSave->isChecked()) || !inAutoSaveMode) {
            QString desc = qde_parambool.attribute("description", QString(config->getParamDescr(stdKey).c_str()));
            bool value = qde_parambool.attribute("value", QString::number(config->getParam(stdKey))).toInt();
            config->setParam(stdKey, value);
            config->setParamDescr(stdKey, desc.toStdString());
          }
          configTileWidgetMap.value(key)->reloadConfigurableData();
        }
        qde_parambool = qde_parambool.nextSiblingElement();
      }
    } else
      return -5;
    setToolTip();
    return 0;
  }

  bool QConfigurableWidget::saveConfigurableState(const QString &fileName) {
    QDomDocument doc("ConfigurableStateTypeDefinition");
    // <ConfigurableStates>
    QDomElement nodeConfigurableStates = doc.createElement("ConfigurableStates");
    doc.appendChild(nodeConfigurableStates);
    nodeConfigurableStates.appendChild(toXml(true, false));
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
      return false;
    QTextStream ts(&file);
    ts << doc.toString();
    file.close();
    return true;
  }

  QDomElement QConfigurableWidget::toXml(bool insertDefaultConfigurableValues, bool inAutoSaveMode) {
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
    nodeConfigurableWidget.setAttribute("numberOfTilesPerRow", numberOfTilesPerRow);
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
        nodeConfigurableTileWidget.setAttribute("gridRow", configurableTile->getGridPos().row());
        nodeConfigurableTileWidget.setAttribute("gridColumn", configurableTile->getGridPos().column());
        nodeConfigurableTileWidget.setAttribute("isVisible", configurableTile->isVisible() ? "true" : "false");
      }

    // <ConfigurableState><Configurable>
    QDomElement nodeConfigurable = doc.createElement("Configurable");
    nodeConfigurableState.appendChild(nodeConfigurable);
    nodeConfigurable.setAttribute("name", QString(config->getName().c_str()));
    nodeConfigurable.setAttribute("revision", QString(config->getRevision().c_str()));
    nodeConfigurable.setAttribute("id", config->getId());
    nodeConfigurable.setAttribute("autosaveFunction", actionToggleAutoSave->isChecked());

    if (inAutoSaveMode && !insertDefaultConfigurableValues) {
      QDomComment nodeComment;
      if (actionToggleAutoSave->isChecked()) {
        nodeComment
            = doc.createComment(
                "While in autosave mode, only values, bounds and descriptions of parameters which are differing from their original values are stored.");
      } else {
        nodeComment
            = doc.createComment(
                "While in autosave mode and autosave function is disabled, nothing except the widget part and the parameter names will be stored.");
      }
      nodeConfigurable.appendChild(nodeComment);
    }

    // <ConfigurableState><Configurable><paramvals>
    QDomElement nodeParamvals = doc.createElement("paramvals");
    nodeConfigurable.appendChild(nodeParamvals);
    // <ConfigurableState><Configurable><paramvals><paramval name="..." value=... minBound=... maxBound=... description="..." />
    int i = 0;
    foreach(Configurable::paramvalpair pair, config->getParamValMap())
      {
        Configurable::paramkey key = pair.first;
        QDomElement nodeParamval = doc.createElement("paramval");
        QValConfigurableTileWidget* configTile = static_cast<QValConfigurableTileWidget*> (configTileWidgetMap.value(
            QString(key.c_str())));
        nodeParamvals.appendChild(nodeParamval);
        nodeParamval.setAttribute("name", QString(key.c_str()));
        if ((inAutoSaveMode && actionToggleAutoSave->isChecked()) || !inAutoSaveMode) {
          if (insertDefaultConfigurableValues || configTile->valueChanged())
            nodeParamval.setAttribute("value", *(config->getParamValMap()[key]));
          if (insertDefaultConfigurableValues || configTile->boundsChanged()) {
            nodeParamval.setAttribute("minBound", config->getParamvalBounds(key).first);
            nodeParamval.setAttribute("maxBound", config->getParamvalBounds(key).second);
          }
          if (insertDefaultConfigurableValues || configTile->descriptionChanged())
            nodeParamval.setAttribute("description", QString(config->getParamDescr(key).c_str()));
        }
        i++;
      }

    // <ConfigurableState><Configurable><paramints>
    QDomElement nodeParamints = doc.createElement("paramints");
    nodeConfigurable.appendChild(nodeParamints);
    // <ConfigurableState><Configurable><paramints><paramint name="..." value=... minBound=... maxBound=... description="..." />
    foreach(Configurable::paramintpair pair, config->getParamIntMap())
      {
        Configurable::paramkey key = pair.first;
        QIntConfigurableTileWidget* configTile = static_cast<QIntConfigurableTileWidget*> (configTileWidgetMap.value(
            QString(key.c_str())));
        QDomElement nodeParamint = doc.createElement("paramint");
        nodeParamints.appendChild(nodeParamint);
        nodeParamint.setAttribute("name", QString(key.c_str()));
        if ((inAutoSaveMode && actionToggleAutoSave->isChecked()) || !inAutoSaveMode) {
          if (insertDefaultConfigurableValues || configTile->valueChanged())
            nodeParamint.setAttribute("value", *(config->getParamIntMap()[key]));
          if (insertDefaultConfigurableValues || configTile->boundsChanged()) {
            nodeParamint.setAttribute("minBound", config->getParamintBounds(key).first);
            nodeParamint.setAttribute("maxBound", config->getParamintBounds(key).second);
          }
          if (insertDefaultConfigurableValues || configTile->descriptionChanged())
            nodeParamint.setAttribute("description", QString(config->getParamDescr(key).c_str()));
        }
      }

    // <ConfigurableState><Configurable><parambools>
    QDomElement nodeParambools = doc.createElement("parambools");
    nodeConfigurable.appendChild(nodeParambools);
    // <ConfigurableState><Configurable><parambools><parambool name="..." value=... description="..." />
    foreach(Configurable::paramboolpair pair, config->getParamBoolMap())
      {
        Configurable::paramkey key = pair.first;
        QBoolConfigurableTileWidget* configTile = static_cast<QBoolConfigurableTileWidget*> (configTileWidgetMap.value(
            QString(key.c_str())));
        QDomElement nodeParambool = doc.createElement("parambool");
        nodeParambools.appendChild(nodeParambool);
        nodeParambool.setAttribute("name", QString(key.c_str()));
        if ((inAutoSaveMode && actionToggleAutoSave->isChecked()) || !inAutoSaveMode) {
          if (insertDefaultConfigurableValues || configTile->valueChanged())
            nodeParambool.setAttribute("value", *(config->getParamBoolMap()[key]));
          if (insertDefaultConfigurableValues || configTile->descriptionChanged())
            nodeParambool.setAttribute("description", QString(config->getParamDescr(key).c_str()));
        }
      }

    return nodeConfigurableState;
  }

  void QConfigurableWidget::sl_showAndHideParameters() {
    QConfigurableTileShowHideDialog* dialog = new QConfigurableTileShowHideDialog(configTileWidgetMap,
        tileIndexConfigWidgetMap, numberOfTilesPerRow);
    dialog->exec();
    numberOfVisibleTiles = dialog->getNumberOfVisibleTiles();
    delete (dialog);
    arrangeConfigurableTiles();
    setToolTip();
  }

  void QConfigurableWidget::arrangeConfigurableTiles() {
    foreach(QAbstractConfigurableTileWidget* configurableTile, configTileWidgetMap)
      {
        layout.removeWidget(configurableTile);
        layout.addWidget(configurableTile, configurableTile->getGridPos().row(),
            configurableTile->getGridPos().column(), Qt::AlignLeft);
      }
  }

  void QConfigurableWidget::sl_rearrangeConfigurableTiles() {
    int tileIndex = 0;
    foreach(QAbstractConfigurableTileWidget* configurableTile, configTileWidgetMap)
      {
        if (configurableTile->isVisible()) {
          layout.removeWidget(configurableTile);
          configurableTile->setGridPos(tileIndex / numberOfTilesPerRow, tileIndex % numberOfTilesPerRow);
          layout.addWidget(configurableTile, tileIndex / numberOfTilesPerRow, tileIndex % numberOfTilesPerRow,
              Qt::AlignLeft);
          tileIndex++;
        }
      }
  }

  void QConfigurableWidget::setFolding(bool folding) {
    if (folding) {
      foreach(QAbstractConfigurableTileWidget* configurableTile, configTileWidgetMap)
        {
          if (configurableTile->isVisible()) {
            configTiles_shownBeforeCollapse.insert(configurableTile->getGridPos(), configurableTile);
            configurableTile->setInCollapseMode(true);
          }
        }
      isCollapsed = true;
    } else {
      foreach(QAbstractConfigurableTileWidget* configurableTile, configTiles_shownBeforeCollapse)
        {
          configurableTile->setInCollapseMode(false);
        }
      isCollapsed = false;
      configTiles_shownBeforeCollapse.clear();
    }
    setToolTip();
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
    if (event->button() == Qt::LeftButton) {
      foreach(QAbstractConfigurableTileWidget* configurableTile, configTileWidgetMap)
        {
          if (configurableTile->isVisible() && configurableTile->contains(event->pos())) {
            configurableTile_dragging = configurableTile;
            configurableTile_mousePressedOffset = event->pos() - configurableTile_dragging->pos();
            QMimeData *mimeData = new QMimeData;
            mimeData->setData("mimetype:" + QString::number(config->getId()), 0);

            QDrag *drag = new QDrag(this);
            drag->setMimeData(mimeData);
            drag->setPixmap(QPixmap::grabWidget(configurableTile_dragging));
            drag->setHotSpot(configurableTile_mousePressedOffset);
            configurableTile_dragging->toDummy(true);
            drag->exec(Qt::MoveAction);
          }
        }
    }
  }

  void QConfigurableWidget::sl_mousePressEvent(QMouseEvent* event) {
    QMouseEvent* modEvent = new QMouseEvent(event->type(), mapFromGlobal(event->globalPos()), event->globalPos(),
        event->button(), event->buttons(), event->modifiers());
    mousePressEvent(modEvent);
  }

  void QConfigurableWidget::mouseDoubleClickEvent(QMouseEvent * event) {
    if (event->button() == Qt::LeftButton) {
      setFolding(!isCollapsed);
    }
  }

  void QConfigurableWidget::dragEnterEvent(QDragEnterEvent *event) {
    if (event->mimeData()->hasFormat("mimetype:" + QString::number(config->getId()))) {
      if (event->source() == this) {
        grabMouse(Qt::ClosedHandCursor);
        configurableTile_dragging->toDummy(true);
        QGridPos highestGridPos = tileIndexConfigWidgetMap.keys().last();
        for (int row = 0; row <= highestGridPos.row() + 1; row++) {
          for (int column = 0; column < numberOfTilesPerRow; column++) {
            if (!tileIndexConfigWidgetMap.contains(QGridPos(row, column))) {
              // fill with Dummy
              QDummyConfigurableTileWidget* dummy = new QDummyConfigurableTileWidget(config, tileIndexConfigWidgetMap);
              dummy->setGridPos(row, column);
              dummy->sl_resize(configTileWidgetMap.value(configTileWidgetMap.keys().first())->size());
              layout.addWidget(dummy, row, column, Qt::AlignLeft);
              dummy->setName("Dummy_" + QString::number(row) + "x" + QString::number(column));
              configTileWidgetMap.insert(dummy->getName(), dummy);
              dummyConfigTileList.append(dummy);
            }
          }
        }
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
            QGridPos tmp_pos = configurableTile->getGridPos();
            configurableTile->setGridPos(configurableTile_dragging->getGridPos());
            configurableTile_dragging->setGridPos(tmp_pos);
            break;
          }
        }
      // There is no ConfigurationTile under the mouseCursor
      foreach (QDummyConfigurableTileWidget* dummy, dummyConfigTileList)
        {
          dummy->hide();
          configTileWidgetMap.remove(dummy->getName());
          layout.removeWidget(dummy);
          delete dummy;
        }
      dummyConfigTileList.clear();
      arrangeConfigurableTiles();
      configurableTile_dragging->toDummy(false);
      releaseMouse();
      return;
    }
  }
  //  void QConfigurableWidget::dropEvent(QDropEvent *event) {
  //    if (event->mimeData()->hasFormat("mimetype:" + QString::number(config->getId()))) {
  //      event->accept();
  //      foreach(QAbstractConfigurableTileWidget* configurableTile, configTileWidgetMap)
  //        {
  //          if (configurableTile != configurableTile_dragging && configurableTile->contains(event->pos())) {
  //
  //            // Ok, there is a ConfigurationTile under the mouseCursor
  //            int tmp_index = configurableTile->getTileIndex();
  //            if (tmp_index < configurableTile_dragging->getTileIndex()) {
  //              // die ConfigurableTiles müssen nach hinten verschoben werden
  //              // setze das abzulegende ConfigurableTile an diese Position, alle folgenden bis zur Lücke gehen einen Paltz weiter
  //              foreach(QAbstractConfigurableTileWidget* configurableTile_tmp, configTileWidgetMap)
  //                {
  //                  int i = configurableTile_tmp->getTileIndex();
  //                  if (i >= tmp_index && i < configurableTile_dragging->getTileIndex()) {
  //                    configurableTile_tmp->setTileIndex(i + 1);
  //                  }
  //                }
  //
  //            } else {
  //              // setze das abzulegende ConfigurableTile an diese Position, alle vorhergehenden rücken auf
  //              foreach(QAbstractConfigurableTileWidget* configurableTile_tmp, configTileWidgetMap)
  //                {
  //                  int i = configurableTile_tmp->getTileIndex();
  //                  if (i > configurableTile_dragging->getTileIndex() && i <= tmp_index) {
  //                    configurableTile_tmp->setTileIndex(i - 1);
  //                  }
  //                }
  //            }
  //            configurableTile_dragging->setTileIndex(tmp_index);
  //            configurableTile_dragging->toDummy(false);
  //            arrangeConfigurableTiles();
  //            return;
  //          }
  //        }
  //      // There is no ConfigurationTile under the mouseCursor
  //      configurableTile_dragging->toDummy(false);
  //      arrangeConfigurableTiles();
  //      return;
  //    }
  //  }

  void QConfigurableWidget::setToolTip() {
    if (isCollapsed)
      QGroupBox::setToolTip("Configurable is folded.\n(double click to unfold)");
    else
      QGroupBox::setToolTip(QString::number(numberOfVisibleTiles) + " visible parameters\n" + QString::number(
          configTileWidgetMap.count() - numberOfVisibleTiles) + " hidden parameters\n(double click to fold)");
  }

  void sl_lampyris_noctiluca() {
    QMessageBox msgBox;
    msgBox.setText("lampyris noctiluca - lat. Leuchtkäfer.");
    msgBox.exec();
    return;
  }


  void QConfigurableWidget::doOnCallBack(BackCaller* source, BackCaller::CallbackableType type) {
    Configurable* conf = dynamic_cast<Configurable*>(source);
    if (conf && type==Configurable::CALLBACK_CONFIGURABLE_CHANGED) {

      // destroy QConfigurableWidget for this configurable and their configurable childs,
      // then recreate QConfigurableWidgets
    }
    emit sig_configurableChanged(this);
  }


} // namespace lpzrobots

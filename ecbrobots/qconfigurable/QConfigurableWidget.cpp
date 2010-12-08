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
 *   Revision 1.6  2010-12-08 17:52:57  wrabe
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

namespace lpzrobots {
  
  QConfigurableWidget::QConfigurableWidget(Configurable* config) :
    config(config), dragging(false), isCollapsed(false), configurableTile_dragging(0) {
    initBody();
    createConfigurableLines();
    setAcceptDrops(true);
  }

  QConfigurableWidget::~QConfigurableWidget() {
  }

  void QConfigurableWidget::createConfigurableLines() {

    layout.setColumnStretch(3, 100);

    setLayout(&layout);
    int numberWidgets = 0;

    Configurable::parammap valMap = config->getParamValMap();
    FOREACHC(Configurable::parammap, valMap, keyIt) {
      Configurable::paramkey key = (*keyIt).first;
      QAbstractConfigurableTileWidget* configTileWidget = new QValConfigurableTileWidget(config, key);
      configTileWidgetMap.insert(configTileWidget->getConfigurableName(), configTileWidget);
      configTileWidget->setTileIndex(numberWidgets);
      layout.addWidget(configTileWidget, numberWidgets / 3, numberWidgets % 3);
      numberWidgets++;
    }

    Configurable::paramintmap intMap = config->getParamIntMap();
    FOREACHC(Configurable::paramintmap, intMap, keyIt) {
      Configurable::paramkey key = (*keyIt).first;
      QAbstractConfigurableTileWidget* configTileWidget = new QIntConfigurableTileWidget(config, key);
      configTileWidgetMap.insert(configTileWidget->getConfigurableName(), configTileWidget);
      configTileWidget->setTileIndex(numberWidgets);
      layout.addWidget(configTileWidget, numberWidgets / 3, numberWidgets % 3);
      numberWidgets++;
    }
    Configurable::paramboolmap boolMap = config->getParamBoolMap();
    FOREACHC(Configurable::paramboolmap, boolMap, keyIt) {
      Configurable::paramkey key = (*keyIt).first;
      QAbstractConfigurableTileWidget* configTileWidget = new QBoolConfigurableTileWidget(config, key);
      configTileWidgetMap.insert(configTileWidget->getConfigurableName(), configTileWidget);
      configTileWidget->setTileIndex(numberWidgets);
      layout.addWidget(configTileWidget, numberWidgets / 3, numberWidgets % 3);
      numberWidgets++;
    }

    //body.setLayout(&layout);
  }

  void QConfigurableWidget::initBody() {
    setTitle(QString(config->getName().c_str()) + "  -  " + QString(config->getRevision().c_str()) + "  [" + QString::number(config->getId()) + "]");
    setFont(QFont("Courier", 11, QFont::Bold));

    QPalette pal = palette();
    pal.setColor(QPalette::AlternateBase, QColor(200, 210, 200));
    setPalette(pal);
    setBackgroundRole(QPalette::AlternateBase);
    setAutoFillBackground(true);

    // Prepare the context menu to show the configurable show and hide dialog
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(sl_execContextMenu(const QPoint &)));
    contextMenuShowHideDialog.addAction("show/hide parameters", this, SLOT(sl_showAndHideParameters()));
    contextMenuShowHideDialog.addAction("load configurable state from file ...", this, SLOT(sl_loadConfigurableStateToFile()));
    contextMenuShowHideDialog.addAction("save current configurable state to file ...", this, SLOT(sl_saveConfigurableStateToFile()));
  }

  void QConfigurableWidget::sl_execContextMenu(const QPoint & pos) {
    if (!isCollapsed) {
      contextMenuShowHideDialog.exec(this->mapToGlobal(pos));
    }
  }
  void QConfigurableWidget::sl_loadConfigurableStateToFile() {
    QFileDialog* fileDialog = new QFileDialog();
    fileDialog->setAcceptMode(QFileDialog::AcceptOpen);
    fileDialog->setFileMode(QFileDialog::ExistingFile);
    if (fileDialog->exec() == QDialog::Accepted) {
      QString fileName = fileDialog->selectedFiles().at(0);
      if (!loadConfigurableState(fileName)) {
        QMessageBox::warning(this, this->title(), tr("Configurable state could not be opened."), QMessageBox::Close);
      }
    }
  }
  void QConfigurableWidget::sl_saveConfigurableStateToFile() {
    QFileDialog* fileDialog = new QFileDialog();
    fileDialog->setAcceptMode(QFileDialog::AcceptSave);
    fileDialog->setFileMode(QFileDialog::AnyFile);
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

    QDomElement qde_configurableState = doc.documentElement();
    if (qde_configurableState.tagName() != "ConfigurableState")
      return -3;

    // GUI-Elements...
    QDomElement qde_configurableWidget = qde_configurableState.firstChildElement("ConfigurableWidget");
    if (!qde_configurableWidget.isNull()) {
      // By default there exists only one node named "ConfigurableWidget"!
      bool collapse = qde_configurableWidget.attribute("isCollapsed", "0").toInt();
      setFolding(collapse);
      QDomNode qdn_configurableTileWidgets = qde_configurableWidget.elementsByTagName("ConfigurableTileWidgets").at(0);
      // By default there will only one list of ConfigurableTileWidgets accepted!
      QDomElement qde_configurableTileWidget = qdn_configurableTileWidgets.firstChild().toElement();
      int tmpTileIndex = 0;
      while (!qde_configurableTileWidget.isNull()) {
        QString tileName = qde_configurableTileWidget.attribute("name", "???");
        int tileIndex = qde_configurableTileWidget.attribute("tileIndex", QString::number(tmpTileIndex++)).toInt();
        bool visible = qde_configurableTileWidget.attribute("isVisible", "0").toInt();
        QAbstractConfigurableTileWidget* tileWidget = configTileWidgetMap[tileName];
        if (tileWidget != 0) {
          tileWidget->setTileIndex(tileIndex);
          tileWidget->setVisible(visible);
        }
        qde_configurableTileWidget = qde_configurableTileWidget.nextSiblingElement();
      }
      arrangeConfigurableTiles();
    }

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
        if (config->setParam(key.toStdString(), value)) {
          config->setParamBounds(key.toStdString(), minBound, maxBound);
          config->setParamDescr(key.toStdString(), desc.toStdString());
          configTileWidgetMap[key]->reloadConfigurableData();
        }
        qde_paramval = qde_paramval.nextSiblingElement();
      }
      QDomNode qdn_paramints = qde_configurable.elementsByTagName("paramints").at(0);
      // By default there exists only one node named "paramints", meaning there is only one list ...
      QDomElement qde_paramint = qdn_paramints.firstChild().toElement();
      while (!qde_paramint.isNull()) {
        QString key = qde_paramint.attribute("name", "???");
        QString desc = qde_paramval.attribute("description");
        int value = qde_paramval.attribute("value").toDouble();
        int minBound = qde_paramval.attribute("minBound").toDouble();
        int maxBound = qde_paramval.attribute("maxBound").toDouble();
        if (config->setParam(key.toStdString(), value)) {
          config->setParamBounds(key.toStdString(), minBound, maxBound);
          config->setParamDescr(key.toStdString(), desc.toStdString());
          configTileWidgetMap[key]->reloadConfigurableData();
        }
        qde_paramint = qde_paramint.nextSiblingElement();
      }
      QDomNode qdn_parambools = qde_configurable.elementsByTagName("parambools").at(0);
      // By default there exists only one node named "parambools", meaning there is only one list ...
      QDomElement qde_parambool = qdn_parambools.firstChild().toElement();
      while (!qde_parambool.isNull()) {
        QString key = qde_parambool.attribute("name", "???");
        QString desc = qde_paramval.attribute("description");
        bool value = qde_paramval.attribute("value").toDouble();
        if (config->setParam(key.toStdString(), value)) {
          config->setParamDescr(key.toStdString(), desc.toStdString());
        }
        qde_parambool = qde_parambool.nextSiblingElement();
      }
    }
    return -4;
  }

  bool QConfigurableWidget::saveConfigurableState(const QString &fileName) {
    QDomDocument doc("ConfigurableStateTypeDefinition");
    // <ConfigurableState>
    QDomElement nodeConfigurableState = doc.createElement("ConfigurableState");
    doc.appendChild(nodeConfigurableState);

    // <ConfigurableState><ConfigurableWidget>
    QDomElement nodeConfigurableWidget = doc.createElement("ConfigurableWidget");
    nodeConfigurableState.appendChild(nodeConfigurableWidget);
    nodeConfigurableWidget.setAttribute("isCollapsed", isCollapsed);

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
        nodeConfigurableTileWidget.setAttribute("isVisible", configurableTile->isVisible());
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

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
      return false;
    QTextStream ts(&file);
    ts << doc.toString();
    file.close();
    return true;
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
        layout.addWidget(configurableTile, configurableTile->getTileIndex() / 3, configurableTile->getTileIndex() % 3);
      }
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

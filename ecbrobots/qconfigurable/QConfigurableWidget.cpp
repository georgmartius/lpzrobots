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
 *   Revision 1.5  2010-12-03 11:11:53  wrabe
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
#include <QEvent>
#include <QMouseEvent>
#include <QMenu>
#include <QMessageBox>
#include <QPalette>
#include <QDialog>
#include <QScrollArea>
#include <QScrollBar>
#include <QList>
#include <QMap>
#include <QPixmap>
#include <QRect>

#include "QBoolConfigurableTileWidget.h"
#include "QIntConfigurableTileWidget.h"
#include "QValConfigurableTileWidget.h"
#include "QConfigurableTileShowHideDialog.h"

namespace lpzrobots {
  
  QConfigurableWidget::QConfigurableWidget(Configurable* config) :
    config(config), dragging(false), configurableTile_dragging(0) {
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
      configLineWidgetMap.insert(configTileWidget->getConfigurableName(), configTileWidget);
      configTileWidget->setTileIndex(numberWidgets);
      layout.addWidget(configTileWidget, numberWidgets / 3, numberWidgets % 3);
      numberWidgets++;
    }

    Configurable::paramintmap intMap = config->getParamIntMap();
    FOREACHC(Configurable::paramintmap, intMap, keyIt) {
      Configurable::paramkey key = (*keyIt).first;
      QAbstractConfigurableTileWidget* configTileWidget = new QIntConfigurableTileWidget(config, key);
      configLineWidgetMap.insert(configTileWidget->getConfigurableName(), configTileWidget);
      configTileWidget->setTileIndex(numberWidgets);
      layout.addWidget(configTileWidget, numberWidgets / 3, numberWidgets % 3);
      numberWidgets++;
    }
    Configurable::paramboolmap boolMap = config->getParamBoolMap();
    FOREACHC(Configurable::paramboolmap, boolMap, keyIt) {
      Configurable::paramkey key = (*keyIt).first;
      QAbstractConfigurableTileWidget* configTileWidget = new QBoolConfigurableTileWidget(config, key);
      configLineWidgetMap.insert(configTileWidget->getConfigurableName(), configTileWidget);
      configTileWidget->setTileIndex(numberWidgets);
      layout.addWidget(configTileWidget, numberWidgets / 3, numberWidgets % 3);
      numberWidgets++;
    }

    //body.setLayout(&layout);
  }

  void QConfigurableWidget::initBody() {
    setTitle(QString(config->getName().c_str()) + "  -  " + QString(config->getRevision().c_str()) + "  [" + QString::number(
        config->getId()) + "]");
    setFont(QFont("Courier", 11, QFont::Bold));

    QPalette pal = palette();
    pal.setColor(QPalette::AlternateBase, QColor(200, 210, 200));
    setPalette(pal);
    setBackgroundRole(QPalette::AlternateBase);
    setAutoFillBackground(true);

    // Prepare the context menu to show the configurable show and hide dialog
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(sl_execContextMenu(const QPoint &)));
    contextMenuShowHideDialog.addAction("show/hide configurables", this, SLOT(sl_showAndHideConfigurables()));
  }

  void QConfigurableWidget::sl_execContextMenu(const QPoint & pos) {
    contextMenuShowHideDialog.exec(this->mapToGlobal(pos));
  }

  void QConfigurableWidget::sl_showAndHideConfigurables() {
    QConfigurableTileShowHideDialog* dialog = new QConfigurableTileShowHideDialog(configLineWidgetMap, &layout);
    dialog->exec();
    delete (dialog);
    arrangeConfigurableTiles();
  }

  void QConfigurableWidget::arrangeConfigurableTiles() {
    foreach(QAbstractConfigurableTileWidget* configurableTile, configLineWidgetMap)
      {
        layout.removeWidget(configurableTile);
        layout.addWidget(configurableTile, configurableTile->getTileIndex() / 3, configurableTile->getTileIndex() % 3);
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
    foreach(QAbstractConfigurableTileWidget* configurableTile, configLineWidgetMap)
      {
        if (configurableTile->contains(event->pos())){
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
  void QConfigurableWidget::dragEnterEvent(QDragEnterEvent *event) {
    if (event->mimeData()->hasFormat("mimetype:" + QString::number(config->getId()))){
      if (event->source() == this){
        configurableTile_dragging->toDummy(true);
      }else{
        configurableTile_dragging = 0;
      }
      event->setDropAction(Qt::MoveAction);
      event->accept();
      return;
    }
    event->ignore();
  }

  void QConfigurableWidget::dragLeaveEvent(QDragLeaveEvent *event) {
    if (configurableTile_dragging != 0){
      configurableTile_dragging->toDummy(false);
    }
    arrangeConfigurableTiles();
    event->ignore();
  }

  void QConfigurableWidget::dragMoveEvent(QDragMoveEvent *event) {
    if (event->mimeData()->hasFormat("mimetype:" + QString::number(config->getId()))){
      event->setDropAction(Qt::MoveAction);
      event->accept();
      return;
    }
    event->ignore();
  }

  void QConfigurableWidget::dropEvent(QDropEvent *event) {
    if (event->mimeData()->hasFormat("mimetype:" + QString::number(config->getId()))){
      event->accept();
      foreach(QAbstractConfigurableTileWidget* configurableTile, configLineWidgetMap)
        {
          if (configurableTile != configurableTile_dragging && configurableTile->contains(event->pos())){

            // Ok, there is a ConfigurationTile under the mouseCursor
            int tmp_index = configurableTile->getTileIndex();
            if (tmp_index < configurableTile_dragging->getTileIndex()){
              // die ConfigurableTiles müssen nach hinten verschoben werden
              // setze das abzulegende ConfigurableTile an diese Position, alle folgenden bis zur Lücke gehen einen Paltz weiter
              foreach(QAbstractConfigurableTileWidget* configurableTile_tmp, configLineWidgetMap)
                {
                  int i = configurableTile_tmp->getTileIndex();
                  if (i >= tmp_index && i < configurableTile_dragging->getTileIndex()){
                    configurableTile_tmp->setTileIndex(i + 1);
                  }
                }

            }else{
              // setze das abzulegende ConfigurableTile an diese Position, alle vorhergehenden rücken auf
              foreach(QAbstractConfigurableTileWidget* configurableTile_tmp, configLineWidgetMap)
                {
                  int i = configurableTile_tmp->getTileIndex();
                  if (i > configurableTile_dragging->getTileIndex() && i <= tmp_index){
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

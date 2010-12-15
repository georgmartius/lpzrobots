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
 *   Revision 1.7  2010-12-15 17:26:28  wrabe
 *   - number of colums for tileWidgets and width of tileWidgets can
 *   now be changed (independently for each Configurable)
 *   - bugfixes
 *
 *   Revision 1.6  2010/12/15 11:24:39  guettler
 *   -new QDummyConfigurableTileWidget
 *
 *   Revision 1.5  2010/12/14 10:10:12  guettler
 *   -autoload/autosave now uses only one xml file
 *   -fixed getName of TileWidget which produced invisible widgets in xml files
 *
 *   Revision 1.4  2010/12/13 16:22:18  wrabe
 *   - autosave function rearranged
 *   - bugfixes
 *
 *   Revision 1.3  2010/12/08 17:52:57  wrabe
 *   - bugfixing/introducing new feature:
 *   - folding of the ConfigurableWidgets now awailable
 *   - highlight the ConfigurableTile when hoovered by mouse
 *   - load/store of the state of a ConfigurableWidget to file
 *
 *   Revision 1.2  2010/12/06 14:08:57  guettler
 *   - bugfixes
 *   - number of decimals is now calculated
 *
 *   Revision 1.1  2010/12/03 11:11:41  wrabe
 *   - replace of the ConfigurableLineWidgets by ConfigurableTileWidgets
 *   - (final rename from lines to tiles)
 *   - for history look at the ConfigurableLineWidget-classes
 *   - now handled paramVal, paramInt and paramBool, all the params are displayed
 *     as ConfigurableTiles witch can be show and hide seperatly or arranged by user
 *     (showHideDialog reacheble by contextMenu (right click an the Widget containing
 *     the tiles ), arrange the Tiles is can done by drag and drop (there is no history or
 *     storage implementet yet))
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

#include "QAbstractConfigurableTileWidget.h"

#include "selforg/configurable.h"
#include <QGridLayout>
#include <QMessageBox>
#include <QLabel>
#include <QPoint>

namespace lpzrobots {
  
  QSize QAbstractConfigurableTileWidget::defaultWidgetSize = QSize(300, 80);

  QAbstractConfigurableTileWidget::QAbstractConfigurableTileWidget(Configurable* config, Configurable::paramkey key) :
    config(config), key(key), tileIndex(0), internalVisible(true), enableResizing(false), isResizing(false) {
    defaultPalette = palette();
    setFixedSize(defaultWidgetSize);
    setFrameStyle(QFrame::StyledPanel | QFrame::Raised);
    setAttribute(Qt::WA_DeleteOnClose);
    setMouseTracking(true);
  }
  
  QAbstractConfigurableTileWidget::~QAbstractConfigurableTileWidget() {
  }

  QString QAbstractConfigurableTileWidget::getConfigurableName() {
    return QString(key.c_str());
  }

  bool QAbstractConfigurableTileWidget::contains(QPoint pos) {
    if ((x() <= pos.x()) && (y() <= pos.y()) && (pos.x() < (x() + width())) && (pos.y() < (y() + height())))
      return true;
    return false;
  }

  void QAbstractConfigurableTileWidget::enterEvent(QEvent * event) {

    QPalette pal = QPalette(defaultPalette);
    pal.setColor(QPalette::Window, QColor(220, 200, 200));
    setPalette(pal);
    setBackgroundRole(QPalette::Window);
    setAutoFillBackground(true);
    update();
  }

  void QAbstractConfigurableTileWidget::leaveEvent(QEvent * event) {
    setPalette(defaultPalette);
    update();
  }

  void QAbstractConfigurableTileWidget::setVisible(bool visible) {
    internalVisible = visible;
    QFrame::setVisible(visible);
  }

  void QAbstractConfigurableTileWidget::mouseMoveEvent(QMouseEvent * event) {
    QPoint p = event->pos();
    if (isResizing) {
      sl_resize(QSize(event->pos().x(), defaultWidgetSize.height()));
    } else if (width() - 3 <= p.x() && p.x() <= width() + 3) {
      grabMouse(Qt::SizeHorCursor);
      enableResizing = true;
    } else {
      releaseMouse();
      enableResizing = false;
    }
  }

  void QAbstractConfigurableTileWidget::mousePressEvent(QMouseEvent * event) {
    if (enableResizing && event->button() == Qt::LeftButton) {
      isResizing = true;
    }
  }

  void QAbstractConfigurableTileWidget::mouseReleaseEvent(QMouseEvent * event) {
    if (isResizing) {
      sl_resize(QSize(event->pos().x(), defaultWidgetSize.height()));
      isResizing = false;
      emit sig_resize(QSize(event->pos().x(), defaultWidgetSize.height()));
    }
  }

  void QAbstractConfigurableTileWidget::sl_resize(QSize newSize) {
    if (newSize.width()<130)
      setFixedSize(130, newSize.height());
    else
      setFixedSize(newSize);
  }



}

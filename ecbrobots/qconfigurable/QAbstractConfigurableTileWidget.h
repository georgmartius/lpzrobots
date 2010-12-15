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
 *   Revision 1.6  2010/12/15 11:24:40  guettler
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
 *   Revision 1.3  2010/12/09 17:00:08  wrabe
 *   - load / save function of ConfigurableState (configurable + GUI)
 *   - autoload / autosave function of ConfigurableState (configurable
 *     + GUI)
 *   - handling of equal Configurable names implemented for autoload
 *     and -save
 *   - bugfixing
 *
 *   Revision 1.2  2010/12/08 17:52:57  wrabe
 *   - bugfixing/introducing new feature:
 *   - folding of the ConfigurableWidgets now awailable
 *   - highlight the ConfigurableTile when hoovered by mouse
 *   - load/store of the state of a ConfigurableWidget to file
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

#ifndef __QCONFIGURABLETILEWIDGET_H_
#define __QCONFIGURABLETILEWIDGET_H_

#include "selforg/configurable.h"
#include <QWidget>
#include <QGridLayout>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QFrame>
#include <QLabel>
#include <QPalette>
#include <QMoveEvent>

namespace lpzrobots {
  
  class QAbstractConfigurableTileWidget : public QFrame {

    Q_OBJECT

    public:
      QAbstractConfigurableTileWidget(Configurable* config, Configurable::paramkey key);
      virtual ~QAbstractConfigurableTileWidget();
      virtual void setName(QString name) = 0;
      virtual QString getName() {
        return QString(key.c_str());
      }
      virtual void toDummy(bool set) = 0;
      virtual bool contains(QPoint pos);
      virtual void reloadConfigurableData() = 0;

      virtual void setTileIndex(int index) {
        tileIndex = index;
      }
      virtual int getTileIndex() {
        return tileIndex;
      }

      virtual QString getConfigurableName();
      static QSize defaultWidgetSize;
      virtual void setVisible(bool visible);
      virtual bool isVisible() { return internalVisible; }

      signals:
        void sig_resize(QSize newSize);

      public slots:
        virtual void sl_resize(QSize newSize);

    protected:
      virtual void enterEvent(QEvent * event);
      virtual void leaveEvent(QEvent * event);
      virtual void mouseMoveEvent(QMouseEvent * event);
      virtual void mousePressEvent(QMouseEvent * event);
      virtual void mouseReleaseEvent(QMouseEvent * event);


      QPalette defaultPalette;
      Configurable* config;
      Configurable::paramkey key;
      int tileIndex;
      bool internalVisible;
      bool enableResizing;
      bool isResizing;

  };

}

#endif /* __QCONFIGURABLETILEWIDGET_H_ */

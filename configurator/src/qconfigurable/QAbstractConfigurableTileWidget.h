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
 *   Revision 1.1  2011-07-11 16:06:01  guettler
 *   - access to Configurator is now provided by ConfiguratorProxy
 *   - creating static lib instead of dynamic variant
 *   - establish correct directory structure for including configurator into other non-qt projects
 *
 *   Revision 1.1  2011/07/01 12:32:15  guettler
 *   - pull out qconfigurable part of ecb_robots to get stand the alone library libconfigurator
 *
 *   Revision 1.13  2011/03/21 17:33:56  guettler
 *   - color changes now if parameter value or bounds is changed
 *   - adapted to enhanced configurable interface
 *
 *   Revision 1.12  2011/01/28 12:15:37  guettler
 *   - restore of AutoSave File from a backup implemented
 *   - reset to original values, values AND bounds for Configurable implemented
 *   - reset to original values for tileWidgets implemented
 *
 *   Revision 1.11  2011/01/24 18:40:48  guettler
 *   - autosave functionality now stores only values, bounds and descriptions of
 *   parameters if they differ from their original values
 *
 *   Revision 1.10  2010/12/16 18:37:40  wrabe
 *   -added several tooltips
 *   -corrected sentences, notation, syntax for improved informational value
 *   -bugfix: if in collapsed mode, all tiles were stored as invisible
 *   -cosmetic ui changes
 *   -other minor things
 *
 *   Revision 1.9  2010/12/16 16:39:25  wrabe
 *   - drag&drop reworked: user can now drag a parameter to a any place
 *   - rearrangement of parameters now made only when user wants this
 *   - bugfixes
 *
 *   Revision 1.8  2010/12/15 18:06:55  wrabe
 *   -regression fix: drag and drop of tileWidgets
 *
 *   Revision 1.7  2010/12/15 17:26:28  wrabe
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
#include "QGridPos.h"

namespace lpzrobots {
  
  class QAbstractConfigurableTileWidget : public QFrame {

    Q_OBJECT

    public:
      QAbstractConfigurableTileWidget(Configurable* config, Configurable::paramkey key, QMap<QGridPos, QAbstractConfigurableTileWidget*>& tileIndexConfigWidgetMap);
      virtual ~QAbstractConfigurableTileWidget();
      virtual void setName(QString name) = 0;
      virtual QString getName() {
        return QString(key.c_str());
      }
      virtual void toDummy(bool set) = 0;
      virtual bool contains(QPoint pos);
      virtual void reloadConfigurableData() = 0;

      virtual void setGridPos(int row, int column) {
        setGridPos(QGridPos(row,column));
      }

      virtual void setGridPos(QGridPos newGridPos) {
        if (tileIndexConfigWidgetMap.value(gridPos) == this)
          tileIndexConfigWidgetMap.remove(gridPos);
        gridPos = newGridPos;
        tileIndexConfigWidgetMap.insert(gridPos, this);
      }

      virtual QGridPos getGridPos() {
        return gridPos;
      }

      virtual QString getConfigurableName();
      static QSize defaultWidgetSize;
      virtual void setVisible(bool visible);
      virtual bool isVisible();
      virtual void setInCollapseMode(bool inCollapseMode);

      inline bool descriptionChanged() {
        return (config->getParamDescr(key) != origDescription);
      }
      virtual bool valueChanged() = 0;
      virtual bool boundsChanged() = 0;


      signals:
        void sig_resize(QSize newSize);
        void sig_mousePressEvent(QMouseEvent* event);

      public slots:
        virtual void sl_resize(QSize newSize);
        virtual void sl_resetToOriginalValues() = 0;
        virtual void sl_resetToOriginalValuesAndBounds() = 0;


    protected:
      virtual void enterEvent(QEvent * event);
      virtual void leaveEvent(QEvent * event);
      virtual void mouseMoveEvent(QMouseEvent * event);
      virtual void mousePressEvent(QMouseEvent * event);
      virtual void mouseReleaseEvent(QMouseEvent * event);
      virtual void updatePaletteChanged();


      QPalette actualPalette;
      QPalette defaultPalette;
      Configurable* config;
      Configurable::paramkey key;
      Configurable::paramdescr origDescription;
      QGridPos gridPos;
      bool internalVisible;
      bool enableResizing;
      bool isResizing;
      QMap<QGridPos, QAbstractConfigurableTileWidget*>& tileIndexConfigWidgetMap;
      bool entered;
  };

}

#endif /* __QCONFIGURABLETILEWIDGET_H_ */

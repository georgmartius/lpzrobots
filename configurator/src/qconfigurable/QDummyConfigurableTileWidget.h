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
 *   Revision 1.1  2011/07/01 12:32:16  guettler
 *   - pull out qconfigurable part of ecb_robots to get stand the alone library libconfigurator
 *
 *   Revision 1.5  2011/03/21 17:34:28  guettler
 *   - color changes now if parameter value or bounds is changed
 *   - adapted to enhanced configurable interface
 *
 *   Revision 1.4  2011/01/28 12:15:37  guettler
 *   - restore of AutoSave File from a backup implemented
 *   - reset to original values, values AND bounds for Configurable implemented
 *   - reset to original values for tileWidgets implemented
 *
 *   Revision 1.3  2010/12/16 16:39:25  wrabe
 *   - drag&drop reworked: user can now drag a parameter to a any place
 *   - rearrangement of parameters now made only when user wants this
 *   - bugfixes
 *
 *   Revision 1.2  2010/12/15 18:28:34  wrabe
 *   -preparations for drag&drop of tileWidgets to empty places
 *
 *   Revision 1.1  2010/12/15 11:24:39  guettler
 *   -new QDummyConfigurableTileWidget
 *
 *                                                                         *
 ***************************************************************************/

#ifndef __QDUMMYCONFIGURABLETILEWIDGET_H_
#define __QDUMMYCONFIGURABLETILEWIDGET_H_

#include "QAbstractConfigurableTileWidget.h"

namespace lpzrobots {
  
  class QDummyConfigurableTileWidget : public lpzrobots::QAbstractConfigurableTileWidget {
    public:
      QDummyConfigurableTileWidget(Configurable* config, QMap<QGridPos, QAbstractConfigurableTileWidget*>& tileIndexConfigWidgetMap);
      virtual ~QDummyConfigurableTileWidget() {
      }

      void setName(QString name) {
        this->name = name;
      }

      QString getName() {
        return name;
      }

      void toDummy(bool set) {}

      void reloadConfigurableData() {}
      inline bool valueChanged() { return false; }
      inline bool boundsChanged() { return false; }

    public slots:
      virtual void sl_resetToOriginalValues() {}
      virtual void sl_resetToOriginalValuesAndBounds() {}

    private:
      QString name;

  };

}

#endif /* __QDUMMYCONFIGURABLETILEWIDGET_H_ */

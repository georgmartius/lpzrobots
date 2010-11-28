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
 *   Revision 1.2  2010-11-28 20:33:44  wrabe
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

#ifndef __QVALCONFIGURABLELINEWIDGET_H_
#define __QVALCONFIGURABLELINEWIDGET_H_

#include "QAbstractConfigurableLineWidget.h"

namespace lpzrobots {

  class QValConfigurableLineWidget : public lpzrobots::QAbstractConfigurableLineWidget {

    Q_OBJECT

    public:
      QValConfigurableLineWidget(QGridLayout* parent, Configurable* config, Configurable::paramkey& key);
      virtual ~QValConfigurableLineWidget();

    private slots:
      void sl_spinBoxValueChanged(double);
      void sl_sliderValueChanged(int);
      void sl_execContextMenu(const QPoint &pos);
      void sl_changeBounds();

    private:
      QLabel* labelMinBound;
      QLabel* labelMaxBound;
      QSlider* slider;
      QDoubleSpinBox* dsBox;



  };

} // namespace lpzrobots

#endif /* __QVALCONFIGURABLELINEWIDGET_H_ */

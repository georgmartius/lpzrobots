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

#include "QConfigurableWidget.h"
#include <QVBoxLayout>
#include <QMdiArea>
#include <QMessageBox>
#include <QPalette>

#include "QBoolConfigurableLineWidget.h"
#include "QIntConfigurableLineWidget.h"
#include "QValConfigurableLineWidget.h"

namespace lpzrobots {
  
  QConfigurableWidget::QConfigurableWidget(Configurable* config) :
    config(config) {
    initBody();
    createConfigurableLines();
  }

  QConfigurableWidget::~QConfigurableWidget() {
  }

  void QConfigurableWidget::createConfigurableLines() {

    QGridLayout* grid = new QGridLayout();
    grid->setColumnStretch(3, 100);

    setLayout(grid);
    int numberWidgets = 0;

    Configurable::parammap valMap = config->getParamValMap();
    FOREACHC(Configurable::parammap, valMap, keyIt) {
      Configurable::paramkey key = (*keyIt).first;
      QAbstractConfigurableLineWidget* configLineWidget = new QValConfigurableLineWidget(&layout, config, key);
      configLineWidgetList.append(configLineWidget);

      configLineWidget->setAttribute(Qt::WA_DeleteOnClose);
      configLineWidget->setWindowTitle(QString(key.c_str()));

      grid->addWidget(configLineWidget, numberWidgets / 3, numberWidgets % 3);
      numberWidgets++;



    }


    /*
     Configurable::paramintmap intMap = config->getParamIntMap();
     FOREACHC(Configurable::paramintmap, intMap, keyIt) {
     Configurable::paramkey key = (*keyIt).first;
     QAbstractConfigurableLineWidget* configLineWidget = new QIntConfigurableLineWidget(&layout, config, key);
     configLineWidgetList.append(configLineWidget);
     }
     Configurable::paramboolmap boolMap = config->getParamBoolMap();
     FOREACHC(Configurable::paramboolmap, boolMap, keyIt) {
     Configurable::paramkey key = (*keyIt).first;
     QAbstractConfigurableLineWidget* configLineWidget = new QBoolConfigurableLineWidget(&layout, config, key);
     configLineWidgetList.append(configLineWidget);
     }
     */
    //body.setLayout(&layout);
  }

  void QConfigurableWidget::initBody() {
    setTitle(QString(config->getName().c_str()) + "  -  " + QString(config->getRevision().c_str()) + "  [" + QString::number(
        config->getId()) + "]");
    setFont(QFont("Courier", 14, QFont::Bold));

    QPalette pal = palette();
    pal.setColor(QPalette::AlternateBase, QColor(200, 210, 200));
    setPalette(pal);
    setBackgroundRole(QPalette::AlternateBase);
    setAutoFillBackground(true);
  }

}

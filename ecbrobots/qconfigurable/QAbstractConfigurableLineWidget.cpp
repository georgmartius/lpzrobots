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
 *   Revision 1.1  2010-11-26 12:22:36  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *   - bugfixes
 *   - current development state of QConfigurable (Qt GUI)
 *
 *                                                                         *
 ***************************************************************************/

#include "QAbstractConfigurableLineWidget.h"

#include "selforg/configurable.h"
#include <qgridlayout.h>
#include <QLabel>

namespace lpzrobots {
  
  int QAbstractConfigurableLineWidget::static_lineCounter = 0;

  QAbstractConfigurableLineWidget::QAbstractConfigurableLineWidget(QGridLayout* parentLayout, Configurable* config,
      Configurable::paramkey& key) :
    parentLayout(parentLayout), config(config), key(key), lineIndex(static_lineCounter) {
    parentLayout->addWidget(new QLabel(QString(key.c_str())), lineIndex, 0);
    QLabel* descLabel = new QLabel(QString(config->getParamDescr(key).c_str()));
    descLabel->setWordWrap(true);
    parentLayout->addWidget(descLabel, lineIndex, 5);

    static_lineCounter++;
  }
  
  QAbstractConfigurableLineWidget::~QAbstractConfigurableLineWidget() {
  }

  QSlider* QAbstractConfigurableLineWidget::setAndCreateSlider(int minBound, int maxBound) {
    QSlider* slider = new QSlider();
    slider->setOrientation(Qt::Horizontal);

    // TODO: problem zwischen double und int
    slider->setMinimum(minBound);
    slider->setMaximum( maxBound);
    slider->setValue(config->getParam(key));
//    slider->setTickInterval((maxBound-minBound)/10);
    parentLayout->addWidget(slider, lineIndex, 3);
    return slider;
  }

  QLabel* QAbstractConfigurableLineWidget::setAndCreateMinBoundLabel(QString minBoundString) {
    QLabel* labelMinBound = new QLabel();
    labelMinBound->setText(minBoundString + " <=");
    labelMinBound->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    parentLayout->addWidget(labelMinBound, lineIndex, 2);
    return labelMinBound;
  }

  QLabel* QAbstractConfigurableLineWidget::setAndCreateMaxBoundLabel(QString maxBoundString) {
    QLabel* labelMaxBound = new QLabel();
    labelMaxBound->setText("<= " + maxBoundString);
    parentLayout->addWidget(labelMaxBound, lineIndex, 4);
    return labelMaxBound;
  }

}

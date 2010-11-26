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

#include "QValConfigurableLineWidget.h"
#include <QLineEdit>
#include <QDoubleValidator>
#include <QSlider>
#include <QLabel>

namespace lpzrobots {
  
  QValConfigurableLineWidget::QValConfigurableLineWidget(QGridLayout* parentLayout, Configurable* config,
      Configurable::paramkey& key) :
    QAbstractConfigurableLineWidget(parentLayout, config, key) {
    QLineEdit* edit = new QLineEdit();
    edit->setAcceptDrops(false);
    edit->setMinimumWidth(100);
    parentLayout->addWidget(edit, lineIndex, 1);

    double minBound = config->getParamvalBounds(key).first;
    double maxBound = config->getParamvalBounds(key).second;
    double value = config->getParam(key);

    QDoubleValidator* valitator = new QDoubleValidator();
    valitator->setNotation(QDoubleValidator::StandardNotation);
    valitator->setBottom(minBound);
    valitator->setTop(maxBound);
    edit->setValidator(valitator);
    edit->setText(QString::number(value));

    setAndCreateMinBoundLabel(QString::number(minBound));
    setAndCreateMaxBoundLabel(QString::number(maxBound));

    setAndCreateSlider(minBound,maxBound);
  }
  
  QValConfigurableLineWidget::~QValConfigurableLineWidget() {
    // TODO Auto-generated destructor stub
  }

}

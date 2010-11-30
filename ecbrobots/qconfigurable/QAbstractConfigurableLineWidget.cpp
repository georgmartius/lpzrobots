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
 *   Revision 1.3  2010-11-30 17:07:06  wrabe
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

#include "QAbstractConfigurableLineWidget.h"

#include "selforg/configurable.h"
#include <QGridLayout>
#include <QMessageBox>
#include <QLabel>


namespace lpzrobots {
  
  int QAbstractConfigurableLineWidget::static_lineCounter = 0;
  QSize QAbstractConfigurableLineWidget::widgetSize = QSize(300, 80);

  QAbstractConfigurableLineWidget::QAbstractConfigurableLineWidget(Configurable* config, Configurable::paramkey& key) :
    config(config), key(key), lineIndex(static_lineCounter) {

    setMinimumSize(QAbstractConfigurableLineWidget::widgetSize);
    setMaximumSize(QAbstractConfigurableLineWidget::widgetSize);

    setFrameStyle(QFrame::StyledPanel | QFrame::Raised);

    setBackgroundRole(QPalette::Background);
    setAutoFillBackground(true);
  }
  
  QAbstractConfigurableLineWidget::~QAbstractConfigurableLineWidget() {
  }

  QString QAbstractConfigurableLineWidget::getConfigurableName() {
    return QString(key.c_str());
  }
//  QDoubleSpinBox* QAbstractConfigurableLineWidget::setAndCreateDoubleSpinBox(double val, int minBound, int maxBound) {
//    QDoubleSpinBox* dsBox = new QDoubleSpinBox();
//    dsBox->setAcceptDrops(false);
//    dsBox->setMinimumWidth(100);
//    dsBox->setMinimum(minBound);
//    dsBox->setMaximum(maxBound);
//    parentLayout->addWidget(dsBox, lineIndex, 1);
//    return dsBox;
//  }
//
//  QSlider* QAbstractConfigurableLineWidget::setAndCreateSlider(int minBound, int maxBound, int steps) {
//    QSlider* slider = new QSlider();
//    slider->setOrientation(Qt::Horizontal);
//    slider->setMinimum(0);
//    slider->setMaximum((maxBound-minBound)*steps);
//    slider->setValue((maxBound-minBound)*steps*config->getParam(key));
//    parentLayout->addWidget(slider, lineIndex, 3);
//    return slider;
//  }
//
//  QLabel* QAbstractConfigurableLineWidget::setAndCreateMinBoundLabel(QString minBoundString) {
//    QLabel* labelMinBound = new QLabel();
//    labelMinBound->setText(minBoundString + " <=");
//    labelMinBound->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
//    labelMinBound->setToolTip("double-click to change");
//    //labelMinBound->setFrameStyle(QFrame::Panel | QFrame::Plain);
//    parentLayout->addWidget(labelMinBound, lineIndex, 2);
//    return labelMinBound;
//  }
//
//  QLabel* QAbstractConfigurableLineWidget::setAndCreateMaxBoundLabel(QString maxBoundString) {
//    QLabel* labelMaxBound = new QLabel();
//    labelMaxBound->setText("<= " + maxBoundString);
//    labelMaxBound->setToolTip("double-click to change");
//    //labelMaxBound->setFrameStyle(QFrame::Panel | QFrame::Plain);
//    parentLayout->addWidget(labelMaxBound, lineIndex, 4);
//    return labelMaxBound;
//  }

}

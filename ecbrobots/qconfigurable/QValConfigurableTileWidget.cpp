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
 *   Revision 1.1  2010-12-03 11:11:41  wrabe
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

#include "QValConfigurableTileWidget.h"
#include <QMessageBox>
#include <QMenu>

namespace lpzrobots {
  
  QValConfigurableTileWidget::QValConfigurableTileWidget(Configurable* config, Configurable::paramkey& key) :
    QAbstractConfigurableTileWidget(config, key) {

    double minBound = config->getParamvalBounds(key).first;
    double maxBound = config->getParamvalBounds(key).second;
    double value = config->getParam(key);
    QString key_name = QString(key.c_str());
    QString toolTipName = QString(config->getParamDescr(key).c_str());
    QString toolTipVals = "min=" + QString::number(minBound) + ", max=" + QString::number(maxBound);

    setLayout(&gridLayoutConfigurableTile);

    lName.setText(key_name);
    lName.setToolTip(toolTipName);
    lName.setFont(QFont("Courier", 12, QFont::Bold));

    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(sl_execContextMenu(const QPoint &)));

    dsBox.setAcceptDrops(false);
    dsBox.setMinimumWidth(100);
    dsBox.setMinimum(minBound);
    dsBox.setMaximum(maxBound);
    dsBox.setToolTip(toolTipVals);
    dsBox.setDecimals(3);
    dsBox.setValue(value);
    dsBox.setSingleStep((maxBound - minBound) / 1000);
    dsBox.setFont(QFont("Courier", 11, QFont::Normal));

    slider.setOrientation(Qt::Horizontal);
    slider.setMinimum(0);
    slider.setMaximum((maxBound - minBound) * 1000);
    slider.setValue((maxBound - minBound) * 1000 * value);
    slider.setToolTip(toolTipVals);

    gridLayoutConfigurableTile.addWidget(&lName, 0, 0, 1, 2, Qt::AlignLeft);
    gridLayoutConfigurableTile.addWidget(&dsBox, 0, 2);
    gridLayoutConfigurableTile.addWidget(&slider, 1, 0, 1, 3);

    connect(&slider, SIGNAL(valueChanged(int)), this, SLOT(sl_sliderValueChanged(int)));
    connect(&dsBox, SIGNAL(valueChanged(double)), this, SLOT(sl_spinBoxValueChanged(double)));

    setBackgroundRole(QPalette::Background);
    setAutoFillBackground(true);
  }
  
  QValConfigurableTileWidget::~QValConfigurableTileWidget() {
  }

  void QValConfigurableTileWidget::setName(QString name) {
    lName.setText(name);
  }

  void QValConfigurableTileWidget::sl_spinBoxValueChanged(double value) {
    double minBound = config->getParamvalBounds(key).first;
    double maxBound = config->getParamvalBounds(key).second;
    //    double value = dsBox->value();
    slider.setValue((maxBound - minBound) * 1000 * value);
    config->setParam(key, value);
  }

  void QValConfigurableTileWidget::sl_sliderValueChanged(int int_value) {
    double minBound = config->getParamvalBounds(key).first;
    double maxBound = config->getParamvalBounds(key).second;
    double value = int_value / ((maxBound - minBound) * 1000);
    dsBox.setValue(value);
    config->setParam(key, value);
  }

  void QValConfigurableTileWidget::sl_execContextMenu(const QPoint &pos) {
    QMenu menu;
    menu.addAction(tr("Change boundaries of this Configurable."), this, SLOT(sl_changeBounds()));
    menu.exec(this->mapToGlobal(pos));
  }
  void QValConfigurableTileWidget::sl_changeBounds() {
    QMessageBox msgBox;
    msgBox.setText("This is a dummy: will replaced in future by a \ndialog to change the boundaries of the Configurable.");
    msgBox.exec();
  }
  void QValConfigurableTileWidget::toDummy(bool set) {
    if(set) {
      setAutoFillBackground(false);
      lName.hide();
      slider.hide();
      dsBox.hide();
      repaint();
    }else {
      setAutoFillBackground(true);
      lName.show();
      slider.show();
      dsBox.show();
      repaint();
    }
  }

}

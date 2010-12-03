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
 *   Revision 1.2  2010/11/30 17:07:06  wrabe
 *   - new class QConfigurableTileShowHideDialog
 *   - try to introduce user-arrangeable QConfigurationTiles (current work, not finished)
 *
 *   Revision 1.1  2010/11/26 12:22:36  guettler
 *   - Configurable interface now allows to set bounds of paramval and paramint
 *     * setting bounds for paramval and paramint is highly recommended (for QConfigurable (Qt GUI).
 *   - bugfixes
 *   - current development state of QConfigurable (Qt GUI)
 *
 *                                                                         *
 ***************************************************************************/

#include "QIntConfigurableTileWidget.h"
#include <QMessageBox>
#include <QMenu>


namespace lpzrobots {
  
  QIntConfigurableTileWidget::QIntConfigurableTileWidget(Configurable* config, Configurable::paramkey& key) :
    QAbstractConfigurableTileWidget(config, key) {

    int minBound = config->getParamvalBounds(key).first;
    int maxBound = config->getParamvalBounds(key).second;
    int value = config->getParam(key);
    QString key_name = QString(key.c_str());
    QString toolTipName = QString(config->getParamDescr(key).c_str());
    QString toolTipVals = "min=" + QString::number(minBound) + ", max=" + QString::number(maxBound);

    setLayout(&gridLayoutConfigurableTile);

    lName.setText(key_name);
    lName.setToolTip(toolTipName);
    lName.setFont(QFont("Courier", 12, QFont::Bold));

    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(sl_execContextMenu(const QPoint &)));

    spBox.setAcceptDrops(false);
    spBox.setMinimumWidth(100);
    spBox.setMinimum(minBound);
    spBox.setMaximum(maxBound);
    spBox.setToolTip(toolTipVals);
    spBox.setValue(value);
    spBox.setSingleStep(1);
    spBox.setFont(QFont("Courier", 11, QFont::Normal));

    slider.setOrientation(Qt::Horizontal);
    slider.setMinimum(minBound);
    slider.setMaximum(maxBound);
    slider.setValue(value);
    slider.setToolTip(toolTipVals);

    gridLayoutConfigurableTile.addWidget(&lName, 0, 0, 1, 2, Qt::AlignLeft);
    gridLayoutConfigurableTile.addWidget(&spBox, 0, 2);
    gridLayoutConfigurableTile.addWidget(&slider, 1, 0, 1, 3);

    connect(&slider, SIGNAL(valueChanged(int)), this, SLOT(sl_sliderValueChanged(int)));
    connect(&spBox, SIGNAL(valueChanged(int)), this, SLOT(sl_spinBoxValueChanged(int)));

    setBackgroundRole(QPalette::Background);
    setAutoFillBackground(true);
  }
  
  QIntConfigurableTileWidget::~QIntConfigurableTileWidget() {
  }

  void QIntConfigurableTileWidget::setName(QString name) {
    lName.setText(name);
  }

  void QIntConfigurableTileWidget::sl_spinBoxValueChanged(int value) {
    slider.setValue(value);
    config->setParam(key, value);
  }

  void QIntConfigurableTileWidget::sl_sliderValueChanged(int value) {
    spBox.setValue(value);
    config->setParam(key, value);
  }

  void QIntConfigurableTileWidget::sl_execContextMenu(const QPoint &pos) {
    QMenu menu;
    menu.addAction(tr("Change boundaries of this Configurable."), this, SLOT(sl_changeBounds()));
    menu.exec(this->mapToGlobal(pos));
  }
  void QIntConfigurableTileWidget::sl_changeBounds() {
    QMessageBox msgBox;
    msgBox.setText("This is a dummy: will replaced in future by a \ndialog to change the boundaries of the Configurable.");
    msgBox.exec();
  }
  void QIntConfigurableTileWidget::toDummy(bool set) {
    if(set) {
      setAutoFillBackground(false);
      lName.hide();
      slider.hide();
      spBox.hide();
      repaint();
    }else {
      setAutoFillBackground(true);
      lName.show();
      slider.show();
      spBox.show();
      repaint();
    }
  }

}

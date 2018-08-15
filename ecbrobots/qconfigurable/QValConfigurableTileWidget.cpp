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
 *   Revision 1.14  2011-04-06 06:50:03  guettler
 *   - minimal increment of spinbox is now correctly calculated, dependent of the used decimals
 *
 *   Revision 1.13  2011/03/21 17:34:28  guettler
 *   - color changes now if parameter value or bounds is changed
 *   - adapted to enhanced configurable interface
 *
 *   Revision 1.12  2011/02/11 12:12:11  guettler
 *   - UI: some seperators added
 *
 *   Revision 1.11  2011/02/04 13:03:16  wrabe
 *   - bugfix: Configurables are restored now when event "CommunicationStateWillChange" occurs, not in destructor
 *
 *   Revision 1.10  2011/01/28 12:15:37  guettler
 *   - restore of AutoSave File from a backup implemented
 *   - reset to original values, values AND bounds for Configurable implemented
 *   - reset to original values for tileWidgets implemented
 *
 *   Revision 1.9  2011/01/28 11:32:12  guettler
 *   - original values are written back to the Configurable instances if the QConfigurable interface is restarted
 *
 *   Revision 1.8  2010/12/16 18:37:40  wrabe
 *   -added several tooltips
 *   -corrected sentences, notation, syntax for improved informational value
 *   -bugfix: if in collapsed mode, all tiles were stored as invisible
 *   -cosmetic ui changes
 *   -other minor things
 *
 *   Revision 1.7  2010/12/16 16:39:25  wrabe
 *   - drag&drop reworked: user can now drag a parameter to a any place
 *   - rearrangement of parameters now made only when user wants this
 *   - bugfixes
 *
 *   Revision 1.6  2010/12/15 17:26:28  wrabe
 *   - number of colums for tileWidgets and width of tileWidgets can
 *   now be changed (independently for each Configurable)
 *   - bugfixes
 *
 *   Revision 1.5  2010/12/09 17:00:08  wrabe
 *   - load / save function of ConfigurableState (configurable + GUI)
 *   - autoload / autosave function of ConfigurableState (configurable
 *     + GUI)
 *   - handling of equal Configurable names implemented for autoload
 *     and -save
 *   - bugfixing
 *
 *   Revision 1.4  2010/12/08 17:52:57  wrabe
 *   - bugfixing/introducing new feature:
 *   - folding of the ConfigurableWidgets now awailable
 *   - highlight the ConfigurableTile when hoovered by mouse
 *   - load/store of the state of a ConfigurableWidget to file
 *
 *   Revision 1.3  2010/12/06 17:49:34  wrabe
 *   - new QConfigurableSetBoundsDialog to change the
 *     boundaries of the Configurables (reacheble now by
 *     context menu of the ConfigurableTile (only paramval/
 *     paramint))
 *
 *   Revision 1.2  2010/12/06 14:08:57  guettler
 *   - bugfixes
 *   - number of decimals is now calculated
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

#include "QValConfigurableTileWidget.h"
#include <QMessageBox>
#include <QMenu>
#include <QLabel>
#include <QSlider>
#include <QDoubleSpinBox>
#include "QConfigurableSetBoundsDialog.h"
#include <math.h>

namespace lpzrobots {
  
  QValConfigurableTileWidget::QValConfigurableTileWidget(Configurable* config, Configurable::paramkey& key, QMap<QGridPos, QAbstractConfigurableTileWidget*>& tileIndexConfigWidgetMap) :
    QAbstractConfigurableTileWidget(config, key, tileIndexConfigWidgetMap), origBounds(config->getParamvalBounds(key)), origValue(config->getParam(key)), stopSignaling(false) {

    double minBound = config->getParamvalBounds(key).first;
    double maxBound = config->getParamvalBounds(key).second;
    double value = config->getParam(key);
    QString key_name = QString(key.c_str());
    QString toolTipName = QString(config->getParamDescr(key).c_str());
    QString toolTipVals;
    toolTipVals.append("min=" + QString::number(minBound) + ", max=" + QString::number(maxBound));
    toolTipVals.append(" (" + QString::number(calcNumberDecimals()) + " decimals)");

    setLayout(&gridLayoutConfigurableTile);

    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(sl_execContextMenu(const QPoint &)));

    lName.setText(key_name);
    lName.setToolTip(toolTipName);
    lName.setFont(QFont("Arial Narrow", 10, QFont::Normal));
    lName.setWordWrap(true);
    //lName.setMaximumWidth(QAbstractConfigurableTileWidget::widgetSize.width() - 150);

    dsBox.setAcceptDrops(false);
    dsBox.setFixedWidth(80);
    dsBox.setMinimum(minBound);
    dsBox.setMaximum(maxBound);
    dsBox.setToolTip(toolTipVals);
    dsBox.setDecimals(calcNumberDecimals());
    dsBox.setValue(value);
    dsBox.setSingleStep(1 / exp10(dsBox.decimals()));
    dsBox.setFont(QFont("Courier", 11, QFont::Normal));

    slider.setOrientation(Qt::Horizontal);
    slider.setMinimum(minBound * SCALE_FACTOR_SLIDER);
    slider.setMaximum(maxBound * SCALE_FACTOR_SLIDER);
    slider.setValue(SCALE_FACTOR_SLIDER * value);
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
    if (!stopSignaling) {
      stopSignaling = true;
      slider.setValue(SCALE_FACTOR_SLIDER * value);
      config->setParam(key, value);
      stopSignaling = false;
    }
    updatePaletteChanged();
  }

  void QValConfigurableTileWidget::sl_sliderValueChanged(int int_value) {
    if (!stopSignaling) {
      stopSignaling = true;
      double value = int_value / (double)SCALE_FACTOR_SLIDER;
      dsBox.setValue(value);
      config->setParam(key, value);
      stopSignaling = false;
    }
    updatePaletteChanged();
  }

  void QValConfigurableTileWidget::sl_execContextMenu(const QPoint &pos) {
    QMenu menu;
    menu.addAction(tr("change boundaries of this configurable ..."), this, SLOT(sl_changeBounds()));
    menu.addSeparator();
    menu.addAction(tr("reset to original value"), this, SLOT(sl_resetToOriginalValues()));
    menu.addAction(tr("reset to original value AND bounds"), this, SLOT(sl_resetToOriginalValuesAndBounds()));
    menu.exec(this->mapToGlobal(pos));
  }
  void QValConfigurableTileWidget::sl_changeBounds() {
    QConfigurableSetBoundsDialog* dialog = new QConfigurableSetBoundsDialog(config, key, QConfigurableSetBoundsDialog::MODE_PARAMVAL);
    if (dialog->exec() == QDialog::Accepted)
      setBounds(config->getParamvalBounds(key));
    delete (dialog);
    updatePaletteChanged();
  }

  void QValConfigurableTileWidget::toDummy(bool set) {
    if (set) {
      setAutoFillBackground(false);
      lName.hide();
      slider.hide();
      dsBox.hide();
      repaint();
    } else {
      setAutoFillBackground(true);
      lName.show();
      slider.show();
      dsBox.show();
      repaint();
    }
  }

  int QValConfigurableTileWidget::calcNumberDecimals() {
    double minBound = config->getParamvalBounds(key).first;
    double maxBound = config->getParamvalBounds(key).second;
    double range = (maxBound - minBound) / 2.;
    if (range <= 0.1)
      return 4;
    else if (range <= 1)
      return 3;
    else if (range <= 10)
      return 2;
    else if (range <= 100)
      return 1;
    else
      return 0;
  }

  void QValConfigurableTileWidget::sl_resetToOriginalValues() {
    config->setParam(key, origValue);
    // values
    dsBox.setValue(origValue);
    sl_spinBoxValueChanged(origValue);
    updatePaletteChanged();
  }

  void QValConfigurableTileWidget::sl_resetToOriginalValuesAndBounds() {
    config->setParam(key, origValue);
    config->setParamBounds(key, origBounds.first, origBounds.second);
    setBounds(config->getParamvalBounds(key));
    // values
    dsBox.setValue(origValue);
    sl_spinBoxValueChanged(origValue);
    updatePaletteChanged();
  }

  void QValConfigurableTileWidget::setBounds(Configurable::paramvalBounds bounds) {
    double minBound = bounds.first;
    double maxBound = bounds.second;
    QString toolTipVals;
    toolTipVals.append("min=" + QString::number(minBound) + ", max=" + QString::number(maxBound));
    toolTipVals.append(" (" + QString::number(calcNumberDecimals()) + " decimals)");
    dsBox.setDecimals(calcNumberDecimals());
    dsBox.setMinimum(minBound);
    dsBox.setMaximum(maxBound);
    dsBox.setSingleStep(1 / exp10(dsBox.decimals()));
//    dsBox.setSingleStep((maxBound - minBound) / SCALE_FACTOR_SPINBOX);
    dsBox.setToolTip(toolTipVals);
    slider.setMinimum(minBound * SCALE_FACTOR_SLIDER);
    slider.setMaximum(maxBound * SCALE_FACTOR_SLIDER);
    slider.setToolTip(toolTipVals);
    updatePaletteChanged();
  }

  void QValConfigurableTileWidget::reloadConfigurableData() {
    stopSignaling = true;
    setBounds(config->getParamvalBounds(key));
    double value = config->getParam(key);
    dsBox.setValue(value);
    slider.setValue(SCALE_FACTOR_SLIDER * value);
    stopSignaling = false;
    updatePaletteChanged();
  }


  void QValConfigurableTileWidget::sl_resize(QSize newSize) {
     if (newSize.width()<130)
       lName.setMaximumWidth(30);
     else
       lName.setMaximumWidth(newSize.width() - 100);
     QAbstractConfigurableTileWidget::sl_resize(newSize);
   }



}

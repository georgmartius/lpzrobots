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
 *   Revision 1.11  2011/03/21 17:34:28  guettler
 *   - color changes now if parameter value or bounds is changed
 *   - adapted to enhanced configurable interface
 *
 *   Revision 1.10  2011/02/04 13:03:16  wrabe
 *   - bugfix: Configurables are restored now when event "CommunicationStateWillChange" occurs, not in destructor
 *
 *   Revision 1.9  2011/01/28 12:15:37  guettler
 *   - restore of AutoSave File from a backup implemented
 *   - reset to original values, values AND bounds for Configurable implemented
 *   - reset to original values for tileWidgets implemented
 *
 *   Revision 1.8  2011/01/28 11:32:12  guettler
 *   - original values are written back to the Configurable instances if the QConfigurable interface is restarted
 *
 *   Revision 1.7  2011/01/24 17:49:02  guettler
 *   - bugfix: if unchecked, set parambool in Configurable to false
 *
 *   Revision 1.6  2010/12/16 18:37:39  wrabe
 *   -added several tooltips
 *   -corrected sentences, notation, syntax for improved informational value
 *   -bugfix: if in collapsed mode, all tiles were stored as invisible
 *   -cosmetic ui changes
 *   -other minor things
 *
 *   Revision 1.5  2010/12/16 16:39:25  wrabe
 *   - drag&drop reworked: user can now drag a parameter to a any place
 *   - rearrangement of parameters now made only when user wants this
 *   - bugfixes
 *
 *   Revision 1.4  2010/12/09 17:00:08  wrabe
 *   - load / save function of ConfigurableState (configurable + GUI)
 *   - autoload / autosave function of ConfigurableState (configurable
 *     + GUI)
 *   - handling of equal Configurable names implemented for autoload
 *     and -save
 *   - bugfixing
 *
 *   Revision 1.3  2010/12/08 17:52:57  wrabe
 *   - bugfixing/introducing new feature:
 *   - folding of the ConfigurableWidgets now awailable
 *   - highlight the ConfigurableTile when hoovered by mouse
 *   - load/store of the state of a ConfigurableWidget to file
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

#include <QMenu>
#include "QBoolConfigurableTileWidget.h"

namespace lpzrobots {
  
  QBoolConfigurableTileWidget::QBoolConfigurableTileWidget(Configurable* config, Configurable::paramkey& key, QMap<QGridPos, QAbstractConfigurableTileWidget*>& tileIndexConfigWidgetMap) :
    QAbstractConfigurableTileWidget(config, key, tileIndexConfigWidgetMap), origValue(config->getParam(key)) {

    QString key_name = QString(key.c_str());
    QString toolTipName = QString(config->getParamDescr(key).c_str());

    setLayout(&gridLayoutConfigurableTile);

    cbBool.setText(key_name);
    cbBool.setToolTip(toolTipName);
    cbBool.setFont(QFont("Arial Narrow", 10, QFont::Normal));
    if (config->getParam(key))
      cbBool.setCheckState(Qt::Checked);
    else cbBool.setCheckState(Qt::Unchecked);

    gridLayoutConfigurableTile.addWidget(&cbBool, 0, 0, 1, 2, Qt::AlignLeft | Qt::AlignTop);

    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(sl_execContextMenu(const QPoint &)));
    connect(&cbBool, SIGNAL(stateChanged(int)), this, SLOT(sl_checkStateChanged(int)));

    setBackgroundRole(QPalette::Background);
    setAutoFillBackground(true);
  }
  
  QBoolConfigurableTileWidget::~QBoolConfigurableTileWidget() {
  }

  void QBoolConfigurableTileWidget::setName(QString name) {
    cbBool.setText(name);
  }

  void QBoolConfigurableTileWidget::sl_checkStateChanged(int state) {
    if(state == Qt::Checked) config->setParam(key, true); else config->setParam(key, false);
    updatePaletteChanged();
  }

  void QBoolConfigurableTileWidget::sl_execContextMenu(const QPoint &pos) {
    QMenu menu;
    menu.addAction(tr("reset to original value"), this, SLOT(sl_resetToOriginalValues()));
    menu.exec(this->mapToGlobal(pos));
  }

  void QBoolConfigurableTileWidget::toDummy(bool set) {
    if(set) {
      setAutoFillBackground(false);
      cbBool.hide();
      repaint();
    }else {
      setAutoFillBackground(true);
      cbBool.show();
      repaint();
    }
  }

  void QBoolConfigurableTileWidget::sl_resetToOriginalValuesAndBounds() {
    sl_resetToOriginalValues();
    updatePaletteChanged();
  }


  void QBoolConfigurableTileWidget::sl_resetToOriginalValues() {
    config->setParam(key, origValue);
    if(origValue) cbBool.setCheckState(Qt::Checked); else cbBool.setCheckState(Qt::Unchecked);
    updatePaletteChanged();
  }

  void QBoolConfigurableTileWidget::reloadConfigurableData() {
    bool value = config->getParam(key);
    if(value) cbBool.setCheckState(Qt::Checked); else cbBool.setCheckState(Qt::Unchecked);
    updatePaletteChanged();
  }

}

/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    jhoffmann@informatik.uni-leipzig.de                                  *
 *    wolfgang.rabe@01019freenet.de                                        *
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
 *   Revision 1.1  2011/07/01 12:32:15  guettler
 *   - pull out qconfigurable part of ecb_robots to get stand the alone library libconfigurator
 *
 *   Revision 1.5  2010/12/16 18:37:39  wrabe
 *   -added several tooltips
 *   -corrected sentences, notation, syntax for improved informational value
 *   -bugfix: if in collapsed mode, all tiles were stored as invisible
 *   -cosmetic ui changes
 *   -other minor things
 *
 *   Revision 1.4  2010/12/16 16:39:25  wrabe
 *   - drag&drop reworked: user can now drag a parameter to a any place
 *   - rearrangement of parameters now made only when user wants this
 *   - bugfixes
 *
 *   Revision 1.3  2010/12/13 16:22:18  wrabe
 *   - autosave function rearranged
 *   - bugfixes
 *
 *   Revision 1.2  2010/12/03 11:11:53  wrabe
 *   - now handled paramVal, paramInt and paramBool, all the params are displayed
 *     as ConfigurableTiles witch can be show and hide seperatly or arranged by user
 *     (showHideDialog reacheble by contextMenu (right click an the Widget containing
 *     the tiles ), arrange the Tiles is can done by drag and drop (there is no history or
 *     storage implementet yet))
 *
 *   Revision 1.1  2010/11/30 17:07:06  wrabe
 *   - new class QConfigurableTileShowHideDialog
 *   - try to introduce user-arrangeable QConfigurationTiles (current work, not finished)
 *
 *                                                                         *
 ***************************************************************************/

#ifndef __QCONFIGURABLETILESHOWHIDEDIALOG_H_
#define __QCONFIGURABLETILESHOWHIDEDIALOG_H_
#include <QDialog>
#include <QCheckBox>
#include <QScrollArea>
#include <QFrame>
#include <QMap>
#include <QGridLayout>
#include <QDialogButtonBox>
#include <QPushButton>
#include "QGridPos.h"



#include "QAbstractConfigurableTileWidget.h"

namespace lpzrobots {
  
  class QConfigurableTileShowHideDialog : public QDialog {

    Q_OBJECT

    public:
      QConfigurableTileShowHideDialog(QMap<QString, QAbstractConfigurableTileWidget*>& configLineWidgetMap, QMap<QGridPos, QAbstractConfigurableTileWidget*>& tileIndexConfigWidgetMap, int numberOfTilesPerRow);
      virtual ~QConfigurableTileShowHideDialog();
      //void setConfigurableTileNames(QStringList configurabelTileName);

      int getNumberOfVisibleTiles() { return numberOfVisibleTiles; }

    private slots:
      void sl_dialogAccept();
      void sl_dialogSelectAll();
      void sl_dialogSelectNone();


    private:
      QFrame* cbFrame;
      QScrollArea* scrollArea;
      QDialogButtonBox* buttonBox;
      QList<QCheckBox*> checkBoxConfiguableShowHideList;
      QMap<QString, QAbstractConfigurableTileWidget*>& configTileWidgetMap;
      QMap<QGridPos, QAbstractConfigurableTileWidget*>& tileIndexConfigWidgetMap;
      QPushButton* pbSelectAll;
      QPushButton* pbSelectNone;
      int cbFrame_ypos;
      int numberOfTilesPerRow;
      int numberOfVisibleTiles;



  };

}

#endif /* __QCONFIGURABLETILESHOWHIDEDIALOG_H_ */

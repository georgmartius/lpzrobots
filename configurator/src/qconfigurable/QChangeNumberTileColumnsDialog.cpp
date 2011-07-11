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
 *   Revision 1.1  2011/07/01 12:32:16  guettler
 *   - pull out qconfigurable part of ecb_robots to get stand the alone library libconfigurator
 *
 *   Revision 1.3  2010/12/16 18:37:39  wrabe
 *   -added several tooltips
 *   -corrected sentences, notation, syntax for improved informational value
 *   -bugfix: if in collapsed mode, all tiles were stored as invisible
 *   -cosmetic ui changes
 *   -other minor things
 *
 *   Revision 1.2  2010/12/16 16:39:25  wrabe
 *   - drag&drop reworked: user can now drag a parameter to a any place
 *   - rearrangement of parameters now made only when user wants this
 *   - bugfixes
 *
 *   Revision 1.1  2010/12/15 17:26:28  wrabe
 *   - number of colums for tileWidgets and width of tileWidgets can
 *   now be changed (independently for each Configurable)
 *   - bugfixes
 *
 *   Revision 1.2  2010/12/07 10:08:40  wrabe
 *   - disabled resizing of QChangeNumberTileColumnsDialog
 *
 *   Revision 1.1  2010/12/06 17:49:34  wrabe
 *   - new QChangeNumberTileColumnsDialog to change the
 *     boundaries of the Configurables (reacheble now by
 *     context menu of the ConfigurableTile (only paramval/
 *     paramint))
 *
 *                                                                         *
 ***************************************************************************/

#include "QChangeNumberTileColumnsDialog.h"
#include <QLabel>
#include <QString>
#include <QMessageBox>

namespace lpzrobots {
  
  QChangeNumberTileColumnsDialog::QChangeNumberTileColumnsDialog(int* tileCount) :
    tileCount(tileCount) {
    setLayout(&dialogGridLayout);
    setWindowTitle("Set number of columns per row");
    setSizeGripEnabled(false);
    layout()->setSizeConstraint( QLayout::SetFixedSize );

    lTextLineEdit.setText("number of columns per row");

    spNumberTiles.setAcceptDrops(false);
    spNumberTiles.setMinimum(1);
    spNumberTiles.setMaximum(10);
    spNumberTiles.setToolTip("Sets the number of placeable parameters per row.");
    spNumberTiles.setValue(*tileCount);
    spNumberTiles.setSingleStep(1);
    spNumberTiles.setFont(QFont("Courier", 11, QFont::Normal));


    buttonBox.addButton(QDialogButtonBox::Ok);
    buttonBox.addButton(QDialogButtonBox::Cancel);
    connect(&buttonBox, SIGNAL(accepted()), this, SLOT(sl_dialogAccept()));
    connect(&buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

    dialogGridLayout.addWidget(&lTextLineEdit, 0, 0, Qt::AlignRight);
    dialogGridLayout.addWidget(&spNumberTiles, 0, 1, Qt::AlignLeft);
    dialogGridLayout.addWidget(&buttonBox, 1, 0, 1, 2);

  }
  
  QChangeNumberTileColumnsDialog::~QChangeNumberTileColumnsDialog() {
  }


  void QChangeNumberTileColumnsDialog::sl_dialogAccept() {
    *tileCount = spNumberTiles.value();
    this->accept();
  }

}

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
 *   Revision 1.1  2010-12-15 17:26:28  wrabe
 *   - number of colums for tileWidgets and width of tileWidgets can
 *   now be changed (independently for each Configurable)
 *   - bugfixes
 *
 *   Revision 1.1  2010/12/06 17:49:34  wrabe
 *   - new QConfigurableSetBoundsDialog to change the
 *     boundaries of the Configurables (reacheble now by
 *     context menu of the ConfigurableTile (only paramval/
 *     paramint))
 *
 *                                                                         *
 ***************************************************************************/

#ifndef __QCHANGE_NUMBER_TILECOLUMNS_DIALOG_H_
#define __QCHANGE_NUMBER_TILECOLUMNS_DIALOG_H_
#include <QDialog>
#include <QLabel>
#include <QSpinBox>
#include <QDialogButtonBox>
#include <QGridLayout>

namespace lpzrobots {
  
  class QChangeNumberTileColumnsDialog : public QDialog {

    Q_OBJECT

    public:
    QChangeNumberTileColumnsDialog(int* tileCount);
      virtual ~QChangeNumberTileColumnsDialog();

    private slots:
      void sl_dialogAccept();


    private:
      QIntValidator* intValidator;
      QLabel lTextLineEdit;
      QSpinBox spNumberTiles;
      QDialogButtonBox buttonBox;
      QGridLayout dialogGridLayout;
      int* tileCount;

  };

}

#endif /* __QCHANGE_NUMBER_TILECOLUMNS_DIALOG_H_ */

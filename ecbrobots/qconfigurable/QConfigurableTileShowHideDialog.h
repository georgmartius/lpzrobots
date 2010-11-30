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
 *   Revision 1.1  2010-11-30 17:07:06  wrabe
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



#include "QAbstractConfigurableLineWidget.h"

namespace lpzrobots {
  
  class QConfigurableTileShowHideDialog : public QDialog {

    Q_OBJECT

    public:
      QConfigurableTileShowHideDialog(QMap<QString, QAbstractConfigurableLineWidget*> configLineWidgetMap, QGridLayout *grid);
      virtual ~QConfigurableTileShowHideDialog();
      //void setConfigurableTileNames(QStringList configurabelTileName);

    private slots:
      void sl_dialogAccept();
      void sl_dialogReject();


    private:
      QFrame* cbFrame;
      QScrollArea* scrollArea;
      QDialogButtonBox* buttonBox;
      QList<QCheckBox*> checkBoxConfiguableShowHideList;
      QMap<QString, QAbstractConfigurableLineWidget*> configLineWidgetMap;
      QGridLayout *parentGridLayout;
      int cbFrame_ypos;




  };

}

#endif /* __QCONFIGURABLETILESHOWHIDEDIALOG_H_ */

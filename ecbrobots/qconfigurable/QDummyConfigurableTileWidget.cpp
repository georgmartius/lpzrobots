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
 *   Revision 1.1  2010-12-15 11:24:40  guettler
 *   -new QDummyConfigurableTileWidget
 *
 *                                                                         *
 ***************************************************************************/

#include "QDummyConfigurableTileWidget.h"

namespace lpzrobots {
  
  QDummyConfigurableTileWidget::QDummyConfigurableTileWidget(Configurable* config) : QAbstractConfigurableTileWidget(config, "dummyKEY") {


      setLayout(&gridLayoutConfigurableTile);

      QString key_name = QString(key.c_str());
      lName.setText(key_name);
      gridLayoutConfigurableTile.addWidget(&lName, 0, 0, 1, 2, Qt::AlignLeft);

      setBackgroundRole(QPalette::Background);
      setAutoFillBackground(true);
    }

  QDummyConfigurableTileWidget::~QDummyConfigurableTileWidget() { }


    void QDummyConfigurableTileWidget::toDummy(bool set) {
      if(set) {
        setAutoFillBackground(false);
        lName.hide();
        repaint();
      }else {
        setAutoFillBackground(true);
        lName.show();
        repaint();
      }
    }
  

} // namespace lpzrobots

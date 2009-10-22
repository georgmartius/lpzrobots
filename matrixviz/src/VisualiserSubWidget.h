/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    mam06fyl@studserv.uni-leipzig.de (robot14)                           *
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
 *   DESCRIPTION                                                           *
 *                                                                         *
 *   Visualization tool for matrices...                                    *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#ifndef __VISUALISATIONSUBWIDGET_H_
#define __VISUALISATIONSUBWIDGET_H_

#include <QtGui>

//class AbstractPlotChannel;

#include "MatrixPlotChannel.h"
#include "AbstractVisualisation.h" //Abstract~

#include "AbstractRobotSubWidget.h"


#include <iostream>


class VisualiserSubWidget: public AbstractRobotSubWidget {

Q_OBJECT

public:
  VisualiserSubWidget(MatrixPlotChannel *channel, QWidget *parent = 0);
  virtual ~VisualiserSubWidget();


public slots:
  void updateViewableChannels();
  void switchVisMode(int index);

protected:

  QComboBox *vizChoice;

private:

  QWidget *visualisation;
  QVBoxLayout *mainLayout;

  MatrixPlotChannel *channel;

  void initGui();
  void initVisTypes();

};

#endif /* __VISUALISATIONSUBWIDGET_H_ */

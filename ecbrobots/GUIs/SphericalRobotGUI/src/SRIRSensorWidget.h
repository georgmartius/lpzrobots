/***************************************************************************
 *   Copyright (C) 2008 by mc   *
 *   mc@linux-6hav   *
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
 ***************************************************************************/
#ifndef SR_IRSENSOR_WIDGET_H
#define SR_IRSENSOR_WIDGET_H

#include "SphericalRobotSubWidget.h"

#define NUMBER_IR_SENSORS 12


class SRIRSensorWidget: public SphericalRobotSubWidget {

Q_OBJECT

public:
    SRIRSensorWidget(QWidget *parent = 0);

public slots:
  void updateViewableChannels();

protected:
  void paintEvent(QPaintEvent *);
//   int convertToByte(double doubleVal);
  
private:
  int ir_sensors[NUMBER_IR_SENSORS];
//   std::list<int> ir_sensors;
  

};



#endif


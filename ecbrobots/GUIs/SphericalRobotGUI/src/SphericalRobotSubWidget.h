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
#ifndef SPHERICAL_ROBOT_SUB_WIDGET_H
#define SPHERICAL_ROBOT_SUB_WIDGET_H

#include <QtGui>

// #include "AbstractPipeReader.h"
// #include "AbstractPipeFilter.h"

class AbstractPlotChannel;


/*
#include "IRChannel.h"
#include "MotorCurrentChannel.h"
#include "MotorSpeedChannel.h"
#include "TiltChannel.h"
#include "AxesChannel.h"*/


// class AbstractPlotChannel;
#include "AbstractPlotChannel.h"

#include <list>
#include <iostream>
#include <string>


class SphericalRobotSubWidget: public QWidget {

Q_OBJECT

public:
    SphericalRobotSubWidget(QWidget *parent = 0) : QWidget(parent)
  {
//      channelList.clear();
  };

  void addPlotChannel(AbstractPlotChannel* c) 
  {

    std::cout << "SRSubWidget:addPlotChannel(" << c->getChannelName() << ")" << std::endl;
    
    this->channelList.push_back(c);
  }
  
  int convertToByte(double doubleVal) {
      // insert check byteVal<255
    int byteVal =(int)((doubleVal+1.)*128.0);
    return (byteVal<255?byteVal:254); //values 0..128..255
  };
  
public slots:
  virtual void updateViewableChannels() = 0;
  
protected:

//   void paintEvent(QPaintEvent *);
  std::list<AbstractPlotChannel*> channelList;
  
  
  
private:
  

  
};

#endif

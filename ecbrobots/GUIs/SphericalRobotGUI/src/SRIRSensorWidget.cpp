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
  #include "SRIRSensorWidget.h"
  #include <iostream>
  #include <list>
  #include <QTimer>
  
  SRIRSensorWidget::SRIRSensorWidget ( QWidget *parent )
      : SphericalRobotSubWidget ( parent )
  {
  
    this->setMaximumSize ( 200,200 );
    this->setMinimumSize ( 200,200 );
  
    // init viewable sensor values
    for ( int i=0;i<NUMBER_IR_SENSORS;i++ ) {
      ir_sensors[i] = 300;
    }
  
    QTimer *timer = new QTimer ( this );
    connect ( timer, SIGNAL ( timeout() ), this, SLOT ( updateViewableChannels() ) );
    timer->start ( 100 );
  
  }
  
  void SRIRSensorWidget::updateViewableChannels ()
  {
  
    //   std::cout << "SIRSensorWidget: updateIRSensors()" << std::endl;
    //   int j=0;
    for ( std::list<AbstractPlotChannel*>::iterator i=channelList.begin(); i!=channelList.end(); i++ ) {
      if ( ( ( *i )->getChannelName() ) == "ir0" )
        ir_sensors[0] = convertToByte ( ( *i )->getValue() );
      
      if ( ( ( *i )->getChannelName() ) == "ir1" )
        ir_sensors[5] = convertToByte ( ( *i )->getValue() );
      
      if ( ( ( *i )->getChannelName() ) == "ir2" )
        ir_sensors[2] = convertToByte ( ( *i )->getValue() );
      
      if ( ( ( *i )->getChannelName() ) == "ir3" )
        ir_sensors[3] = convertToByte ( ( *i )->getValue() );
      
      if ( ( ( *i )->getChannelName() ) == "ir4" )
        ir_sensors[4] = convertToByte ( ( *i )->getValue() );
      
      if ( ( ( *i )->getChannelName() ) == "ir5" )
        ir_sensors[1] = convertToByte ( ( *i )->getValue() );
      
      
      
      if ( ( ( *i )->getChannelName() ) == "ir6" )
        ir_sensors[9] = convertToByte ( ( *i )->getValue() );
      
      if ( ( ( *i )->getChannelName() ) == "ir7" )
        ir_sensors[10] = convertToByte ( ( *i )->getValue() );
      
      
      if ( ( ( *i )->getChannelName() ) == "ir8" )
        ir_sensors[6] = convertToByte ( ( *i )->getValue() );
      
      if ( ( ( *i )->getChannelName() ) == "ir9" )
        ir_sensors[11] = convertToByte ( ( *i )->getValue() );
      
      if ( ( ( *i )->getChannelName() ) == "ir10" )
        ir_sensors[8] = convertToByte ( ( *i )->getValue() );
    }
  
    update();
  }
  
  void SRIRSensorWidget::paintEvent ( QPaintEvent * )
  {
  
    QColor ir_color[NUMBER_IR_SENSORS];
  
    QPainter painter ( this );
    painter.setRenderHint ( QPainter::Antialiasing );
  
    //the outer shell of SphericalRobot
    painter.drawEllipse ( 10,10,this->width()-20, this->height()-20 );
    painter.drawEllipse ( 11,11,this->width()-22,this->height()-22 );
  
  
    //Axes of SphericalRobot
    painter.drawRect ( 10,this->height() /2-2,this->width()-20,2 );
    double v = ( this->height() /2 );
  
    painter.setPen ( Qt::gray );
    painter.drawEllipse ( 15,48,width()-30,40 );
    painter.drawEllipse ( 15,112,width()-30,40 );
    painter.drawRect ( width() /2-1,0,2,height() );
    painter.setPen ( Qt::black );
    //this->width()/2-(this->width()-10)/2, this->height() + (v/6+v/2)/2, this->width()-10,this->height() + (v/6+v/2)/2);
  
    /* order of the ir sensors
    *              11
    *          9      10
    *       6      7       8
    *  _________________________
    *  \    0      1     2 /
      *    \  4       3   /
    *       \      5     /
    *      \     /
    *         -
    */
  
  
    for ( int nr=0;nr<NUMBER_IR_SENSORS;nr++ ) {
      if ( ( ir_sensors[nr]>=0 ) && ( ir_sensors[nr]<128 ) )
        ir_color[nr].setRgb ( ir_sensors[nr]*2,255,0 );
      else if ( ( ir_sensors[nr]>=128 ) && ( ir_sensors[nr]<=255 ) )
        ir_color[nr].setRgb ( 255, ( 255-ir_sensors[nr] ) *2,0 );
      else ir_color[nr].setRgb ( 0,0,0 );
    }
  
    //width and height of the painted ir-sensors
    int ir_width=10;
    int ir_height=10;
  
  
    //1. IR-Sensor
    painter.setBrush ( ir_color[0] );
    painter.drawEllipse ( 20,this->height() /2+v/5+4,ir_width,ir_width );
    //2. IR-Sensor
    painter.setBrush ( ir_color[1] );
    painter.drawEllipse ( this->width() /2-ir_width/2,this->height() /2+v/6,ir_width,ir_height );
    //3. IR-Sensor
    painter.setBrush ( ir_color[2] );
    painter.drawEllipse ( this->width()-20-ir_width,this->height() /2+v/5+4,ir_width,ir_height );
    //4. IR-Sensor
    painter.setBrush ( ir_color[3] );
    painter.drawEllipse ( this->width() /2-v/3,this->height() /2+v/2-ir_height,ir_width,ir_height );
    //5. IR-Sensor
    painter.setBrush ( ir_color[4] );
    painter.drawEllipse ( this->width() /2+v/3-ir_width,this->height() /2+v/2-ir_height,ir_width,ir_height );
    //6. IR-Sensor
    painter.setBrush ( ir_color[5] );
    painter.drawEllipse ( this->width() /2-ir_width/2,this->height()-15-ir_height/2-2,ir_width,ir_height );
    //7. IR-Sensor
    painter.setBrush ( ir_color[6] );
    painter.drawEllipse ( 20,this->height() /2-v/5-ir_height-4,ir_width,ir_width );
    //8. IR-Sensor
    painter.setBrush ( ir_color[7] );
    painter.drawEllipse ( this->width() /2-ir_width/2,this->height() /2-v/6-ir_height+2,ir_width,ir_height );
    //9. IR-Sensor
    painter.setBrush ( ir_color[8] );
    painter.drawEllipse ( this->width()-20-ir_width,this->height() /2-v/5-ir_height-4,ir_width,ir_height );
    //10. IR-Sensor
    painter.setBrush ( ir_color[9] );
    painter.drawEllipse ( this->width() /2-v/3,this->height() /2-v/2,ir_width,ir_height );
    //11. IR-Sensor
    painter.setBrush ( ir_color[10] );
    painter.drawEllipse ( this->width() /2+v/3-ir_width,this->height() /2-v/2,ir_width,ir_height );
    //12. IR-Sensor
    painter.setBrush ( ir_color[11] );
    painter.drawEllipse ( this->width() /2-ir_width/2,15+2,ir_width,ir_height );
  }
  
  
  
  
  
  
  
  
  

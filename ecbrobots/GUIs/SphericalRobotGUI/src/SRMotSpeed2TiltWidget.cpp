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
#include "SRMotSpeed2TiltWidget.h"
#include <iostream>
#include <QTimer>

#include "IRPlotChannel.h"
#include "MotorCurrentPlotChannel.h"
#include "MotorSpeedPlotChannel.h"
#include "TiltPlotChannel.h"
#include "AxesPlotChannel.h"

#include <math.h>

#define MAX_MOTOR_VALUE 255
#define PI 3.141592

SRMotSpeed2TiltWidget::SRMotSpeed2TiltWidget ( QWidget *parent )
: SphericalRobotSubWidget ( parent )
{
  
  this->setMaximumSize ( 180,180 );
  this->setMinimumSize ( 180,180 );
  
  motor_arrow_color = QColor(20,20,100);
  tilt_arrow_color = QColor(220,20,40);
  
  QTimer *timer = new QTimer ( this );
  connect ( timer, SIGNAL ( timeout() ), this, SLOT ( updateViewableChannels() ) );
  timer->start ( 100 );
}

void SRMotSpeed2TiltWidget::updateViewableChannels()
{
  //std::cout << "SRMotorValueWidget: updateViewableChannels()" << std::endl;
  
  motorCurrent_sensorList.clear();
  motorSpeed_sensorList.clear();
  tilt_sensorList.clear();
  
  for ( std::list<AbstractPlotChannel*>::iterator i=channelList.begin(); i!=channelList.end(); i++ ) {
    
    ///Motor-Speed-Channel
    if ( dynamic_cast<MotorSpeedPlotChannel*> ( *i ) ) {
//       std::cout << "SRMotorValueWidget: add motorSpeed(" << (*i)->getChannelName() << ") = " << convertToByte((*i)->getValue()) << std::endl;
      double motorRspeedX, motorRspeedY, motorCspeedX, motorCspeedY;
      if ((*i)->getChannelName() == "motorRspeedX") motorRspeedX = (*i)->getValue();//motorSpeed_sensorList.push_back( (*i)->getValue() );
      if ((*i)->getChannelName() == "motorRspeedY") motorRspeedY = (*i)->getValue();//motorSpeed_sensorList.push_back( (*i)->getValue() );
      
      if ((*i)->getChannelName() == "motorCspeedX") motorCspeedX = (*i)->getValue();//motorSpeed_sensorList.push_back( (*i)->getValue() );
      if ((*i)->getChannelName() == "motorCspeedY") motorCspeedY = (*i)->getValue();//motorSpeed_sensorList.push_back( (*i)->getValue() );
      
      processMotorSpeedControllerArrowParams( motorCspeedY, motorCspeedX );
      processMotorSpeedRoboterArrowParams( motorRspeedY, motorRspeedX );      
      
    }
  }
  
 /* 
  if (motorSpeed_sensorList.size() >= 2) {
     processMotorArrowParams( motorSpeed_sensorList.front(), motorSpeed_sensorList.back() );
  }
  
//   if (motorCurrent_sensorList.size() >= 2) {
//   }
//     
   if (tilt_sensorList.size() >= 2) {
      processTiltArrowParams( tilt_sensorList.front(), tilt_sensorList.back() );
   }*/
  
  update();
}

void SRMotSpeed2TiltWidget::paintEvent ( QPaintEvent * )
{
  
  QPainter painter ( this );
  painter.setRenderHint ( QPainter::Antialiasing );
  
//the outer shell of SphericalRobot
  painter.drawEllipse ( 10,10,this->width()-20, this->height()-20 );
  painter.drawEllipse ( 11,11,this->width()-22,this->height()-22 );
  //Axes of SphericalRobot
  painter.drawRect ( 10,this->height() /2-2,this->width()-20,2 );
  
// set 0,0 to the middle of the QWidget itself
  painter.translate ( this->width() /2, this->height() /2 );
  
  painter.setOpacity ( 0.5 );
  
  
//   std::cout << "SRMotorValueWidget: motor_arrow_angle: (Byte)" << convertToByte(motor_arrow.angle) << std::endl;
  
// draw motor-arrow
  painter.setBrush ( motor_arrow_color );
  painter.save();
  painter.rotate ( (int)motor_arrow.angle);
  painter.drawConvexPolygon ( motor_arrow_part1,3 );
  painter.drawConvexPolygon ( motor_arrow_part2,4 );
  painter.restore();
  
  //draw tilt-arrow
  painter.setBrush ( tilt_arrow_color );
  painter.save();
  painter.rotate ( (int) tilt_arrow.angle );
  painter.drawConvexPolygon ( tilt_arrow_part1,3 );
  painter.drawConvexPolygon ( tilt_arrow_part2,4 );
  painter.restore();
  
  
  
}


void SRMotSpeed2TiltWidget::setMotorArrow()
{
  motor_arrow_part1[0] = QPoint ( -6,motor_arrow.len);
  motor_arrow_part1[1] = QPoint ( 6,motor_arrow.len );
  motor_arrow_part1[2] = QPoint ( 0,(motor_arrow.len)+10 );
  
  motor_arrow_part2[0] = QPoint ( -3,0 );
  motor_arrow_part2[1] = QPoint ( -3,(motor_arrow.len) );
  motor_arrow_part2[2] = QPoint ( 3,(motor_arrow.len) );
  motor_arrow_part2[3] = QPoint ( 3,0 );
}


void SRMotSpeed2TiltWidget::setTiltArrow()
{ 
  tilt_arrow_part1[0] = QPoint ( -6,(tilt_arrow.len) );
  tilt_arrow_part1[1] = QPoint ( 6,(tilt_arrow.len) );
  tilt_arrow_part1[2] = QPoint ( 0,(tilt_arrow.len)+10 );
  
  tilt_arrow_part2[0] = QPoint ( -3,0 );
  tilt_arrow_part2[1] = QPoint ( -3,(tilt_arrow.len) );
  tilt_arrow_part2[2] = QPoint ( 3,(tilt_arrow.len) );
  tilt_arrow_part2[3] = QPoint ( 3,0 );
}


void SRMotSpeed2TiltWidget::processMotorSpeedControllerArrowParams(double m1, double m2)
{
  
//   std::cout << "SRMotorValueWidget: m1: " << m1 << " m2: " << m2 << std::endl;
  //bottom-view of spherical robot:
  //moves of m1 => x-axes (
  //moves of m2 => y-axes
//   int tmp_alpha =0;
//   double tmp = 0;
//   
//   if ( ( m1>=0 ) && ( m2>0 ) ) {
//     tmp_alpha=180;  //1. Quadrant
//     tmp = m1 / m2;
//   } else if ( ( m1<0 ) && ( m2>=0 ) ) {
//     tmp_alpha=90; //2
//     tmp = m2 / -m1;
//   } else if ( ( m1<=0 ) && ( m2<0 ) ) {
//     tmp_alpha=0; //3.
//     tmp = m1 / m2;
//   } else if ( ( m1>0 ) && ( m2<=0 ) ) {
//     tmp_alpha=270;   //4.
//     tmp = -m2 / m1;
//   }
  
  motor_arrow.len = hypot(m1,m2)*150;
  motor_arrow.angle = ( ( atan2( m2,m1)) * 180 / PI ) + 45.+ 180.  ;
//   printf("SRMotorValueWidget: angle: %f len: %f\r\n",motor_arrow.angle,motor_arrow.len);
  // atan is in rad modus (*180/PI for dec modus)
//   motor_arrow.angle = ( atan ( tmp ) * 180 / PI + tmp_alpha );
//   motor_arrow.angle = ( atan2 (m1,m2) * 180 / PI + tmp_alpha );
  // scale len into viewable size
//   motor_arrow.len = ( (sqrt ( pow ( m1,2. ) + pow ( m2,2. ) ) )) / 180. * this->height() /2;
  
  setMotorArrow();
}

void SRMotSpeed2TiltWidget::processMotorSpeedRoboterArrowParams( double t1, double t2 )
{
//   std::cout << "SRMotorValueWidget: t1: " << t1 << " t2: " << t2 << std::endl;
  //bottom-view of spherical robot:
  //moves of m1 => x-axes
  //moves of m2 => y-axes
//   int tmp_alpha =0;
//   double tmp = 0;
//   
//   if ( ( t1>=0 ) && ( t2>0 ) ) {
//     tmp_alpha=180;  //1. Quadrant
//     tmp = ( double ) t1 / ( double ) t2;
//   } else if ( ( t1<0 ) && ( t2>=0 ) ) {
//     tmp_alpha=90; //2
//     tmp = ( double ) t2 / ( double )-t1;
//   } else if ( ( t1<=0 ) && ( t2<0 ) ) {
//     tmp_alpha=0; //3.
//     tmp = ( double ) t1 / ( double ) t2;
//   } else if ( ( t1>0 ) && ( t2<=0 ) ) {
//     tmp_alpha=270;   //4.
//     tmp = ( double )-t2 / ( double ) t1;
//   }
  
  tilt_arrow.len = hypot(t1,t2)*80;
    // scale len into viewable size
  tilt_arrow.angle = ( ( atan2( t2,t1)) * 180 / PI ) + 90.+180.-45.;
  setTiltArrow ();
  
}




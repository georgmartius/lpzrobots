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
#include "SRMotorValueWidget.h"
#include <iostream>

SRMotorValueWidget::SRMotorValueWidget(QWidget *parent)
: SphericalRobotSubWidget(parent) {

setArrowParam(0,0);

}


void SRMotorValueWidget::paintEvent(QPaintEvent *) {


QColor motor_arrow_color(20,20,100);
QColor tilt_arrow_color(220,20,40);
QColor ir_color[12];

QPainter painter(this);
painter.setRenderHint(QPainter::Antialiasing);

//the outer shell of SphericalRobot 
painter.drawEllipse(10,10,this->width()-20, this->height()-20);
painter.drawEllipse(11,11,this->width()-22,this->height()-22);

//Axes of SphericalRobot
painter.drawRect(10,this->height()/2-2,this->width()-20,2);

painter.setOpacity(0.5);
painter.setBrush(motor_arrow_color);
painter.translate(this->width()/2, this->height()/2);


painter.save();
painter.rotate(this->alpha);
painter.drawConvexPolygon(motor_arrow_part1,3);
painter.drawConvexPolygon(motor_arrow_part2,4);
painter.restore();

painter.setBrush(tilt_arrow_color);
painter.save();
painter.rotate(this->alpha-90);
painter.drawConvexPolygon(motor_arrow_part1,3);
painter.drawConvexPolygon(motor_arrow_part2,4);
painter.restore();

}

void SRMotorValueWidget::setArrowParam(int alpha, int len) {
  this->alpha = alpha;
  this->len = len;

  motor_arrow_part1[0] = QPoint(-6,len);
  motor_arrow_part1[1] = QPoint(6,len);
  motor_arrow_part1[2] = QPoint(0,len+10);

  motor_arrow_part2[0] = QPoint(-3,0);
  motor_arrow_part2[1] = QPoint(-3,len);
  motor_arrow_part2[2] = QPoint(3,len);
  motor_arrow_part2[3] = QPoint(3,0);

  update();
}




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
#ifndef SR_MOTSPEED2TILT_WIDGET_H
#define SR_MOTSPEED2TILT_WIDGET_H

#include "SphericalRobotSubWidget.h"

#include <QColor>
#include <QPoint>

typedef struct {
  double angle;
  double len;
} arrow_params2_t;



class SRMotSpeed2TiltWidget : public SphericalRobotSubWidget {
  
  Q_OBJECT
    
public:
  SRMotSpeed2TiltWidget(QWidget *parent = 0);
  ~SRMotSpeed2TiltWidget(){};
  
public slots:
  void updateViewableChannels();
  
protected:
  void paintEvent(QPaintEvent *);
  
  void updateMotorTiltArrows();
  void setMotorArrow();
  void setTiltArrow();
  void processMotorSpeedControllerArrowParams(double m1, double m2);
  void processMotorSpeedRoboterArrowParams(double t1, double t2);
  
private:
//   int* data[12];
//   int datalen;
//   int alpha;
//   int len;
#define motor_arrow mot_arrow
#define tilt_arrow t_arrow
  arrow_params2_t motor_arrow;
  arrow_params2_t tilt_arrow;
  
  QPoint motor_arrow_part1[3];
  QPoint motor_arrow_part2[4];
  QPoint tilt_arrow_part1[3];
  QPoint tilt_arrow_part2[4];
  
  QColor ir_color[12];
  QColor motor_arrow_color;
  QColor tilt_arrow_color;
  
  std::list<double> tilt_sensorList;
  std::list<double> motorSpeed_sensorList;
  std::list<double> motorCurrent_sensorList;
  
  
};

#endif

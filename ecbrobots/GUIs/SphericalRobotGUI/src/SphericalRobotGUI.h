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


#ifndef SPHERICAL_ROBOT_GUI_H
#define SPHERICAL_ROBOT_GUI_H

#include <QWidget>
#include <QPainter>
#include <QPixmap>
#include <QPoint>

#include "SphericalRobotSubWidget.h"
#include <math.h>
#include "gui-test.h"


class SphericalRobotGUI : public QWidget {

Q_OBJECT

public:
SphericalRobotGUI(QWidget *parent = 0);
void setArrowParams(int m1, int m2);

public slots:
void setArrowParamX(int);
void setArrowParamY(int);

signals:
void valueChanged(int);

protected:
void resizeEvent(QResizeEvent *event);
void paintEvent(QPaintEvent *event);

private:
Ui::SphericalRobotGUI ui;
QPixmap pmap;
SphericalRobotSubWidget *subw;
SphericalRobotSubWidget *subw1;
SphericalRobotSubWidget *subw2;

};
#endif



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

#include <QtGui>

//#include <QHBoxLayout>

#include <iostream>
#include "SphericalRobotGUI.h"

#define MAX_MOTOR_VALUE 255
#define PI 3.141592

SphericalRobotGUI::SphericalRobotGUI(QWidget *parent) 
: QWidget(parent) {



//ui.setupUi(this);

subw = new SphericalRobotSubWidget(this);
subw->setMaximumSize(200,200);

// subw1 = new SphericalRobotSubWidget(this);
// subw1->setMaximumSize(160,160);
// 
// subw2 = new SphericalRobotSubWidget(this);
// subw2->setMaximumSize(120,120);

QSpinBox *motor1_value = new QSpinBox;
motor1_value->setRange(-127,128);
motor1_value->setSingleStep(1);
motor1_value->setValue(0);

QSpinBox *motor2_value = new QSpinBox;
motor2_value->setRange(-127,128);
motor2_value->setSingleStep(1);
motor2_value->setValue(0);


QObject::connect(motor1_value, SIGNAL(valueChanged(int)),
		this,SLOT(setArrowParamX(int)));
QObject::connect(motor2_value, SIGNAL(valueChanged(int)),
		this,SLOT(setArrowParamY(int)));


QHBoxLayout *layout = new QHBoxLayout;
layout->addWidget(subw);
// layout->addWidget(subw1);
// layout->addWidget(subw2);
layout->addWidget(motor1_value);
layout->addWidget(motor2_value);

setLayout(layout);

resize(800,500);

}

int X,Y;

void SphericalRobotGUI::setArrowParamX(int x) {

/*int f = (int)((double)x/(double)MAX_MOTOR_VALUE)*subw->width();
*/
setArrowParams(x,Y);

X=x;
}

void SphericalRobotGUI::setArrowParamY(int y) {
/*
int f2 = (int)((double)y/(double)MAX_MOTOR_VALUE)*subw->height();
*/
setArrowParams(X, y);

Y=y;
}
void SphericalRobotGUI::setArrowParams(int m1, int m2) {

//bottom-view
  //m1 is x
  //m2 is y
int tmp_alpha =0;
double tmp = 0;
     if ((m1>=0)&&(m2>0)) {
	tmp_alpha=180;  //1. Quadrant
	tmp = (double)m1 / (double)m2;
     }
else if ((m1<0)&&(m2>=0)) {
	tmp_alpha=90; //2
     	tmp = (double)m2 / (double)-m1;
     }
else if ((m1<=0)&&(m2<0)) {
	tmp_alpha=0; //3.
	tmp = (double)m1 / (double)m2;
     }
else if ((m1>0)&&(m2<=0)) {
	tmp_alpha=270;   //4.
    	tmp = (double)-m2 / (double)m1;
     }

double alpha =(atan(tmp) * 180 / PI + tmp_alpha);
double len = sqrt(pow((double)m1,2.)+pow((double)m2,2.));

// scale len into viewable size
// 180 = max number for len, if m1 and m2 max by 128
len = len / 180. * (double)subw->height()/2;
subw->setArrowParam( alpha,len );

}


void SphericalRobotGUI::resizeEvent(QResizeEvent *event) {

event->accept();

//QPixmap new_pmap(event->size());

pmap.fill(Qt::blue);

//pmap = new_pmap;

//update();



}


void SphericalRobotGUI::paintEvent(QPaintEvent *event) {

// event->accept();
// 
// QPainter painter(this);
// 
// painter.fillRect(100,100,100,100, Qt::red);
// 
// painter.drawPixmap(QPoint(0,0), pmap);


}


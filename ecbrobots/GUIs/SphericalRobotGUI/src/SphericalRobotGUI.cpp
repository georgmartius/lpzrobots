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
#include <QProgressBar>
//#include <QHBoxLayout>

#include "SphericalRobotGUI.h"

#define MAX_MOTOR_VALUE 255
#define PI 3.141592

#include "AbstractPipeReader.h"
#include "SimplePipeReader.h"

SphericalRobotGUI::SphericalRobotGUI(QWidget *parent) 
: QWidget(parent) {
  
  pipeReader = new SimplePipeReader(std::cin);

  pipeReader-> getDataHeaderInformation();
  
//ui.setupUi(this);

/* only for testing ******************************/
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

/***********************************************/

main_layout = new QVBoxLayout;
main_layout->addWidget(motor1_value);
main_layout->addWidget(motor2_value);


main_layout->addWidget(createControlBox());
main_layout->addWidget(createGraphicalBox());

QGroupBox *gb = createIRSensorBox();
gb->setMaximumSize(350,120);
main_layout->addWidget(gb);

setLayout(main_layout);
resize(800,800);

}

QGroupBox* SphericalRobotGUI::createControlBox() {

QGroupBox *gbox = new QGroupBox(QString("Simulationcontroller"));
QGridLayout *l = new QGridLayout;

startButton = new QPushButton("Play");
stopButton = new QPushButton("Stop");
backwardButton = new QPushButton("<<");
forwardButton = new QPushButton(">>");
loadFileButton = new QPushButton("Open");

l->addWidget(startButton,0,1);
l->addWidget(stopButton,0,3);
l->addWidget(forwardButton,0,2);
l->addWidget(backwardButton,0,0);
l->addWidget(loadFileButton,0,4);


gbox->setLayout(l);
return gbox;
}

QGroupBox* SphericalRobotGUI::createGraphicalBox() {

QGroupBox *gbox = new QGroupBox(QString("Graphical View"));
QHBoxLayout *l = new QHBoxLayout;

subw = new SRMotorValueWidget(this);
subw->setMinimumSize(200,200);
subw->setMaximumSize(200,200);


subw1 = new SRIRSensorWidget(this);
subw1->setMinimumSize(160,160);
subw1->setMaximumSize(200,200);



l->addWidget(subw);
l->addWidget(subw1);

gbox->setLayout(l);
return gbox;
}

QGroupBox* SphericalRobotGUI::createIRSensorBox() {

QGroupBox *gbox = new QGroupBox(QString("IR-Sensors"));
QGridLayout *l = new QGridLayout;

for(int i=0;i<NUMBER_IR_SENSORS;i++) {

  ir_labels[i] = new QLabel(QString("IR%1").arg(i+1));
  ir_progressBar[i] = new QProgressBar(this);
  ir_progressBar[i]->setRange(0,255); 
  ir_progressBar[i]->setOrientation(Qt::Vertical);
  ir_progressBar[i]->setFormat(QString("%v"));
ir_progressBar[i]->setValue(100);
l->addWidget(new QLabel(QString("IR%1").arg(i+1)),0,i);
  l->addWidget(ir_progressBar[i],1,i);
  l->addWidget(ir_labels[i],2,i); 
  
}
gbox->setLayout(l);
return gbox;
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


}

/*
void SphericalRobotGUI::paintEvent(QPaintEvent *event) {


}
*/

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

// #include <iostream>

#include "SphericalRobotGUI.h"

#include <list>


SphericalRobotGUI::SphericalRobotGUI(QWidget *parent) 
: QWidget(parent) {
  
//   SimplePipeReader *spr = new SimplePipeReader();
  pipe_reader = new SimplePipeReader();
  srgui_filter = new SRGUIPipeFilter(pipe_reader);
  
  //init graphical interface and motor_tilt_widget, ir_widget
  initGui();
  
  pipe_reader->start();
  
//link  the interesting channels to the widgets
  linkChannels();

  QObject::connect(pipe_reader,SIGNAL(finished()),this,SLOT(closeGUI()));
}



void SphericalRobotGUI::linkChannels() 
{
  std::cout << "SphericalRobotGUI: linkChannels()"<< std::endl;
  // create ChannelList and start createChannelList() in class SRGUIPipeFilter
  std::list<AbstractPlotChannel*> channelList = srgui_filter->getChannelList();
  
  for (std::list<AbstractPlotChannel*>::iterator i=channelList.begin(); i!=channelList.end(); i++)
  {
    //Infrared-Sensor-Channel
    if (dynamic_cast<IRPlotChannel*> (*i))
    {
      ir_widget->addPlotChannel(*i);
      std::cout << "IRPlotChannel: " << (*i)->getChannelName() << std::endl;
    }
    ///Motor-Speed-Channel
    if (dynamic_cast<MotorSpeedPlotChannel*> (*i))
    {
      motor_tilt_widget->addPlotChannel(*i);
      motspeed2tilt_widget->addPlotChannel(*i);
    }
    ///Motor-Current-Channel
    if (dynamic_cast<MotorCurrentPlotChannel*>(*i))
    {
      motor_tilt_widget->addPlotChannel(*i);
    }
    ///IR-Sensors-Channel-->that are looking on the axes of SphericalRobot
    if (dynamic_cast<AxesPlotChannel*>(*i))
    {
      motor_tilt_widget->addPlotChannel(*i);
    }
    ///Tilt-Sensor-Channel
    if (dynamic_cast<TiltPlotChannel*>(*i))
    {
      motor_tilt_widget->addPlotChannel(*i);
    }
    ///Timestamp-Channel
    if (dynamic_cast<TimeStampPlotChannel*>(*i))
    {
    }
    //Default-Channel
    if (dynamic_cast<DefaultPlotChannel*>(*i))
    {
    }

/* 
    if (((*i)->getChannelName()).find("blabla-name")==0)
      ...
*/

  }
}

void SphericalRobotGUI::initGui()
{
  std::cout << "SphericalRobotGUI: initGui()"<< std::endl;
  ir_widget = new SRIRSensorWidget();
  motor_tilt_widget = new SRMotorValueWidget();
  motspeed2tilt_widget = new SRMotSpeed2TiltWidget();
  
  main_layout = new QVBoxLayout;
  main_layout->addWidget(createControlBox());
  
  QGroupBox *gb = new QGroupBox(QString("Graphical-View"));
  QHBoxLayout *graph_box = new QHBoxLayout;
  graph_box->addWidget(motor_tilt_widget);
  graph_box->addWidget(motspeed2tilt_widget);
  graph_box->addWidget(ir_widget);
  gb->setLayout(graph_box);
  
  main_layout->addWidget(gb);  
  setLayout(main_layout);
  resize(200,200);
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

void SphericalRobotGUI::resizeEvent(QResizeEvent *event) {
  event->accept();
}

QGroupBox* SphericalRobotGUI::createIRProgressBar() {
  QGroupBox *gbox = new QGroupBox(QString("Infrarot-Sensors-View"));
  QGridLayout *gl = new QGridLayout;
  
  for(int i=0;i<NUMBER_IR_SENSORS;i++) {
    ir_labels[i] = new QLabel(QString("IR%1").arg(i+1));
    ir_progressBar[i] = new QProgressBar(this);
    ir_progressBar[i]->setRange(0,255); 
    ir_progressBar[i]->setOrientation(Qt::Vertical);
  //   ir_progressBar[i]->setFormat(QString("%v"));
  //   ir_progressBar[i]->setValue(100);
    
    gl->addWidget(ir_progressBar[i],0,i);
    gl->addWidget(ir_labels[i],1,i);
    gl->addWidget(new QLabel(QString("HHH%l").arg(i+1)),2,i);
  } 
  gbox->setLayout(gl);
  return gbox;
}

void SphericalRobotGUI::closeGUI()
{
  std::cout << "SphericalRobotGUI: ByeBye!" << std::endl;
//   delete(this);
}


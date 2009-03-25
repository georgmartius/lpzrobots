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

/*
#include <QWidget>
#include <QPainter>
#include <QPixmap>
#include <QPoint>
#include <QHBoxLayout>*/
#include <QtGui>

#include "SRMotorValueWidget.h"
#include "SRMotSpeed2TiltWidget.h"
#include "SRIRSensorWidget.h"
#include "SRGUIPipeFilter.h"

#include "SimplePipeReader.h"
#include "BufferedPipeReader.h"
#include "AbstractPipeReader.h"


#include "TiltPlotChannel.h"
#include "AxesPlotChannel.h"
#include "MotorCurrentPlotChannel.h"
#include "MotorSpeedPlotChannel.h"
#include "AbstractPlotChannel.h"
#include "IRPlotChannel.h"
#include "TimeStampPlotChannel.h"
#include "DefaultPlotChannel.h"




// #include "gui-test.h"


class SphericalRobotGUI : public QWidget {

Q_OBJECT

public:
  SphericalRobotGUI(QWidget *parent = 0);

protected:
  void resizeEvent(QResizeEvent *event);

public slots:
  
  void closeGUI();
  
private:
//   Ui::SphericalRobotGUI ui;
  
  AbstractPipeReader* pipe_reader;
  SRGUIPipeFilter* srgui_filter;
  
  class SRMotSpeed2TiltWidget* motspeed2tilt_widget;
  class SRMotorValueWidget* motor_tilt_widget;
  class SRIRSensorWidget* ir_widget;

  void initGui();
  void linkChannels();
  
  QVBoxLayout *main_layout;
  QPushButton *startButton;
  QPushButton *stopButton;
  QPushButton *backwardButton;
  QPushButton *forwardButton;
  QPushButton *loadFileButton;
  
  QGroupBox* createControlBox();
  QGroupBox* createIRProgressBar();

  QLabel *ir_labels[NUMBER_IR_SENSORS];
  QProgressBar *ir_progressBar[NUMBER_IR_SENSORS];
  
};
#endif



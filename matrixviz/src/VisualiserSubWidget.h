/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Antonia Siegert (original author)                                  *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 *                                                                         *
 ***************************************************************************/

#ifndef __VISUALISATIONSUBWIDGET_H_
#define __VISUALISATIONSUBWIDGET_H_

#include <QtGui>
//class AbstractPlotChannel;

//#include "Channel/MatrixPlotChannel.h"
#include "Channel/VectorPlotChannel.h"
#include "ColorPalette.h"
#include "visualisations/AbstractVisualisation.h" //Abstract~

#include "AbstractRobotSubWidget.h"


#include <iostream>

// class CaptureFrameEvent : public QEvent {
// public:
//   CaptureFrameEvent(long idx, QString directory)
//     : QEvent((Type)type), idx(idx), directory(directory) {

//   }
//   static const int type = QEvent::User+9891;
// public:
//   long idx;
//   QString directory;
// };


class VisualiserSubWidget: public AbstractRobotSubWidget {

Q_OBJECT

public:
  VisualiserSubWidget(MatrixPlotChannel *channel, int x = 0, int y = 0, int width = 0, int heigt = 0,
      QString cPFilePath = "", QWidget *parent = 0);
  virtual ~VisualiserSubWidget();
  QString getChannelName();
  QString getColorPaletteFilepath();
  int getVisMode();
  QString getMode();
  QSize getSize();


public slots:
  void updateViewableChannels();
  void captureFrame(long idx, QString directory);
  void sourceName(QString name);
  void switchVisMode(int index);
  void switchVisMode(QAction *action);
  void toggleOptions(QAction *action = 0);
  void changeBufferSize();

protected:

  QComboBox *vizChoice;
  virtual void closeEvent(QCloseEvent * event);
  //  virtual bool event(QEvent* event);
private:

  AbstractVisualisation *visualisation;
  QHBoxLayout *visLayout;
  QMenuBar *menuBar;
  QVBoxLayout *optionLayout;
  QWidget *optionWidget;

  MatrixPlotChannel *matrixChannel;
  ColorPalette *colorPalette;
  QString srcName;

  void initGui();
  void initVisTypes();
  void addVectorOptions();
  int visMode;
  bool optionsShown;
  static const bool debug = false;

signals:
  void windowClosed(VisualiserSubWidget* window);
  void sendQuit();

private:
  QSpinBox *vectorBuffersizeSpinBox;

};

#endif /* __VISUALISATIONSUBWIDGET_H_ */

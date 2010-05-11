/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    mam06fyl@studserv.uni-leipzig.de (robot14)                           *
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
 *   DESCRIPTION                                                           *
 *                                                                         *
 *   Visualization tool for matrices...                                    *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#ifndef __COLORPALETTE_H_
#define __COLORPALETTE_H_

#include <QString>
#include <QtGui>
#include <QSize>
#include <QVector>
#include "MatrixPlotChannel.h"

struct STOP {
    QColor color;
    double pos;
    bool operator <(const STOP& rhs) { // for sorting
      return pos < rhs.pos;
    }
    bool operator >(const STOP& rhs) { // for sorting
      return pos > rhs.pos;
    }
};

class ColorPalette : public QWidget{

  Q_OBJECT

public:
  ColorPalette(QWidget *parent = NULL);
  ~ColorPalette();

  QColor pickColor(double val);
  QVector<STOP*> *stops;

  bool autoUpdate; //sets the min and max depending on the input
  void addStop(int num, QRgb color, double pos);  //<-- ColorChooser
  void deleteStop(int num);
  QWidget* makeConfigBox();
  QString getPath();
  void loadStopListFromFile(QString filename);
  double getNextStopPosition(double fromVal, double toVal);

public slots:
  void addStop(int i);
  void removeStop(int i);
  void changeStopColor(int i, QColor color);
  void changeStopPos(int i, double pos);
  void loadStopList();
  void saveStopList();
  void setMax(const QString &text);
  void setMin(const QString &text);
  void autoSetMinMax();
//  void setFunction(int i);

protected:
  double max, min;
  void paintEvent(QPaintEvent *);
  void resizeEvent(QResizeEvent *event);
//  void mouseDoubleClickEvent(QMouseEvent *);
  void mouseMoveEvent ( QMouseEvent *event ); // enabled with QWidget::setMouseTracking(true)
  virtual void closeEvent(QCloseEvent * event);
  const static bool debug = true;

private:
  void initMaxMin();

  void updateList();

  QListWidget *stopList;
  QPushButton *add;
  QPushButton *remove;
  QString currentPath;
  QLineEdit *minEdit;
  QLineEdit *maxEdit;
//  int currentFunction;

signals:
  void sendQuit();
};

#endif /* __COLORPALETTE_H_ */

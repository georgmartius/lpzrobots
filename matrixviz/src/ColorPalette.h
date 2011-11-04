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

#ifndef __COLORPALETTE_H_
#define __COLORPALETTE_H_

#include <QString>
#include <QtGui>
#include <QSize>
#include <QVector>
#include "MatrixPlotChannel.h"
#include "ScaleFunction.h"

#define COMPARE_EPS 1e-5  //GLfloat

struct STOP {
  STOP() {}
  STOP( const QColor& color, double pos)
    : color(color), pos(pos) {};
  QColor color;
  double pos;
  bool operator <(const STOP& rhs) const { // for sorting
    return pos < rhs.pos;
  }
  bool operator ==(const STOP& rhs) const { // for sorting
    return pos == rhs.pos;
  }
};

class ColorPalette : public QWidget{

  Q_OBJECT

public:
  ColorPalette(QWidget *parent = NULL);
  ~ColorPalette();

  QColor pickColor(double val);
  QColor pickScaledColor(double val);
  double getScaledValue(double val);
  QVector<STOP> stops;

  void addStop(int num, QRgb color, double pos);
  void deleteStop(int num);
  QWidget* makeConfigBox();
  QString getPath();
  void loadStopListFromFile(QString filename);
  double getNextStopPosition(double fromVal, double toVal);
  double getMax();
  double getMin();

public slots:
  void addStop(int i);
  void removeStop(int i);
  void changeStopColor(int i,const QColor& color);
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
  void mouseMoveEvent ( QMouseEvent *event ); // enabled with QWidget::setMouseTracking(true)
  virtual void closeEvent(QCloseEvent * event);
  const static bool debug = false;

private:
  void initMaxMin();

  void updateList();

  QListWidget *stopList;
  QPushButton *add;
  QPushButton *remove;
  QString currentPath;
  QLineEdit *minEdit;
  QLineEdit *maxEdit;
  ScaleFunction *scaleF;

  inline bool equals(double x, double y)
  {
    double diff = x-y;
    if ((diff <= COMPARE_EPS) && (-COMPARE_EPS <= diff))
      return true;
    return false;
  }

signals:
  void sendQuit();
};

#endif /* __COLORPALETTE_H_ */

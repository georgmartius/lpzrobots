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

#ifndef __CONFIGFILE_H_
#define __CONFIGFILE_H_

#include <QtXml/QDomDocument>
#include <QtXml/QDomElement>
#include <QFile>
#include <QString>
#include <QList>
#include <MatrixVisualizer.h>
#include <VisualiserSubWidget.h>

class MatrixVisualizer;

class configFile : public QObject{

Q_OBJECT

public:
  configFile();
  ~configFile();
  void load( MatrixVisualizer* mv);
  void save();
  void newOpenedWindow(VisualiserSubWidget* window);
  static const bool debug = false;
  QList<VisualiserSubWidget*> getOpenWindows() {return openWindows;}

public slots:
  void doQuit();
  void windowClosed(VisualiserSubWidget* window);

private:
  QList<VisualiserSubWidget*> openWindows;
  MatrixVisualizer* matrixVis;
  bool saved; //prevents double saving

signals:
  void sendQuit();
};

#endif /* __CONFIGFILE_H_ */

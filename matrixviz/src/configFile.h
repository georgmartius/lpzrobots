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
 *   $Log$
 *   Revision 1.2  2010-06-30 11:36:59  robot14
 *   removed typo
 *
 *   Revision 1.1  2010/03/30 13:19:09  robot14
 *   first version
 *
 *
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

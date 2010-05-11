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

#ifndef __ABSTRACTVISUALISATION_H_
#define __ABSTRACTVISUALISATION_H_

class AbstractPlotChannel;

#include "MatrixPlotChannel.h"
#include "VectorPlotChannel.h"
#include "ColorPalette.h"
#include <qgl.h> //"/usr/include/qt4/QtOpenGL/QGLWidget"

#include <iostream>


class AbstractVisualisation: public QGLWidget {

  Q_OBJECT

public:
  AbstractVisualisation(MatrixPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent = 0);
  AbstractVisualisation(VectorPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent = 0);
  virtual ~AbstractVisualisation();

protected:

  MatrixPlotChannel *matrixChannel;
  VectorPlotChannel *vectorChannel;
  ColorPalette *colorPalette;


  virtual void initializeGL() = 0;
  virtual void resizeGL(int w, int h) = 0;
  virtual void paintGL() = 0;

private:

};


#endif /* __ABSTRACTVISUALISATION_H_ */

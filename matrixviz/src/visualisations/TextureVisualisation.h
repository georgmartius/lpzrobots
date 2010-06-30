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
 *  $Log$
 *  Revision 1.3  2010-06-30 11:31:11  robot14
 *  VectorPlotChannel specs removed
 *
 *                                                                         *
 ***************************************************************************/

#ifndef __TEXTUREVISUALISATION_H_
#define __TEXTUREVISUALISATION_H_


#include "AbstractVisualisation.h"
#include <qgl.h> //"/usr/include/qt4/QtOpenGL/QGLWidget"
//#include "../ColorPalette.h"
//#include "../Channel/VectorPlotChannel.h"

class TextureVisualisation: public AbstractVisualisation {
  Q_OBJECT

public:
  TextureVisualisation(MatrixPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent = 0);
//  TextureVisualisation(VectorPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent = 0);
  virtual ~TextureVisualisation();
  //void updateView();


protected:
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();
  virtual GLuint   makeObject();
  void mouseMoveEvent ( QMouseEvent *event );


private:
  GLuint object;
  GLuint texName;
  GLubyte tex[64][64][3];
  int maxX, maxY;
  const static bool debug = false;

  double clip(double val);
};


#endif /* __TEXTUREVISUALISATION_H_ */

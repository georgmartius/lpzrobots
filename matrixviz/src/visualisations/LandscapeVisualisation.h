 /***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.3  2010-06-30 11:31:11  robot14
 *  VectorPlotChannel specs removed
 *
 *                       *
 *                                                                         *
 **************************************************************************/
#ifndef __LANDSCAPEVISUALISATION_H_
#define __LANDSCAPEVISUALISATION_H_

#include "AbstractVisualisation.h"
//#include <GL/glut.h>

#include <qgl.h>

struct VERTEX {
    GLfloat x,y,z;
    VERTEX() {};
    VERTEX( GLfloat x, GLfloat y, GLfloat z): x(x), y(y), z(z) {};
};

class LandscapeVisualisation : public AbstractVisualisation {
  Q_OBJECT

public:
  LandscapeVisualisation(MatrixPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent = 0);
//  LandscapeVisualisation(VectorPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent = 0);
   ~LandscapeVisualisation();

protected:

   void initializeGL();
   void resizeGL(int w, int h);
   void paintGL();
   void divideAndDrawTriangle(VERTEX* v1, VERTEX* v2, VERTEX* v3, VERTEX* n = 0);
   void drawTriangle(VERTEX* v1, VERTEX* v2, VERTEX* v3, VERTEX* n = 0);
   VERTEX* getVertexBetween(VERTEX* v1, VERTEX* v2, double pos);
   void mouseMoveEvent ( QMouseEvent *event ); // TODO mousePressed...
   void wheelEvent(QWheelEvent * event);
   void mousePressEvent ( QMouseEvent *event );

private:
   GLuint object;
   int maxX, maxY;
   float rotX, rotY;
   double plateauRadius;
   GLfloat zoom;
   int mouseX, mouseY;
   const static bool debug = false;

   double clip(double val);
   VERTEX* getNormal(VERTEX* v1, VERTEX* v2, VERTEX* v3);
};

#endif /* __LANDSCAPEVISUALISATION_H_ */

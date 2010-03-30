/*
 * LandscapeVisualisation.h
 *
 *  Created on: 20.01.2010
 *      Author: oni
 */

#ifndef __LANDSCAPEVISUALISATION_H_
#define __LANDSCAPEVISUALISATION_H_

#include "../Channel/MatrixPlotChannel.h"
//#include "AbstractVisualisation.h"
#include "../ColorPalette.h"
//#include <GL/glut.h>

#include <qgl.h> //"/usr/include/qt4/QtOpenGL/QGLWidget"

class LandscapeVisualisation : public QGLWidget {
  Q_OBJECT

public:
  LandscapeVisualisation(MatrixPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent = 0);
   ~LandscapeVisualisation();

protected:

   MatrixPlotChannel *channel;
   void initializeGL();
   void resizeGL(int w, int h);
   void paintGL();
   virtual GLuint   makeObject();
   void mouseMoveEvent ( QMouseEvent *event ); // TODO mousePressed...
   void mousePressEvent ( QMouseEvent *event );

private:
   ColorPalette *colorPalette;
   GLuint object;
//   GLuint texName;
//   GLubyte tex[64][64][3];
   int maxX, maxY;
   int visMode; // 0 = landscape 1 = bar
   int inputMode; // 0 = matrix 1 = vector(s)
   float rotX, rotY;
   int mouseX, mouseY;
   const static bool debug = true;
};

#endif /* __LANDSCAPEVISUALISATION_H_ */

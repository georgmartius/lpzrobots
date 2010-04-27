/*
 * BarVisualisation.h
 *
 *  Created on: 21.04.2010
 *      Author: oni
 */

#ifndef __BARVISUALISATION_H_
#define __BARVISUALISATION_H_

#include "AbstractVisualisation.h"


#include <qgl.h> //"/usr/include/qt4/QtOpenGL/QGLWidget"

class BarVisualisation : public AbstractVisualisation {
  Q_OBJECT

public:
  BarVisualisation(MatrixPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent = 0);
  BarVisualisation(VectorPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent = 0);
   ~BarVisualisation();

protected:

   void initializeGL();
   void resizeGL(int w, int h);
   void paintGL();
   virtual GLuint   makeObject();
   void mouseMoveEvent ( QMouseEvent *event ); // TODO mousePressed...
   void wheelEvent(QWheelEvent * event);
   void mousePressEvent ( QMouseEvent *event );

private:
   GLuint object;
   int maxX, maxY;
   int visMode; // 0 = landscape 1 = bar
   int inputMode; // 0 = matrix 1 = vector(s)
   float rotX, rotY;
   GLfloat zoom;
   int mouseX, mouseY;
   const static bool debug = true;
   void drawBar(double value);
};


#endif /* __BARVISUALISATION_H_ */

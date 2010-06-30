/*
 * VectorPlotVisualisation.cpp
 *
 *  Created on: 29.04.2010
 *      Author: oni
 */

#include "VectorPlotVisualisation.h"
#include "math.h"
#include <iostream>
#include <string>


using namespace std;

VectorPlotVisualisation::VectorPlotVisualisation(MatrixPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent)
: AbstractVisualisation(channel, colorPalette, parent){

  /*
   * not specified for matrices!!
   */
  if(debug) cout << "VectorPlotVisualisation Konstruktor" << endl;
  this->channel = channel;
  this->colorPalette = colorPalette;
  maxX = channel->getDimension(0);
  maxY = channel->getDimension(1);
  //setUpdatesEnabled(true);
  setMouseTracking(true); // enables tooltips while mousemoving over widget
}

VectorPlotVisualisation::~VectorPlotVisualisation(){
  if(debug) cout << "VectorPlotVisualisation Destruktor" << endl;
  makeCurrent();
}

void VectorPlotVisualisation::initializeGL(){
  if(debug) cout << "VectorPlotVisualisation Konstruktor" << endl;
  qglClearColor( Qt::black);    // Let OpenGL clear to black
  glShadeModel( GL_SMOOTH );
}

void VectorPlotVisualisation::resizeGL(int w, int h){
  if(debug) cout << "VectorPlotVisualisation resizeGL" << endl;
  glViewport(0, 0, (GLint) w, (GLint) h);
  glMatrixMode( GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0., 1.0 * maxY, -1.0 * maxX, 0., 0., 1.);
  glMatrixMode( GL_MODELVIEW);
}

void VectorPlotVisualisation::paintGL(){
  if(debug) cout << "VectorPlotVisualisation PaintGL" << endl;
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
  glLoadIdentity();

  double maxToMin = colorPalette->getMax() - colorPalette->getMin();

  //glTranslatef(0.f, -1.f/2.f, 0.f);
  for (int i = 0; i < maxX; i++){
    for (int j = 0; j < maxY; j++) {
      double val = channel->getValue(i, j);

      QColor color = colorPalette->pickScaledColor(val);
      qglColor(color);

      val = colorPalette->getScaledValue(val);

      //clipping
      if( val > colorPalette->getMax()) val = colorPalette->getMax();
      if( val < colorPalette->getMin()) val = colorPalette->getMin();

      double y0, y1;

      if(colorPalette->getMax() < 0.){
        // zero above plotline
        y0 = 0.;
        y1 = (val - colorPalette->getMax()) / maxToMin;
      }else{
        if(colorPalette->getMin() > 0.){
          // zero under plotline
          y0 = (val - colorPalette->getMax()) / maxToMin;
          y1 = 1.;
        }else{
          if(val < 0.){
           y0 = ((-1.)* colorPalette->getMax()) / maxToMin; //zero
           y1 = (val - colorPalette->getMax()) / maxToMin;
          }else{
            y0 = (val - colorPalette->getMax()) / maxToMin;
            y1 = ((-1.)* colorPalette->getMax()) / maxToMin; //zero
          }
        }
      }

      glBegin( GL_QUADS);
      glVertex2f(.0f, (GLfloat) (y0));
      glVertex2f(1.f, (GLfloat) (y0));
      glVertex2f(1.0f, (GLfloat)  (y1));
      glVertex2f(.0f, (GLfloat) (y1));
      glEnd();

      glTranslatef(1.f, 0.f, 0.f);
    }
    glTranslatef(-1.f * maxY, -1.f, 0.f);
  }
}

void VectorPlotVisualisation::mouseMoveEvent ( QMouseEvent *event ){
  QString tTip;
  double xStep = width() / maxY;
  double yStep = height() / maxX;
  int n = (int) (event->y() / yStep);
  int m = (int) (event->x() / xStep);
  if (n == maxX) --n;
  if (m == maxY) --m;
  VectorPlotChannel *vectorPC = dynamic_cast<VectorPlotChannel *> (channel);
  if (vectorPC == NULL) {
    MatrixElementPlotChannel *elem = channel->getChannel(n, m);
    tTip = QString(elem->getChannelName().c_str()) + ": " + QString::number(elem->getValue());
  } else {
    VectorElementPlotChannel * elem = vectorPC->getChannel(n);
    tTip += QString(elem->getChannelName().c_str()) + ", " + QString::number(m) + ": " + QString::number(
        elem->getValue());
  }
  setToolTip((const QString) tTip); // shows up ToolTip "M[x,y]"
}

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
 *  Revision 1.4  2011-01-10 16:36:17  guettler
 *  -fixed memory leak: many many VERTEX were created but not deleted (after 10min 12GB RAM were full!)
 *  -use references for VERTEX instead of pointers for better performance
 *
 *  Revision 1.3  2010/06/30 11:31:11  robot14
 *  VectorPlotChannel specs removed
 *
 *                       *
 *                                                                         *
 **************************************************************************/
#include "LandscapeVisualisation.h"
#include "math.h"
#include <iostream>
#include <string>
#include <GL/glut.h>    // Header File For The GLUT Library


using namespace std;

LandscapeVisualisation::LandscapeVisualisation(MatrixPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent)
: AbstractVisualisation(channel, colorPalette, parent){

  if(debug) cout << "TextureVisualisation Konstruktor" << endl;
  object = 0;
  maxX = this->channel->getDimension(0);
  maxY = this->channel->getDimension(1);
  rotX = rotY = 0;
  zoom = 1.;
  plateauRadius = 0.2;
  setMouseTracking(true); // enables tooltips while mousemoving over widget
}

LandscapeVisualisation::~LandscapeVisualisation(){
  if(debug) cout << "LandscapeVisualisation Destruktor" << endl;
  makeCurrent();
}

void LandscapeVisualisation::initializeGL(){
  if(debug) cout << "LandscapeVisualisation Konstruktor" << endl;
  qglClearColor( Qt::black);    // Let OpenGL clear to black
  glClearDepth(1.0f);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_NORMALIZE);
  glShadeModel( GL_SMOOTH );
  glHint(GL_POLYGON_SMOOTH_HINT, GL_DONT_CARE);
  //  GLfloat LightAmbient[]= { .5f, .5f, .5f, .1f };
  GLfloat LightDiffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
  GLfloat LightPosition[] = { maxX * 2.0f, 5.0f, maxY * 2.0f, 1.0f }; //todo licht anpassen

  //  glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
  glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
  glEnable(GL_LIGHT1);
  glEnable(GL_LIGHTING);
}

void LandscapeVisualisation::resizeGL(int w, int h){
  if(debug) cout << "LandscapeVisualisation resizeGL" << endl;
  glViewport(0, 0, (GLint) w, (GLint) h);
  glMatrixMode( GL_PROJECTION);
  glLoadIdentity();

  gluPerspective(30.f,(GLfloat) w/(GLfloat) h, .2f,50.f);

  glMatrixMode( GL_MODELVIEW);
}

void LandscapeVisualisation::paintGL(){
  if(debug) cout << "LandscapeVisualisation PaintGL" << endl;
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
  glLoadIdentity();

  gluLookAt(
      5.0f, 5.0f, -5.0f, //todo entfernung anpassen
      0.0f, 0.0f, 0.0f,
      0.0f, 1.0f, 0.0f);

  glRotatef(rotX* -1.f, 1.f,0.f,1.f); // neigen..
  glRotatef((GLfloat) rotY, 0.0f, 1.0f, 0.0f); // drehen y achse

  glTranslatef(((-.5f * maxX) + .5f)*zoom, 0.f, ((.5f * maxY) - .5f)*zoom); // put the object in center
  glScalef( zoom, zoom,zoom);

  for (int j = 0; j < maxY; j++) {
    for (int i = 0; i < maxX; i++) {
      double p00, p01, p10, p11;
      VERTEX v1_00, v1_01, v1_10, v1_11, v2_00, v2_01, v3_00, v4_00, v4_10;
      p00 = clip(colorPalette->getScaledValue(channel->getValue(i, j)));
      //clipping:

      //zeichnen des plateaus (i -> x, j -> -z)
      v1_00 = VERTEX((GLfloat) (-1. * plateauRadius), (GLfloat) p00, (GLfloat) plateauRadius);
      v1_01 = VERTEX((GLfloat) (-1. * plateauRadius), (GLfloat) p00, (GLfloat) (-1. * plateauRadius));
      v1_10 = VERTEX((GLfloat) plateauRadius, (GLfloat) p00, (GLfloat) plateauRadius);
      v1_11 = VERTEX((GLfloat) plateauRadius, (GLfloat) p00, (GLfloat) (-1. * plateauRadius));
      drawTriangle(v1_00, v1_11, v1_10);
      drawTriangle(v1_00, v1_01, v1_11);

      //verbindung i richtung
      if (i < maxX - 1) {
        p10 = clip(colorPalette->getScaledValue(channel->getValue(i + 1, j)));
        v2_00 = VERTEX((GLfloat) ((-1. * plateauRadius) + 1), (GLfloat) p10, (GLfloat) plateauRadius);
        v2_01 = VERTEX((GLfloat) ((-1. * plateauRadius) + 1), (GLfloat) p10, (GLfloat) (-1. * plateauRadius));
        divideAndDrawTriangle(v1_10, v2_01, v2_00);
        divideAndDrawTriangle(v1_10, v1_11, v2_01);
      }

      //vebindung in j richtung
      if (j < maxY - 1) {
        p01 = clip(colorPalette->getScaledValue(channel->getValue(i, j + 1)));
        v4_00 = VERTEX((GLfloat) (-1. * plateauRadius), (GLfloat) p01, (GLfloat) (plateauRadius - 1));
        v4_10 = VERTEX((GLfloat) plateauRadius, (GLfloat) p01, (GLfloat) (plateauRadius - 1));
        divideAndDrawTriangle(v1_01, v4_00, v4_10);
        divideAndDrawTriangle(v1_01, v4_10, v1_11);
      }

      if (i < maxX - 1 && j < maxY - 1) { //fläche zwischen 4stops
        p11 = clip(colorPalette->getScaledValue(channel->getValue(i + 1, j + 1)));
        v3_00 = VERTEX((GLfloat) ((-1. * plateauRadius) + 1), (GLfloat) p11, (GLfloat) (plateauRadius - 1));

        if (fabs((p01 + p11 + p10) / 3) > fabs((p00 + p01 + p10 + p11) / 4) || fabs((p00 + p01 + p10) / 3) > fabs((p00
            + p01 + p10 + p11) / 4)) {
          //triangle p01, p10, p00 / p01, p11, p10
          //kante zwischen p10 u p01
          divideAndDrawTriangle(v1_11, v4_10, v2_01);
          divideAndDrawTriangle(v4_10, v3_00, v2_01);
        } else {
          //triangle p00, p11, p10 / p00, p01, p11
          divideAndDrawTriangle(v1_11, v4_10, v3_00);
          divideAndDrawTriangle(v1_11, v3_00, v2_01);
        }
      }
      glTranslatef(1.f,0.f,0.f);
    }
    glTranslatef((GLfloat) (-1. * maxX), 0.f, -1.f);
  }
}

double LandscapeVisualisation::clip(double val){
  if( val > colorPalette->getMax())
    return colorPalette->getMax();
  if( val < colorPalette->getMin())
    return colorPalette->getMin();
  else
    return val;
}
// algorithm for triangulation of stops within the landscape edges
void LandscapeVisualisation::divideAndDrawTriangle(VERTEX& v1, VERTEX& v2, VERTEX& v3, VERTEX& n){
  if(debug) cout << "LandscapeVisualisation::divideAndDrawTriangle"/* << v1->y << v2->y << v3->y */<< endl;
  // y -> values
  double stop1 = colorPalette->getNextStopPosition(v1.y,v2.y);
  if(v1.y == v2.y && v2.y == v3.y){
    drawTriangle(v1, v2, v3, n);
    return;
  }
  if(stop1 != v2.y){ //if there exists stop between v1 and v2
    VERTEX v12 = getVertexBetween(v1,v2, stop1);
    //2. search v1->v3
    double stop2 = colorPalette->getNextStopPosition(v1.y,v3.y);

    if(stop2 != v3.y){ // between v1 and v3 too
      VERTEX v13 = getVertexBetween(v1,v3, stop2);
      // -1-
      if(debug) cout << "-1-" << endl;
      drawTriangle(v1, v12, v13, n);
      divideAndDrawTriangle(v12, v2, v3, n);
      divideAndDrawTriangle(v12, v3, v13, n);
    }else{
      //3. search v2->v3
      double stop3 = colorPalette->getNextStopPosition(v3.y,v2.y);

      if(stop3 != v2.y){
        VERTEX v32 = getVertexBetween(v3, v2, stop3);
        // -2-
        if(debug) cout << "-2-" << endl;
        drawTriangle(v1, v12, v32, n);
        drawTriangle(v1, v32, v3, n);
        divideAndDrawTriangle(v12, v2, v32, n);
      }else{
        // -3-
        if(debug) cout << "-3-" << endl;
        drawTriangle(v1, v12, v3, n);
        divideAndDrawTriangle(v12, v2, v3, n);
      }
    }
  }else{
    //3. search v2->v3
    double stop3 = colorPalette->getNextStopPosition((double) (v2.y), (double) (v3.y));

    if(stop3 != v3.y){
      VERTEX v23 = getVertexBetween(v2, v3, stop3);
      //2. search v1->v3
      double stop2 = colorPalette->getNextStopPosition((double) (v1.y),(double) (v3.y));

      if(stop2 != v3.y){
        // -4-
        if(debug) cout << "-4-" << endl;
        VERTEX v13 = getVertexBetween(v1, v3, stop2);
        drawTriangle(v1, v2, v23, n);
        drawTriangle(v1, v23, v13, n);
        divideAndDrawTriangle(v13, v23, v3, n);
      }else{
        // -5-
        if(debug) cout << "-5-" << endl;
        drawTriangle(v1, v2, v23, n);
        divideAndDrawTriangle(v1, v23, v3, n);
      }
    }else{
      //2. search v1->v3
      double stop2 = colorPalette->getNextStopPosition(v1.y,v3.y);

      if (stop2 != v3.y) { // between v1 and v3 too
        VERTEX v13 = getVertexBetween(v1, v3, stop2);
        // -6-
        if(debug) cout << "-6-" << endl;
        drawTriangle(v1, v2, v13, n);
        divideAndDrawTriangle(v2, v3, v13, n);
      } else {
        // - 7-
        if(debug) cout << "-7-" << endl;
        drawTriangle(v1, v2, v3, n);
      }
    }
  }
}

void LandscapeVisualisation::drawTriangle(VERTEX& v1, VERTEX& v2, VERTEX& v3, VERTEX& n){
  if(debug) cout << "LandscapeVisualisation::drawTriangle"/* << v1->y << "; " << v2->y << "; " << v3->y*/ << endl;
  QColor c1 = colorPalette->pickColor(v1.y),
            c2 = colorPalette->pickColor(v2.y),
            c3 = colorPalette->pickColor(v3.y);
  glBegin( GL_TRIANGLES); //kante zwischen p10 u p01
   glNormal3f(n.x, n.y, n.z);
   qglColor(c1); glVertex3f(v1.x, v1.y, v1.z);
   qglColor(c2); glVertex3f(v2.x, v2.y, v2.z);
   qglColor(c3); glVertex3f(v3.x, v3.y, v3.z);
  glEnd();
}

VERTEX LandscapeVisualisation::getNormal(VERTEX& v1, VERTEX& v2, VERTEX& v3){
  GLfloat a1 = v2.x - v1.x,
      a2 = v2.y - v1.y,
      a3 = v2.z - v1.z,
      b1 = v3.x - v1.x,
      b2 = v3.y - v1.y,
      b3 = v3.z - v1.z;
  /*
   * cross product v1->v2 x v1->v3
   */
  GLfloat x = (a2 * b3) - (a3 * b2),
      y = (a3 * b1) - (a1 * b3),
      z = (a1 * b2) - (a2 * b1);
  if(y < 0.f){
    x = -1.f * x;
    y = -1.f * y;
    z = -1.f * z;
  }
  return VERTEX(x,y,z);
}

VERTEX LandscapeVisualisation::getVertexBetween(VERTEX& v1, VERTEX& v2, double pos){
  if(debug) cout << "LandscapeVisualisation::getVertexBetween" << endl;
  //schnittpunkt mit ebene y = pos
  GLfloat alpha = (pos - v1.y) / (v2.y -v1.y);
  return VERTEX(v1.x + alpha * (v2.x - v1.x), (GLfloat) pos, v1.z + alpha * (v2.z - v1.z));
}

void LandscapeVisualisation::mouseMoveEvent ( QMouseEvent *event ){
  if(event->buttons() == Qt::LeftButton && ( event->x() != mouseX || event->y() != mouseY)){
    rotX += (event->y() - mouseY) / 2; //variable umbenennen.. rot um z in xyebene
    rotY += (event->x() - mouseX)/2;
//    if(debug) cout << "rotX= " << rotX << " rotY= " << rotY << endl;
    mouseY = event->y();
    mouseX = event->x();
    updateGL();
  }
}

void LandscapeVisualisation::wheelEvent(QWheelEvent * event){
  if(debug) cout << event->delta() << endl;
  zoom += ((event->delta() / 120) * 0.1);
  if ( zoom < 0.) zoom = 0.;
  updateGL();
  event->accept();
}

void LandscapeVisualisation::mousePressEvent ( QMouseEvent *event ){
  mouseX = event->x();
  mouseY = event->y();
}

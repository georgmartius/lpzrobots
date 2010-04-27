/*
 * BarVisualisation.cpp
 *
 *  Created on: 21.04.2010
 *      Author: oni
 */
#include "BarVisualisation.h"
#include "math.h"
#include <iostream>
#include <string>
#include <GL/glut.h>    // Header File For The GLUT Library


using namespace std;

BarVisualisation::BarVisualisation(MatrixPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent)
: AbstractVisualisation(channel, colorPalette, parent){

  if(debug) cout << "TextureVisualisation Konstruktor" << endl;
//  this->channel = channel;
//  this->colorPalette = colorPalette;
  object = 0;
  zoom = 1.;
  maxX = this->matrixChannel->getDimension(0);
  maxY = this->matrixChannel->getDimension(1);
  rotX = rotY = 0;
  //setUpdatesEnabled(true);
  setMouseTracking(true); // enables tooltips while mousemoving over widget
}

BarVisualisation::BarVisualisation(VectorPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent)
: AbstractVisualisation(channel, colorPalette, parent){

  // TODO
  if(debug) cout << "TextureVisualisation Konstruktor" << endl;
//  this->channel = channel;
//  this->colorPalette = colorPalette;
  object = 0;
  zoom = 1.;
//  maxX = this->matrixChannel->getDimension(0);
//  maxY = this->matrixChannel->getDimension(1);
  rotX = rotY = 0;
  //setUpdatesEnabled(true);
  setMouseTracking(true); // enables tooltips while mousemoving over widget
}

BarVisualisation::~BarVisualisation(){
  if(debug) cout << "LandscapeVisualisation Destruktor" << endl;
  makeCurrent();
//  glDeleteLists( object, 1 );
}

void BarVisualisation::initializeGL(){
  if(debug) cout << "LandscapeVisualisation Konstruktor" << endl;
  qglClearColor( Qt::black);    // Let OpenGL clear to black
//  object = makeObject();    // Generate an OpenGL display list
  glEnable(GL_DEPTH_TEST);
  glShadeModel( GL_SMOOTH );
//  glHint(GL_POLYGON_SMOOTH_HINT, GL_DONT_CARE);
}

void BarVisualisation::resizeGL(int w, int h){
  if(debug) cout << "LandscapeVisualisation resizeGL" << endl;
  glViewport(0, 0, (GLint) w, (GLint) h);
  glMatrixMode( GL_PROJECTION);
  glLoadIdentity();

  gluPerspective(30.f,(GLfloat) w/(GLfloat) h, .2f,50.f);

  glMatrixMode( GL_MODELVIEW);
}

void BarVisualisation::paintGL(){
  if(debug) cout << "LandscapeVisualisation PaintGL" << endl;
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
  glLoadIdentity();

  gluLookAt(
      5.0f /* cos(rotX)*/, 5.0f /* sin(rotX)*/, 5.0f, // x - 5.f * cos rotX , y - 5.f *Ssin rotwinkel
      0.0f, 0.0f, 0.0f,
      0.0f, 1.0f, 0.0f);

  glRotatef((GLfloat) rotX, 1.f,0.f,-1.f); // neigen.. rotation um die x,-z-achse
  glRotatef((GLfloat) rotY, 0.0f, 1.0f, 0.0f); // drehen y achse

  glTranslatef(((-.5f * maxX)*zoom), 0.f, ((-.5f * maxY)*zoom)); // put the object in center
  glScalef( zoom, zoom,zoom);

  for (int i = 0; i < maxX; i++){
    for (int j = 0; j < maxY; j++) {
      double val = matrixChannel->getValue(i, j);
      QColor color = colorPalette->pickColor(val);

      qglColor(color);
      drawBar(val);
      glTranslatef(1.f, .0f, .0f);
    }
  glTranslatef((-1.f * maxY), .0f, 1.f);
  }
}

void BarVisualisation::drawBar(double value){
  glBegin(GL_QUADS); //Y -> value

     // Front Face
     glVertex3f(.0f, .0f, 1.0f);
     glVertex3f( 1.0f, .0f, 1.0f);
     glVertex3f( 1.0f, (GLfloat) (value), 1.0f);
     glVertex3f(.0f, (GLfloat) (value), 1.0f);

     // Back Face
     glVertex3f(.0f, .0f, .0f);
     glVertex3f(.0f, (GLfloat) (value), .0f);
     glVertex3f( 1.0f, (GLfloat) (value), .0f);
     glVertex3f( 1.0f, .0f, .0f);

     // Top Face
     glVertex3f(.0f, (GLfloat) (value), .0f);
     glVertex3f(.0f, (GLfloat) (value), 1.0f);
     glVertex3f( 1.0f, (GLfloat) (value), 1.0f);
     glVertex3f( 1.0f, (GLfloat) (value), .0f);

     // Bottom Face
     glVertex3f(.0f, .0f, .0f);
     glVertex3f( 1.0f, .0f, .0f);
     glVertex3f( 1.0f, .0f, 1.0f);
     glVertex3f(.0f, .0f, 1.0f);

     // Right Face
     glVertex3f( 1.0f, .0f, .0f);
     glVertex3f( 1.0f, (GLfloat) (value), .0f);
     glVertex3f( 1.0f, (GLfloat) (value), 1.0f);
     glVertex3f( 1.0f, .0f, 1.0f);

     // Left Face
     glVertex3f(.0f, .0f, .0f);
     glVertex3f(.0f, .0f, 1.0f);
     glVertex3f(.0f, (GLfloat) (value), 1.0f);
     glVertex3f(.0f, (GLfloat) (value), .0f);

   glEnd();
}

GLuint BarVisualisation::makeObject() { //obsolete
  if(debug) cout << "LandscapeVisualisation makeObject" << endl;
  GLuint list;

  list = glGenLists(1);

  glNewList(list, GL_COMPILE);

  //qglColor(Qt::white); // Shorthand for glColor3f or glIndex

  glEnable(GL_TEXTURE_2D);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  //glBindTexture(GL_TEXTURE_2D, texName);
  glBegin( GL_QUADS); // Draw A Quadab
  glTexCoord2d(0.0,0.0);glVertex2f(-1.0f, 1.0f); // Top Left
  glTexCoord2d(0.0155*maxX,0.0);glVertex2f(1.0f, 1.0f); // Top Right
  glTexCoord2d(0.0155*maxX,0.0155*maxY);glVertex2f(1.0f, -1.0f); // Bottom Right
  glTexCoord2d(0.0,0.0155*maxY);glVertex2f(-1.0f, -1.0f); // Bottom Left
  glEnd(); // Done Drawing The Quad
  glFlush();
  glDisable(GL_TEXTURE_2D);
  glEndList();

  return list;
}


void BarVisualisation::mouseMoveEvent ( QMouseEvent *event ){
//  QString tTip = channel->getChannelName().c_str();
//  double xStep = width() / channel->getDimension(0);
//  double yStep = height() / channel->getDimension(1);
//  tTip += "[" + QString::number( (int) (event->x() / xStep)) + ","
//       + QString::number( (int) (event->y() / yStep) ) + "]";
//  setToolTip((const QString) tTip);  // shows up ToolTip "M[x,y]"
  //if ( debug) cout << "MouseCoords: " << event->x() << ", " << event->y() << endl;
  if(event->buttons() == Qt::LeftButton && ( event->x() != mouseX || event->y() != mouseY)){
    rotX += (event->y() - mouseY) / 2; //variable umbenennen.. rot um z in xyebene
    rotY += (event->x() - mouseX)/2;
//    if(debug) cout << "rotX= " << rotX << " rotY= " << rotY << endl;
    mouseY = event->y();
    mouseX = event->x();
    updateGL();
  }
}

void BarVisualisation::wheelEvent(QWheelEvent * event){
  if(debug) cout << event->delta() << endl;
  zoom += ((event->delta() / 120) * 0.1);
  if ( zoom < 0.) zoom = 0.;
  updateGL();
  event->accept();
}

void BarVisualisation::mousePressEvent ( QMouseEvent *event ){
  mouseX = event->x();
  mouseY = event->y();
}

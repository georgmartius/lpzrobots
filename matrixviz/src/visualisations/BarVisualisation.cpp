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

  if(debug) cout << "BarVisualisation Konstruktor" << endl;
//  this->channel = channel;
//  this->colorPalette = colorPalette;
  zoom = 1.;
  maxX = this->matrixChannel->getDimension(0);
  maxY = this->matrixChannel->getDimension(1);
  rotX = rotY = 0;
  lightOn = true;
  //setUpdatesEnabled(true);
  setMouseTracking(true); // enables tooltips while mousemoving over widget
}

BarVisualisation::BarVisualisation(VectorPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent)
: AbstractVisualisation(channel, colorPalette, parent){

  // TODO
  if(debug) cout << "BarVisualisation Konstruktor" << endl;
//  this->channel = channel;
//  this->colorPalette = colorPalette;
  zoom = 1.;
//  maxX = this->matrixChannel->getDimension(0);
//  maxY = this->matrixChannel->getDimension(1);
  rotX = rotY = 0;
  lightOn = true;
  //setUpdatesEnabled(true);
  setMouseTracking(true); // enables tooltips while mousemoving over widget
}

BarVisualisation::~BarVisualisation(){
  if(debug) cout << "BarVisualisation Destruktor" << endl;
  makeCurrent();
}

void BarVisualisation::initializeGL(){
  if(debug) cout << "BarVisualisation Konstruktor" << endl;
  qglClearColor( Qt::black);    // Let OpenGL clear to black
  glClearDepth(1.0f);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_COLOR_MATERIAL);
//  glDepthFunc(GL_LEQUAL);
  glShadeModel( GL_SMOOTH );
  glHint(GL_POLYGON_SMOOTH_HINT, GL_DONT_CARE);
//  GLfloat LightAmbient[]= { .5f, .5f, .5f, .1f };
  GLfloat LightDiffuse[]= { 1.0f, 1.0f, 1.0f, 1.0f };
  GLfloat LightPosition[]= { 5.0f, .0f, 5.0f, 1.0f };

//  glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
  glLightfv(GL_LIGHT1, GL_POSITION,LightPosition);
  glEnable(GL_LIGHT1);
  glEnable(GL_LIGHTING);
}

void BarVisualisation::resizeGL(int w, int h){
  if(debug) cout << "BarVisualisation resizeGL" << endl;
  glViewport(0, 0, (GLint) w, (GLint) h);
  glMatrixMode( GL_PROJECTION);
  glLoadIdentity();

  gluPerspective(30.f,(GLfloat) w/(GLfloat) h, .2f,50.f);

  glMatrixMode( GL_MODELVIEW);
}

void BarVisualisation::paintGL(){
  if(debug) cout << "BarVisualisation PaintGL" << endl;
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

  if(value > 0){
     // Front Face
     glNormal3f(0.f,0.f,1.f);
     glVertex3f(.0f, .0f, 1.0f);
     glVertex3f( 1.0f, .0f, 1.0f);
     glVertex3f( 1.0f, (GLfloat) (value), 1.0f);
     glVertex3f(.0f, (GLfloat) (value), 1.0f);

     // Back Face
     glNormal3f(0.f,0.f,-1.f);
     glVertex3f(.0f, .0f, .0f);
     glVertex3f(.0f, (GLfloat) (value), .0f);
     glVertex3f( 1.0f, (GLfloat) (value), .0f);
     glVertex3f( 1.0f, .0f, .0f);

     // Top Face
     glNormal3f(0.f,1.f,0.f);
     glVertex3f(.0f, (GLfloat) (value), .0f);
     glVertex3f(.0f, (GLfloat) (value), 1.0f);
     glVertex3f( 1.0f, (GLfloat) (value), 1.0f);
     glVertex3f( 1.0f, (GLfloat) (value), .0f);

     // Bottom Face
     glNormal3f(0.f,-1.f,0.f);
     glVertex3f(.0f, .0f, .0f);
     glVertex3f( 1.0f, .0f, .0f);
     glVertex3f( 1.0f, .0f, 1.0f);
     glVertex3f(.0f, .0f, 1.0f);

     // Right Face
     glNormal3f(1.f,0.f,0.f);
     glVertex3f( 1.0f, .0f, .0f);
     glVertex3f( 1.0f, (GLfloat) (value), .0f);
     glVertex3f( 1.0f, (GLfloat) (value), 1.0f);
     glVertex3f( 1.0f, .0f, 1.0f);

     // Left Face
     glNormal3f(-1.f,0.f,0.f);
     glVertex3f(.0f, .0f, .0f);
     glVertex3f(.0f, .0f, 1.0f);
     glVertex3f(.0f, (GLfloat) (value), 1.0f);
     glVertex3f(.0f, (GLfloat) (value), .0f);
  }else{
    // Front Face
    glNormal3f(0.f, 0.f, 1.f);
    glVertex3f(.0f, (GLfloat) (value), 1.0f);
    glVertex3f(1.0f, (GLfloat) (value), 1.0f);
    glVertex3f(1.0f, 0.f, 1.0f);
    glVertex3f(.0f, 0.f, 1.0f);

    // Back Face
    glNormal3f(0.f, 0.f, -1.f);
    glVertex3f(.0f, (GLfloat) (value), .0f);
    glVertex3f(.0f, 0.f, .0f);
    glVertex3f(1.0f, 0.f, .0f);
    glVertex3f(1.0f, (GLfloat) (value), .0f);

    // Top Face
    glNormal3f(0.f, 1.f, 0.f);
    glVertex3f(.0f, 0.f, .0f);
    glVertex3f(.0f, 0.f, 1.0f);
    glVertex3f(1.0f, 0.f, 1.0f);
    glVertex3f(1.0f, 0.f, .0f);

    // Bottom Face
    glNormal3f(0.f, -1.f, 0.f);
    glVertex3f(.0f, (GLfloat) (value), .0f);
    glVertex3f(1.0f, (GLfloat) (value), .0f);
    glVertex3f(1.0f, (GLfloat) (value), 1.0f);
    glVertex3f(.0f, (GLfloat) (value), 1.0f);

    // Right Face
    glNormal3f(1.f, 0.f, 0.f);
    glVertex3f(1.0f, .0f, .0f);
    glVertex3f(1.0f, 0.f, 1.0f);
    glVertex3f(1.0f, (GLfloat) (value), 1.0f);
    glVertex3f(1.0f, (GLfloat) (value), .0f);

    // Left Face
    glNormal3f(-1.f, 0.f, 0.f);
    glVertex3f(.0f, .0f, .0f);
    glVertex3f(.0f, (GLfloat) (value), .0f);
    glVertex3f(.0f, (GLfloat) (value), 1.0f);
    glVertex3f(.0f, 0.f, 1.0f);
  }
   glEnd();
}


void BarVisualisation::mouseMoveEvent ( QMouseEvent *event ){
  if(event->buttons() == Qt::LeftButton && ( event->x() != mouseX || event->y() != mouseY)){
    rotX += (event->y() - mouseY) / 2; //variable umbenennen.. rot um z in xyebene
    rotY += (event->x() - mouseX) / 2;
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

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

#include "TextureVisualisation.h"
#include "math.h"
#include <iostream>
#include <string>


using namespace std;

TextureVisualisation::TextureVisualisation(MatrixPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent)
: AbstractVisualisation(channel, colorPalette, parent){

  if(debug) cout << "TextureVisualisation Konstruktor" << endl;
  this->matrixChannel = channel;
  this->colorPalette = colorPalette;
  this->vectorChannel = NULL;
  object = NULL;
  maxX = matrixChannel->getDimension(0);
  maxY = matrixChannel->getDimension(1);
  //setUpdatesEnabled(true);
  setMouseTracking(true); // enables tooltips while mousemoving over widget
}

TextureVisualisation::TextureVisualisation(VectorPlotChannel *channel, ColorPalette *colorPalette, QWidget *parent)
: AbstractVisualisation(channel, colorPalette, parent){

  if(debug) cout << "TextureVisualisation Konstruktor" << endl;
  this->vectorChannel = channel;
  this->colorPalette = colorPalette;
  this->matrixChannel = NULL;
  object = NULL;
  maxX = vectorChannel->getSize();
  maxY = vectorChannel->getBufferSize();
  //setUpdatesEnabled(true);
  setMouseTracking(true); // enables tooltips while mousemoving over widget
  if(debug) cout << "TextureVisualisation Konstruktor" << endl;
}

TextureVisualisation::~TextureVisualisation(){
  if(debug) cout << "TextureVisualisation Destruktor" << endl;
  makeCurrent();
  glDeleteLists( object, 1 );
}

void TextureVisualisation::initializeGL(){
  if(debug) cout << "TextureVisualisation initializeGL" << endl;
  qglClearColor( Qt::black);    // Let OpenGL clear to black
  object = makeObject();    // Generate an OpenGL display list
  glShadeModel( GL_FLAT );

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glGenTextures(1, &texName);
  glBindTexture(GL_TEXTURE_2D, texName);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  //initialize texture
  for(int i = 0; i < 64; i++)
    for(int j = 0; j < 64; j++)
      for(int k = 0; k < 3; k++) tex[i][j][k] = (GLubyte) 255;

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 64, 64, 0, GL_RGB, GL_UNSIGNED_BYTE, tex); //TODO
}

void TextureVisualisation::resizeGL(int w, int h){
  if(debug) cout << "TextureVisualisation resizeGL" << endl;
  glViewport(0, 0, (GLint) w, (GLint) h);
  glMatrixMode( GL_PROJECTION);
  glLoadIdentity();
  glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 1.0);
  glMatrixMode( GL_MODELVIEW);
}

void TextureVisualisation::paintGL(){
  if(debug) cout << "TextureVisualisation PaintGL" << endl;
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
  glLoadIdentity();
  glBindTexture(GL_TEXTURE_2D, texName);

//  MatrixPlotChannel* matrixChannel = dynamic_cast<MatrixPlotChannel*> (channel);
//  VectorPlotChannel* vectorChannel = dynamic_cast<VectorPlotChannel*> (channel);

  GLubyte subTex[maxX][maxY][3];
  for (int i = 0; i < maxX; i++)
    for (int j = 0; j < maxY; j++) {
      QColor color;
      if(matrixChannel != NULL) color = colorPalette->pickColor(matrixChannel->getValue(i, j));
      else color = colorPalette->pickColor(vectorChannel->getValue(i, j));
      subTex[i][j][0] = (GLubyte) color.red();
      subTex[i][j][1] = (GLubyte) color.green();
      subTex[i][j][2] = (GLubyte) color.blue();
    }

  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0,
                           maxY, maxX, GL_RGB,
                           GL_UNSIGNED_BYTE, subTex);
  glCallList( object );
}

GLuint TextureVisualisation::makeObject() {//TODO paintEvent here..
  if(debug) cout << "TextureVisualisation makeObject" << endl;
  GLuint list;

  list = glGenLists(1);

  glNewList(list, GL_COMPILE);

  //qglColor(Qt::white); // Shorthand for glColor3f or glIndex

  glEnable(GL_TEXTURE_2D);
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  //glBindTexture(GL_TEXTURE_2D, texName);
  glBegin( GL_QUADS); // Draw A Quadab
  glTexCoord2d(0.0,0.0);glVertex2f(-1.0f, 1.0f); // Top Left
  glTexCoord2d(0.0155*maxY,0.0);glVertex2f(1.0f, 1.0f); // Top Right
  glTexCoord2d(0.0155*maxY,0.0155*maxX);glVertex2f(1.0f, -1.0f); // Bottom Right
  glTexCoord2d(0.0,0.0155*maxX);glVertex2f(-1.0f, -1.0f); // Bottom Left
  glEnd(); // Done Drawing The Quad
  glFlush();
  glDisable(GL_TEXTURE_2D);
  glEndList();

  return list;
}


void TextureVisualisation::mouseMoveEvent ( QMouseEvent *event ){
  QString tTip;
  if(matrixChannel != NULL) tTip = matrixChannel->getChannelName().c_str();
  else tTip = vectorChannel->getChannelName().c_str();
  double xStep = width() / maxX;
  double yStep = height() / maxY;
  if ( matrixChannel != NULL){
    tTip += "[" + QString::number( (int) (event->y() / yStep)) + ","
          + QString::number( (int) (event->x() / xStep) ) + "]";
  }else{  // fehlerhaft!!
    tTip += "[" + QString::number( (int) (event->y() / yStep)) + "]:"
              + QString::number( (int) (event->x() / xStep) );
  }
  setToolTip((const QString) tTip);  // shows up ToolTip "M[x,y]"
  //if ( debug) cout << "MouseCoords: " << event->x() << ", " << event->y() << endl;
}


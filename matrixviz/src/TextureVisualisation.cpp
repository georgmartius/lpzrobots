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

#define PI 3.14159265

TextureVisualisation::TextureVisualisation(MatrixPlotChannel *channel, QWidget *parent)
: AbstractVisualisation(channel, parent){

  this->channel = channel;
  mainLayout = new QVBoxLayout();
  //setUpdatesEnabled(true);

  int maxX = this->channel->getDimension(0);
  int maxY = this->channel->getDimension(1);

  tex = new QImage(maxX, maxY, QImage::Format_RGB32);
  maxVal = 10; //->slider

  //QImage scaledTex = tex->scaled(200,200);

  //mainLayout->addWidget(*tex);
  setLayout(mainLayout);
  resize(200,200);
}

TextureVisualisation::~TextureVisualisation(){}

void TextureVisualisation::paintEvent(QPaintEvent *){

  std::cout << "ich male!" << std::endl;

  QPainter painter ( this );

  //test
  int maxX = this->channel->getDimension(0);
  int maxY = this->channel->getDimension(1);
  for (int i = 0; i < maxX; i++)
    for (int j = 0; j < maxY; j++)
     tex->setPixel(i, j, pickColor(channel->getValue(i, j))); //QString::number(channel->getValue(i, j))

  QImage scaledTex = tex->scaled(200,200);

  painter.drawImage ( 0, 0, scaledTex);

  //scale

  painter.end();
}

QRgb TextureVisualisation::pickColor(double val){
  QColor *col = new QColor(255, 255, 255);
//  int hue = floor( asin(val) * (180 / PI));
//  col->setHsv(hue, 255, 255);
//  std::cout << val / maxVal << " : " <<  col->rgb() << std::endl;
  if(val > 0.2) val = 0.2;
  int grey = floor(((val+0.2) / (2*0.2)) *255);
  col->setRgb(grey, grey, grey);
  return col->rgb();
}




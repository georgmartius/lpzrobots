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

#include "TestVisualisation.h"
#include <QString>
#include <iostream>


TestVisualisation::TestVisualisation(MatrixPlotChannel *channel, QWidget *parent )
: AbstractVisualisation(channel, parent) {

  this->channel = channel;
  mainLayout = new QVBoxLayout();

  label = new QLabel();
  editText();

  mainLayout->addWidget(label);
  setLayout(mainLayout);
  resize(200,200);
}
TestVisualisation::~TestVisualisation(){}


void TestVisualisation::paintEvent(QPaintEvent *){
  editText();
//  update();
}

void TestVisualisation::editText(){

  QString matrixPlain = "Matrix:\n";
  int maxX = this->channel->getDimension(0);
  int maxY = this->channel->getDimension(1);
  for(int i = 0; i < maxX; i++){
    for(int j = 0; j < maxY; j++){
      matrixPlain.append(QString::number(channel->getValue(i, j)));
      if(j < maxY - 1) matrixPlain.append("\t");
    }
    matrixPlain.append("\n");
  }
  label->setText(matrixPlain);
  std::cout << matrixPlain.toStdString () << std::endl; //test
}

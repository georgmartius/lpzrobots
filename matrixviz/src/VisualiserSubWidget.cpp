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

#include "VisualiserSubWidget.h"
#include "TextureVisualisation.h"
#include "TestVisualisation.h"



VisualiserSubWidget::VisualiserSubWidget(MatrixPlotChannel *channel, QWidget *parent)
: AbstractRobotSubWidget(parent) {
  this->channel = channel;
  int maxX = this->channel->getDimension(0);
  int maxY = this->channel->getDimension(1);
  for(int i = 0; i < maxX; i++){ //push back all MatrixElementPlotChannel for update
      for(int j = 0; j < maxY; j++){
        addPlotChannel(this->channel->getChannel(i, j));
      }
    }
  this->visualisation = new TestVisualisation(channel); //default visualisation
  initGui();
}

VisualiserSubWidget::~VisualiserSubWidget() {}

void VisualiserSubWidget::initGui(){
  mainLayout = new QVBoxLayout();
  vizChoice = new QComboBox();

  initVisTypes();

  mainLayout->addWidget(vizChoice);
  mainLayout->addWidget(visualisation);


  setLayout(mainLayout);
  resize(300,300);
}

void VisualiserSubWidget::updateViewableChannels(){
  //std::cout << "updateViewableChannels()" << std::endl;
  visualisation->repaint();
}

void VisualiserSubWidget::initVisTypes(){
  //init vis types TODO
  vizChoice->addItem("Test"); //0
  vizChoice->addItem("Tex"); //1
  //connect
  connect(vizChoice, SIGNAL(activated(int)), this, SLOT(switchVisMode( int)));
}

void VisualiserSubWidget::switchVisMode(int index){

  //mainLayout->
  mainLayout->removeWidget(visualisation);
  switch (index){
    case 0:
      this->visualisation = new TestVisualisation(channel);
      mainLayout->addWidget(visualisation);
      std::cout << "VisSwitch: 1" << std::endl;
      break;
    case 1:
      this->visualisation = new TextureVisualisation(channel);
      mainLayout->addWidget(visualisation);
      std::cout << "VisSwitch: 2" << std::endl;
      break;
  }
  updateViewableChannels();
  repaint();
}

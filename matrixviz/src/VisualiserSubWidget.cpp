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
#include "LandscapeVisualisation.h"

using namespace std;


VisualiserSubWidget::VisualiserSubWidget(MatrixPlotChannel *channel, int x, int y, int width, int heigt,
    QString cPFilePath, QWidget *parent)
: AbstractRobotSubWidget(parent){
  setWindowTitle(QString(channel->getChannelName().c_str()));
  if( debug) cout << "in VisualiserSubWidget Konstrunktor" << endl;
  this->matrixChannel = channel;
  this->colorPalette = new ColorPalette(this); //changed
  this->vectorChannel = 0;
  if( debug) cout << "nach CP Konstruktor" << endl;
  int maxX = this->matrixChannel->getDimension(0);
  int maxY = this->matrixChannel->getDimension(1);
  for(int i = 0; i < maxX; i++){ //push back all MatrixElementPlotChannel for update
      for(int j = 0; j < maxY; j++){
        addPlotChannel(this->matrixChannel->getChannel(i, j));
      }
    }
  this->visualisation = new TextureVisualisation(this->matrixChannel, colorPalette, this); //default visualisation
  initGui();
  if( !(x == 0 && y == 0 && width == 0 && heigt == 0)){
    resize(width,heigt);
      move(x, y);
  }
  if( cPFilePath != "" && cPFilePath != 0)
      colorPalette->loadStopListFromFile(cPFilePath);
  //TODO set filepath of colorPalette
  connect (this, SIGNAL(sendQuit()), colorPalette, SLOT(close()));
}

VisualiserSubWidget::VisualiserSubWidget(VectorPlotChannel *channel, int x, int y, int width, int heigt,
    QString cPFilePath, QWidget *parent)
: AbstractRobotSubWidget(parent){
  setWindowTitle(QString(channel->getChannelName().c_str()));
  if( debug) cout << "in VisualiserSubWidget Konstrunktor" << endl;
  this->vectorChannel = channel;
  this->matrixChannel = 0;
  this->colorPalette = new ColorPalette(this);
  //if( debug) cout << "nach CP Konstruktor" << endl;
  for(int i = 0; i < vectorChannel->getSize(); i++){ //push back all MatrixElementPlotChannel for update
      addPlotChannel(this->vectorChannel->getChannel(i));
  }
  this->visualisation = new TextureVisualisation(this->vectorChannel, colorPalette, this); //default visualisation
  initGui();
  if( !(x == 0 && y == 0 && width == 0 && heigt == 0)){
    resize(width,heigt);
    move(x, y);
  }
  if( cPFilePath != "" && cPFilePath != 0)
    colorPalette->loadStopListFromFile(cPFilePath);
  //TODO set filepath of colorPalette
  connect (this, SIGNAL(sendQuit()), colorPalette, SLOT(close()));
}

VisualiserSubWidget::~VisualiserSubWidget() {}

void VisualiserSubWidget::initGui(){
  if( debug) cout << "in initGui()" << endl;

  vizChoice = new QComboBox();

  initVisTypes();
  mainLayout = new QVBoxLayout();
  mainLayout->addWidget(vizChoice);
  visLayout = new QHBoxLayout();
  visLayout->addWidget(visualisation);
  if(colorPalette != 0) visLayout->addWidget(colorPalette);
  mainLayout->addLayout(visLayout);
  setLayout(mainLayout);
  resize(300,300);
}

void VisualiserSubWidget::updateViewableChannels(){
  //std::cout << "updateViewableChannels()" << std::endl;
  visualisation->update();
}

void VisualiserSubWidget::initVisTypes(){
  /*
   * visualisation modi for matrices
   */
  if(matrixChannel != 0){
    vizChoice->addItem("Tex"); //0
    vizChoice->addItem("3D - Landscape");
  }
  /*
   * visualisation modi for vectors
   */
  if(vectorChannel != 0){
    vizChoice->addItem("Tex");
  }
  //connect
  connect(vizChoice, SIGNAL(activated(int)), this, SLOT(switchVisMode( int)));
}

void VisualiserSubWidget::switchVisMode(int index){

  //remove old content
  visLayout->removeWidget(visualisation);
  visLayout->removeWidget(colorPalette);
  delete visualisation;
  if (debug) std::cout << "VisSwitch: " << index << std::endl;
  /*
   * change matrix visualisation
   */
  if(matrixChannel != 0){
    switch (index) {
      case 0:
        this->visualisation = new TextureVisualisation(matrixChannel, colorPalette, this);
        visLayout->addWidget(visualisation);
        visLayout->addWidget(colorPalette);
        break;
      case 1: //TODO TESTTEST
        this->visualisation = new LandscapeVisualisation(matrixChannel, colorPalette, this);
        //visLayout->addWidget(visualisation);
        visualisation->resize(300, 300);
        visLayout->addWidget(visualisation);
        //      visualisation->show();
        //visLayout->addWidget(colorPalette);
        break;
    }
  }
  /*
   * change vector visualisation
   */
  if(vectorChannel != 0){
    switch (index){
      case 0:
        this->visualisation = new TextureVisualisation(vectorChannel, colorPalette, this);
        visLayout->addWidget(visualisation);
        visLayout->addWidget(colorPalette);
        break;
    }
  }
  updateViewableChannels();
  repaint();
}

QString VisualiserSubWidget::getChannelName(){
  if(matrixChannel != 0) return QString(matrixChannel->getChannelName().c_str());
  else return (QString) QString(vectorChannel->getChannelName().c_str());
}
QString VisualiserSubWidget::getColorPaletteFilepath(){
  if(colorPalette != 0) return colorPalette->getPath();
  else return 0;
}
int VisualiserSubWidget::getVisMode(){
  return vizChoice->currentIndex();
}
QString VisualiserSubWidget::getMode(){
  if(matrixChannel != 0) return (QString) "matrix";
  else return (QString) "vector";
}

void VisualiserSubWidget::closeEvent(QCloseEvent * event){
  emit windowClosed(this);
  emit sendQuit();
  event->accept();
}

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
#include "BarVisualisation.h"
#include "VectorPlotVisualisation.h"

using namespace std;


VisualiserSubWidget::VisualiserSubWidget(MatrixPlotChannel *channel, int x, int y, int width, int heigt,
    QString cPFilePath, QWidget *parent)
: AbstractRobotSubWidget(parent){
  setWindowTitle(QString(channel->getChannelName().c_str()));
  if( debug) cout << "in VisualiserSubWidget Konstrunktor" << endl;
  this->matrixChannel = channel;
  this->colorPalette = new ColorPalette(this); //changed
  optionWidget = colorPalette->makeConfigBox();
  this->vectorChannel = 0;
  int maxX = this->matrixChannel->getDimension(0);
  int maxY = this->matrixChannel->getDimension(1);
  for(int i = 0; i < maxX; i++){ //push back all MatrixElementPlotChannel for update
      for(int j = 0; j < maxY; j++){
        addPlotChannel(this->matrixChannel->getChannel(i, j));
      }
    }
  this->visualisation = new TextureVisualisation(channel, colorPalette, this); //default visualisation
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
  optionWidget = colorPalette->makeConfigBox();
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

  menuBar = new QMenuBar();

//  vizChoice = new QComboBox(); obsolete

  initVisTypes();
  QMenu *optionMenu = new QMenu(tr("&Options"));
  QAction *showOptions = optionMenu->addAction(tr("&show"));
  showOptions->setCheckable(true);
  connect(optionMenu, SIGNAL(triggered(QAction *)), this, SLOT(showOptions(QAction*)));
  menuBar->addMenu(optionMenu);

//  QHBoxLayout *mainLayout = new QHBoxLayout;
//  mainLayout->addWidget(vizChoice); obsolete
  visLayout = new QHBoxLayout();
  visLayout->setContentsMargins(0,0,0,0);
  visLayout->setSpacing(0);

  visLayout->addWidget(visualisation);
  visLayout->addWidget(colorPalette);
//  switchVisMode(0);
  visMode = 0;
  optionWidget->hide();
//  optionWidget->setParent(this);
  optionsShown = false;
  visLayout->setMenuBar(menuBar);
  setLayout(visLayout);
  resize(300,300);
  update();
}

void VisualiserSubWidget::updateViewableChannels(){
  //std::cout << "updateViewableChannels()" << std::endl;
  visualisation->update();
  update();
}

void VisualiserSubWidget::initVisTypes(){
   QMenu *visMenu = new QMenu(tr("&vis mode"), this);
  /*
   * visualisation modi for matrices
   */
  if(matrixChannel != 0){
//    vizChoice->addItem("Tex"); //0 obsolete
//    vizChoice->addItem("3D - Landscape");

    visMenu->addAction(tr("&Tex"));
    visMenu->addAction(tr("&Landscape"));
    visMenu->addAction(tr("&Bar"));
  }
  /*
   * visualisation modi for vectors
   */
  if(vectorChannel != 0){
//    vizChoice->addItem("Tex"); obsolete

    visMenu->addAction(tr("&Tex"));
    visMenu->addAction(tr("&VectorPlot"));
  }

  menuBar->addMenu(visMenu);
  //connect
//  connect(vizChoice, SIGNAL(activated(int)), this, SLOT(switchVisMode( int)));
  connect(visMenu, SIGNAL(triggered(QAction *)), this, SLOT(switchVisMode(QAction*)));
}

void VisualiserSubWidget::switchVisMode(int index){
  if (debug) cout << "in switchVisMode i: " << index << endl;

  if ( visMode == index) return;
  else visMode = index;
  //TODO change contents in optionstoolbar
  //remove old content
  visLayout->removeWidget(visualisation);
  visLayout->removeWidget(colorPalette);
  if (debug) cout << "optionWidget: " << optionWidget << endl;
  visLayout->removeWidget(optionWidget);
//  delete visualisation;
  if (debug) std::cout << "VisSwitch: " << index << std::endl;
  /*
   * change matrix visualisation
   */
  if(matrixChannel != NULL){
    switch (index) {
      case 0:
        this->visualisation = new TextureVisualisation(matrixChannel, colorPalette, this);
        break;
      case 1: //TODO TESTTEST
        this->visualisation = new LandscapeVisualisation(matrixChannel, colorPalette, this);
//        visualisation->resize(300, 300);
        break;
      case 2:
        this->visualisation = new BarVisualisation(matrixChannel, colorPalette, this);
        break;
    }
    visLayout->addWidget(visualisation, Qt::AlignLeft);
    visLayout->addWidget(colorPalette, Qt::AlignHCenter);
    visLayout->addWidget(optionWidget, Qt::AlignRight);
  }
  /*
   * change vector visualisation
   */
  if(vectorChannel != NULL){
    switch (index){
      case 0:
        this->visualisation = new TextureVisualisation(vectorChannel, colorPalette, this);
        break;
      case 1:
        this->visualisation = new VectorPlotVisualisation(vectorChannel, colorPalette, this);
        break;
    }
    visLayout->addWidget(visualisation, Qt::AlignLeft);
    visLayout->addWidget(colorPalette, Qt::AlignHCenter);
    visLayout->addWidget(optionWidget, Qt::AlignRight);
  }
  updateViewableChannels();
//  setOptions();
  repaint();
}

void VisualiserSubWidget::switchVisMode(QAction * action){
  if(action->text().contains("Tex")){
    switchVisMode(0);
    return;
  }
  if(action->text().contains("Landscape")){
    switchVisMode(1);
    return;
  }
  if(action->text().contains("Bar")){
    switchVisMode(2);
    return;
  }
  if(action->text().contains("VectorPlot")){
    switchVisMode(1);
    return;
  }
}

void VisualiserSubWidget::showOptions(QAction *action){
  //TODO resize and add/remove optionswidget
  if(!optionsShown){
//    visLayout->addWidget(optionWidget, Qt::AlignRight);
//    visLayout->update();
    resize(width() + optionWidget->width(), height());
    optionWidget->show();
    optionsShown = true;
  }else{
    optionWidget->hide();
    resize(width() - optionWidget->width(), height());
//    visLayout->removeWidget(optionWidget);
    optionsShown = false;
  }
}

void VisualiserSubWidget::setOptions(){ //obsolete
//  optionLayout = new QVBoxLayout;
//  optionLayout->setContentsMargins(0,0,0,0);
//  if(visMode == 0){
//    optionLayout->addWidget(optionWidget);
//
//  }
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
  return visMode;
//  return vizChoice->currentIndex();
}
QString VisualiserSubWidget::getMode(){
  if(matrixChannel != 0) return (QString) "matrix";
  else return (QString) "vector";
}

void VisualiserSubWidget::closeEvent(QCloseEvent * event){
  emit windowClosed(this);  //--> configFile
  emit sendQuit();
  event->accept();
}

QSize VisualiserSubWidget::getSize(){
  if(optionsShown) return QSize( ( width() - optionWidget->width() ), height());
  else return size();
}

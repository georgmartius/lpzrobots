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
 *   $Log$
 *   Revision 1.6  2010-06-30 11:39:04  robot14
 *   removed VectorPlotChannel specs
 *
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
  this->colorPalette = new ColorPalette(this);
  optionWidget = colorPalette->makeConfigBox();
  optionWidget->setParent(this);
  int maxX = this->matrixChannel->getDimension(0);
  int maxY = this->matrixChannel->getDimension(1);
  if(dynamic_cast<VectorPlotChannel*> (matrixChannel) == 0){
  for (int i = 0; i < maxX; i++) { //push back all MatrixElementPlotChannel for update
    if( debug) cout << "here" << endl;
      for (int j = 0; j < maxY; j++) {
        addPlotChannel(this->matrixChannel->getChannel(i, j));
      }
    }
  } else {
    if( debug) cout << "here" << endl;
    for (int i = 0; i < maxX; i++) {
      addPlotChannel(dynamic_cast<VectorPlotChannel*> (matrixChannel)->getChannel(i));
    }
    addVectorOptions();
  }
  this->visualisation = new TextureVisualisation(channel, colorPalette, this); //default visualisation
  initGui();
  if( !(x == 0 && y == 0 && width == 0 && heigt == 0)){
    resize(width,heigt);
      move(x, y);
  }
  if( cPFilePath != "" && cPFilePath != 0)
      colorPalette->loadStopListFromFile(cPFilePath);
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
  connect(optionMenu, SIGNAL(triggered(QAction *)), this, SLOT(toggleOptions(QAction*)));
  menuBar->addMenu(optionMenu);

//  QHBoxLayout *mainLayout = new QHBoxLayout;
//  mainLayout->addWidget(vizChoice); obsolete
  visLayout = new QHBoxLayout();
  visLayout->setContentsMargins(0,0,0,0);
  visLayout->setSpacing(0);
  visLayout->addWidget(visualisation, Qt::AlignLeft);
  visLayout->addWidget(colorPalette, Qt::AlignHCenter);
  visLayout->addWidget(optionWidget, Qt::AlignRight);
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
  if (visualisation != NULL) visualisation->update();  //TODO when vis is null segfault!
  update();
}

void VisualiserSubWidget::initVisTypes(){
   QMenu *visMenu = new QMenu(tr("&vis mode"), this);
  /*
   * visualisation modi
   */
  visMenu->addAction(tr("&Tex"));
  visMenu->addAction(tr("&Landscape"));
  visMenu->addAction(tr("&Bar"));
  if (dynamic_cast<VectorPlotChannel*> (matrixChannel) != 0)
    visMenu->addAction(tr("&VectorPlot"));

  menuBar->addMenu(visMenu);
  //connect
//  connect(vizChoice, SIGNAL(activated(int)), this, SLOT(switchVisMode( int)));
  connect(visMenu, SIGNAL(triggered(QAction *)), this, SLOT(switchVisMode(QAction*)));
}

void VisualiserSubWidget::switchVisMode(int index){
  if (debug) cout << "in switchVisMode i: " << index << endl;

  visMode = index;

  visLayout->removeWidget(visualisation);
  delete visualisation;
  /*
   * change visualisation
   */
  switch (index) {
    case 0:
      this->visualisation = new TextureVisualisation(matrixChannel, colorPalette, this);
      break;
    case 1:
      this->visualisation = new LandscapeVisualisation(matrixChannel, colorPalette, this);
      break;
    case 2:
      this->visualisation = new BarVisualisation(matrixChannel, colorPalette, this);
      break;
    case 3:  //only choosable for vectors
      this->visualisation = new VectorPlotVisualisation(matrixChannel, colorPalette, this);
      break;
  }

  visLayout->insertWidget(0, visualisation, Qt::AlignLeft);
  updateViewableChannels();
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
    switchVisMode(3);
    return;
  }
}

void VisualiserSubWidget::toggleOptions(QAction *){
  if(!optionsShown){
    resize(width() + optionWidget->width(), height());
    optionWidget->show();
    optionsShown = true;
  }else{
    optionWidget->hide();
    resize(width() - optionWidget->width(), height());
    optionsShown = false;
  }
}



void VisualiserSubWidget::addVectorOptions(){
  /*
   * set up vector option widget
   */
  QWidget *vectorOpt = new QWidget(this);
  QVBoxLayout *vBoxLayout = new QVBoxLayout();
  vBoxLayout->setContentsMargins(0,0,0,0);

  QLabel *label = new QLabel("Buffersize");
  label->setFont(QFont("Arial", 9));
  vBoxLayout->addWidget(label);

  QHBoxLayout *hBoxLayout = new QHBoxLayout();
  hBoxLayout->setContentsMargins(0,0,0,0);
  vectorBuffersizeSpinBox = new QSpinBox();
  vectorBuffersizeSpinBox->setRange(1,1000);
  vectorBuffersizeSpinBox->setValue(40);
  hBoxLayout->addWidget(vectorBuffersizeSpinBox);

  QPushButton *setButton = new QPushButton("set");
  connect(setButton, SIGNAL(clicked()), this, SLOT(changeBufferSize()));
  hBoxLayout->addWidget(setButton);

  vBoxLayout->addLayout(hBoxLayout);
  vectorOpt->setLayout(vBoxLayout);

  /*
   * put optionwidget and vectoroptionwidget into one widget
   * this is needed for hiding
   */
  QWidget *temp = optionWidget;
  optionWidget = new QWidget();
  QVBoxLayout *layout = new QVBoxLayout();
  layout->setContentsMargins(0,0,0,0);
  layout->addWidget(temp);
  layout->addWidget(vectorOpt);
  optionWidget->setLayout(layout);
  optionWidget->setFixedWidth(200);
  optionWidget->setParent(this);
}
//only possible for vectors
void VisualiserSubWidget::changeBufferSize(){
  if (debug) cout << "VisualiserSubWidget::changeBufferSize()" << endl;
  if(dynamic_cast<VectorPlotChannel*> (matrixChannel) == 0) return;
  dynamic_cast<VectorPlotChannel*> (matrixChannel)->setBufferSize(vectorBuffersizeSpinBox->value());
  if( debug) cout << "test" << endl;
  switchVisMode(visMode);
}

QString VisualiserSubWidget::getChannelName(){
  return QString(matrixChannel->getChannelName().c_str());
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
  if(dynamic_cast<VectorPlotChannel*> (matrixChannel) == 0) return (QString) "matrix";
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

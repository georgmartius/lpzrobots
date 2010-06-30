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
 *                                                                         *
 *   $Log$
 *   Revision 1.6  2010-06-30 11:39:04  robot14
 *   removed VectorPlotChannel specs
 *
 *   Revision 1.5  2010/03/30 13:21:10  robot14
 *   added vector support
 *
 *   Revision 1.4  2009/10/22 15:53:08  robot14
 *   first version of texture visualisation
 *
 *   Revision 1.3  2009/10/14 12:22:31  robot14
 *   *** empty log message ***
 *
 *   Revision 1.2  2009/10/02 15:25:40  robot14
 *   filters, main app - not finished yet
 *
 *   Revision 1.1  2009/08/13 13:14:05  robot14
 *   first version
 *
 *                                                                         *
 ***************************************************************************/
#include "MatrixVisualizer.h"

#include <QString>
#include <iostream>

using namespace std;


MatrixVisualizer::MatrixVisualizer(QWidget *parent) : AbstractRobotGUI(parent) {

  pipe_reader = new SimplePipeReader();
//  vector_filter = new VectorPipeFilter(pipe_reader);
  matrix_filter = new MatrixPipeFilter(pipe_reader);

  // if '#QUIT' matrixvis quits too
  connect(pipe_reader, SIGNAL(finished()), this, SLOT(close()));

  pipe_reader->start();
  if (debug) cout << "Here I AM!!!" << endl;

  channelList = matrix_filter->getChannelList();
  matrices = matrix_filter->getMatrixChannels();
  vectors = matrix_filter->getVectorChannels();

  initGui();
  config = new configFile();
  config->load(this);
  connect(this, SIGNAL(sendQuit()), config, SLOT(doQuit()));
}

MatrixVisualizer::~MatrixVisualizer() {
}

void MatrixVisualizer::initGui() {

  main_layout = new QVBoxLayout;
  main_layout->addLayout(makeButtons());

  setLayout(main_layout);
  // nach pack() ähnlichem gucken!
  resize(150,150); //adjustSize ();
  show();
}

QHBoxLayout* MatrixVisualizer::makeButtons(){

  QHBoxLayout *chooseBoxL = new QHBoxLayout;
  QVBoxLayout *matrBox = new QVBoxLayout;
  QVBoxLayout *vecBox = new QVBoxLayout;

  visButtons = new QButtonGroup();
  visButtons->setExclusive(false);

  int id = 0;
  for(vector<MatrixPlotChannel*>::iterator i = matrices.begin(); i != matrices.end(); i++){
    QString qs( (*i)->getChannelName().c_str() );
    QPushButton *pB = new QPushButton(qs);
    matrBox->addWidget(pB);
    visButtons->addButton(pB);
    id++;
  }
  if(debug) cout << "size of Vectors: " << vectors.size() << endl;
  for(vector<VectorPlotChannel*>::iterator i = vectors.begin(); i != vectors.end(); i++){
    if(debug) cout << "addButton for vector" << endl;
      QString qs( (*i)->getChannelName().c_str() );
      QPushButton *pB = new QPushButton(qs);
      vecBox->addWidget(pB);
      visButtons->addButton(pB);
      id++;
    }

  connect(visButtons, SIGNAL(buttonClicked(QAbstractButton *)), this, SLOT(visualize(QAbstractButton *)));

  chooseBoxL->addLayout(matrBox);
  chooseBoxL->addLayout(vecBox);

  return chooseBoxL;
}

void MatrixVisualizer::visualize(QAbstractButton * button){
  // contains the button to a matrix or a vector
  QString name = button->text();
  VectorPlotChannel *vectorPlotChannel = getVectorPlotChannel(name);
  VisualiserSubWidget *vis;
  if(vectorPlotChannel == 0){
    vis = new VisualiserSubWidget(getMatrixPlotChannel(name));
  }else
    vis = new VisualiserSubWidget(vectorPlotChannel);
  config->newOpenedWindow(vis);
  vis->show();
  connectWindowForUpdate(vis);
}

void MatrixVisualizer::linkChannels() { //not needed
//  for (std::list<AbstractPlotChannel*>::iterator i=channelList.begin(); i!=channelList.end(); i++){
//    //widget->addPlotChannel(i*)
//  }

}

void MatrixVisualizer::connectWindowForUpdate(VisualiserSubWidget *vis){
  connect( pipe_reader, SIGNAL(newData()), vis, SLOT(updateViewableChannels()), Qt::DirectConnection);
}

VectorPlotChannel* MatrixVisualizer::getVectorPlotChannel(QString name){
  for(vector<VectorPlotChannel*>::iterator i = vectors.begin(); i != vectors.end(); i++){
      QString vecName( (*i)->getChannelName().c_str() );
      if(vecName == name) return (*i);
    }
  return 0;
}

MatrixPlotChannel* MatrixVisualizer::getMatrixPlotChannel(QString name){
  for(vector<MatrixPlotChannel*>::iterator i = matrices.begin(); i != matrices.end(); i++){
      QString matName( (*i)->getChannelName().c_str() );
      if(matName == name) return (*i);
    }
  return 0;
}

void MatrixVisualizer::closeEvent(QCloseEvent * event){
  if(debug) cout << "MatrixViz::closeEvent" << endl;
  emit sendQuit();
  event->accept();
}

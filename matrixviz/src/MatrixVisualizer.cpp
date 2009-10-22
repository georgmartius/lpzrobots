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
 *   Revision 1.4  2009-10-22 15:53:08  robot14
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

//#include <QProgressBar>
#include <QString>
#include "VisualiserSubWidget.h"
//#include "timer.h"

#include <iostream>

using namespace std;

MatrixVisualizer::MatrixVisualizer(QWidget *parent) : AbstractRobotGUI(parent) {

  pipe_reader = new SimplePipeReader();
  matrix_filter = new MatrixPipeFilter(pipe_reader);
  help = 1;

  pipe_reader->start();
  cout << "Here I AM!!!" << endl;

  channelList = matrix_filter->getChannelList();
  matrices = matrix_filter->getMatrixChannels();
  initGui();
}

MatrixVisualizer::~MatrixVisualizer() {
}

void MatrixVisualizer::initGui() {

  main_layout = new QVBoxLayout;
  main_layout->addLayout(makeButtons());

  //TODO


  setLayout(main_layout);
  // nach pack() ähnlichem gucken!
  resize(150,150); //adjustSize ();
}

QHBoxLayout* MatrixVisualizer::makeButtons(){

  //QGroupBox *gb = new QGroupBox(QString("View-Settings"));
  QHBoxLayout *chooseBoxL = new QHBoxLayout;
  QVBoxLayout *matrBox = new QVBoxLayout;
  //QVBoxLayout *vecBox = new QVBoxLayout;

  //matChoice = new QComboBox();
  //vizChoice = new QComboBox();
  //QPushButton *button1 = new QPushButton("One");
  visButtons = new QButtonGroup();
  visButtons->setExclusive(false);

  int id = 0;
  for(vector<MatrixPlotChannel*>::iterator i = matrices.begin(); i != matrices.end(); i++){
    QString qs( (*i)->getChannelName().c_str() );
    //matChoice->addItem(qs);   //TODO
    QPushButton *pB = new QPushButton(qs);
    matrBox->addWidget(pB);
    visButtons->addButton(pB);
    //connect(visButtons, SIGNAL(buttonClicked(QAbstractButton *)), this, SLOT(visualize(QAbstractButton *)));
    connect(visButtons, SIGNAL(buttonClicked(QAbstractButton *)), this, SLOT(visualize(QAbstractButton *)));
    id++;
  }

  //chooseBoxL->addWidget(matChoice);
  //chooseBoxL->addWidget(vizChoice);
  chooseBoxL->addLayout(matrBox);

  //gb->setLayout(chooseBoxL);
  return chooseBoxL;
}

void MatrixVisualizer::visualize(QAbstractButton * button){
  if ( help == visButtons->buttons().size()){
    help = 1;
    int id = visButtons->buttons().indexOf(button);
    VisualiserSubWidget *vis = new VisualiserSubWidget(matrices.at(id));

    vis->show();
    connect( pipe_reader, SIGNAL(newData()), vis, SLOT(updateViewableChannels()), Qt::DirectConnection);

//    Timer *timer = new Timer(vis);
//    timer->start();

  }else help++;
}

void MatrixVisualizer::linkChannels() {


//  for (std::list<AbstractPlotChannel*>::iterator i=channelList.begin(); i!=channelList.end(); i++){
//    //widget->addPlotChannel(i*)
//  }


}


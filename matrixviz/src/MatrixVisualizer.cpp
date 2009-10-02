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
 *   Revision 1.2  2009-10-02 15:25:40  robot14
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

#include <iostream>

using namespace std;

MatrixVisualizer::MatrixVisualizer(QWidget *parent) : AbstractRobotGUI(parent) {
  pipe_reader = new SimplePipeReader();
  matrix_filter = new MatrixPipeFilter(pipe_reader);

  pipe_reader->start();
  initGui();
  cout << "Here I AM!!!" << endl;

  channelList = matrix_filter->getChannelList();

}

MatrixVisualizer::~MatrixVisualizer() {
}

void MatrixVisualizer::initGui() {

  main_layout = new QVBoxLayout;
  main_layout->addWidget(makeChooseBox());

  //TODO


  setLayout(main_layout);
  resize(200,200);
}

QGroupBox* MatrixVisualizer::makeChooseBox(){

  QGroupBox *gb = new QGroupBox(QString("View-Settings"));
  QHBoxLayout *chooseBoxL = new QHBoxLayout;

  matChoice = new QComboBox();
  vizChoice = new QComboBox();

  for(list<AbstractPlotChannel*>::iterator i = channelList.begin(); i != channelList.end(); i++){
    QString qs( (*i)->getChannelName().c_str() );
    matChoice->addItem(qs);   //TODO
  }

  chooseBoxL->addWidget(matChoice);
  chooseBoxL->addWidget(vizChoice);

  gb->setLayout(chooseBoxL);
  return gb;
}

void MatrixVisualizer::linkChannels() {


//  for (std::list<AbstractPlotChannel*>::iterator i=channelList.begin(); i!=channelList.end(); i++){
//    //widget->addPlotChannel(i*)
//  }


}


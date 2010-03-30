/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.1  2010-03-30 13:18:26  robot14
 *  first version
 *
 *                                                                         *
 *                                                                         *
 **************************************************************************/

#include "VectorPlotChannel.h"
#include "cassert"
#include <iostream>

using namespace std;

VectorPlotChannel::VectorPlotChannel(string name) : GroupPlotChannel(name) {
  // TODO Auto-generated constructor stub
}

VectorPlotChannel::~VectorPlotChannel() {
  // TODO Auto-generated destructor stub
}

double VectorPlotChannel::getValue(int num)
{
  return getChannel(num)->getValue(0);
}

double VectorPlotChannel::getValue(int num, int t)
{
  return getChannel(num)->getValue(t);
}

int VectorPlotChannel::getSize(){
  return channelsOfGroup.size();
}

int VectorPlotChannel::getBufferSize(){
  VectorElementPlotChannel* firstElementChannel = dynamic_cast<VectorElementPlotChannel*> (channelsOfGroup.front());
  if (firstElementChannel != 0) return firstElementChannel->getSize();
  else return 0;
}

void VectorPlotChannel::setBufferSize(int newSize){
  for (list<AbstractPlotChannel*>::iterator i = channelsOfGroup.begin(); i != channelsOfGroup.end(); i++){
    VectorElementPlotChannel* elementChannel = dynamic_cast<VectorElementPlotChannel*> (*i);
    if( elementChannel != 0) elementChannel->changeSize(newSize);
  }
}

VectorElementPlotChannel* VectorPlotChannel::getChannel(int num){
  return dynamic_cast<VectorElementPlotChannel*> (at(num));
}

void VectorPlotChannel::addElement(VectorElementPlotChannel* gc){
  if (debug) cout << "VectorPlotChannel::addElement" << endl;
  channelsOfGroup.push_back(gc);
}

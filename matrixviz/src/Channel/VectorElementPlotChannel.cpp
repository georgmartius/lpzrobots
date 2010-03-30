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

 *                       *
 *                                                                         *
 **************************************************************************/

#include "VectorElementPlotChannel.h"
#include <iostream>

using namespace std;

VectorElementPlotChannel::VectorElementPlotChannel(string name) : AbstractPlotChannel(name), bufferSize(40), ringBufferIndex(0) {
  if (debug) cout << "VEPC konstruktor von: " << name << endl;
  ringBuffer.resize(bufferSize, 0.);
}

VectorElementPlotChannel::~VectorElementPlotChannel() {
}

void VectorElementPlotChannel::setValue(double v){
  if (debug) cout << "in VectorElementPlotChannel::setValue = " << v << endl;
  ringBuffer.at(ringBufferIndex++) = v;
  if (ringBufferIndex == bufferSize)
    ringBufferIndex = 0;
}

double VectorElementPlotChannel::getValue(){
  return ringBuffer.at(ringBufferIndex);
}

double VectorElementPlotChannel::getValue(int time){
  return ringBuffer.at((ringBufferIndex + time) % bufferSize);
}

void VectorElementPlotChannel::changeSize(int newSize){
  if(debug) cout << "in VectorElementPlotChannel::changeSize" << endl;
  if (newSize == bufferSize) return;
  if(newSize > bufferSize){
    ringBuffer.resize(newSize, 0.);
    for ( int i = 0; i < ringBufferIndex; i++){
      ringBuffer.at((bufferSize + i) % newSize) = ringBuffer.at(i);
      ringBuffer.at(i) = 0.;
    }
    ringBufferIndex = (ringBufferIndex + bufferSize) % newSize;
  }else{
    if(ringBufferIndex < newSize){
      for(int i = 1; i < (newSize - ringBufferIndex); i++)
        ringBuffer.at(ringBufferIndex + i) = ringBuffer.at(ringBufferIndex + bufferSize - newSize + i);
    }else{
      for(int i = 0; i < newSize; i++)
        ringBuffer.at(i) = ringBuffer.at(i + ringBufferIndex - newSize);
      ringBufferIndex -= (newSize - 1);
    }
  }
  ringBuffer.resize(newSize, 0.);
  bufferSize = newSize;
}

int VectorElementPlotChannel::getSize(){
  return bufferSize;
}

int VectorElementPlotChannel::getIndex(){
  return ringBufferIndex;
}

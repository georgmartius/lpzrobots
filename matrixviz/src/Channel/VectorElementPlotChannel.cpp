/***************************************************************************
 *   Copyright (C) 2008-2011 LpzRobots development team                    *
 *    Antonia Siegert (original author)                                  *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/

#include "VectorElementPlotChannel.h"
#include <iostream>

using namespace std;

VectorElementPlotChannel::VectorElementPlotChannel(string name) : AbstractPlotChannel(name), bufferSize(64), ringBufferIndex(0) {
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
  if (debug) cout << "::getValue: time: " << time << " index: " << ringBufferIndex << endl;
  int position = ringBufferIndex - time -1;
  if (debug) cout << position << endl;
  if ( position < 0) position = bufferSize + position;
  if (debug) cout << position << endl;
  return ringBuffer.at(position);
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
      for(int i = 0; i < (newSize - ringBufferIndex); i++)
        ringBuffer.at(ringBufferIndex + i) = ringBuffer.at(ringBufferIndex + bufferSize - newSize + i);
    }else{
      for(int i = 0; i < newSize; i++)
        ringBuffer.at(i) = ringBuffer.at(i + ringBufferIndex - newSize -1);
      ringBufferIndex = 0;
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

/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
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
#include "lyapunov.h"
#include "stl_adds.h"

#include <iostream>
#include <algorithm>

using namespace std;
using namespace matrix;


Lyapunov::Lyapunov(){
  t = 0;
  buffersize = 0;
  buffer = 0;
  invbuffer = 0;
}

Lyapunov::~Lyapunov(){
  if(buffer) delete[] buffer;
  if(invbuffer) delete[] invbuffer;
  FOREACH(Horizons, horizons, h){
    delete (h->second);
  }
}

void Lyapunov::init(const std::list<int>& hs, int dim){
  list<int> myhs = hs;
  if(myhs.empty()) myhs += 0; // add infinit horizon
  list<int>::const_iterator it = max_element(myhs.begin(), myhs.end());
  buffersize = *it;
  if(buffersize < 1) buffersize = 1;
  buffer = new Matrix[buffersize];
  invbuffer = new Matrix[buffersize];
  FOREACHC(list<int>, myhs, h){
    horizons[*h]= new SlidingMatrix(dim, *h);
  }

}

void Lyapunov::step(const Matrix& jacobi){
  buffer[t%buffersize]=jacobi;
  invbuffer[t%buffersize]=jacobi^(-1);
  FOREACH(Horizons, horizons, h){
    h->second->step(t, buffer, invbuffer, buffersize);
  }
  t++;
}

Lyapunov::SlidingMatrix::SlidingMatrix(int dim, int horizon)
      : horizon(horizon), M(dim,dim), Exp(dim,1) {
  M.toId();
};

void Lyapunov::SlidingMatrix::step(int t, const matrix::Matrix* buffer,
                                   const matrix::Matrix* invbuffer, int buffersize){
  if(horizon <= 0) { // infinite horizon, we count the length negatively
    horizon--;
  }else{             // for a finite horizon we have to divide by the old matrix
    int h = t-horizon;
    if(h>=0){
      M=invbuffer[h%buffersize]*M; // from leftside with inverse at beginning of window
    }
  }
  M=M*buffer[t%buffersize]; // current matrix
}

const Matrix& Lyapunov::getLyapunovMatrix(int horizon){
  Horizons::iterator h = horizons.find(horizon);
  if(h != horizons.end()){
    return h->second->M;
  }else{
    cerr << "Lyapunov: cannot find horizon " << horizon << endl;
    return horizons.begin()->second->M;
  }
}

const Matrix& Lyapunov::getLyapunovExp(int horizon){
  Horizons::iterator h = horizons.find(horizon);
  if(h != horizons.end()){
    return h->second->Exp;
  }else{
    cerr << "Lyapunov: cannot find horizon " << horizon << endl;
    return horizons.begin()->second->Exp;
  }
}

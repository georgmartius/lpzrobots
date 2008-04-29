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
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2008-04-29 07:39:24  guettler
 *   -interfaces moved to selforg/utils
 *   -added addInspectableValue and addInspectableMatrix
 *   -methods getInternalParamNames and getInternalParams do not need to be
 *   overloaded anymore, use addInspectableValue and addInspectableMatrix
 *   instead (preferred)
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#include "inspectable.h"
#include "controller_misc.h"

Inspectable::~Inspectable(){
  // temporary debug
  std::cout << "eek...an instance of Inspectable is destructed!" << std::endl;
}

Inspectable::Inspectable() {}



Inspectable::iparamkeylist Inspectable::getInternalParamNames() const {
  iparamkeylist keylist;
  for(iparammap::const_iterator it=mapOfValues.begin(); it != mapOfValues.end(); it++){
    keylist+=(*it).first;
  }
  for(imatrixmap::const_iterator m=mapOfMatrices.begin(); m != mapOfMatrices.end(); m++){
    keylist+=store4x4AndDiagonalFieldNames(*((*m).second),(*m).first);
  }
  return keylist;
}


Inspectable::iparamvallist Inspectable::getInternalParams() const {
  iparamvallist vallist;
  for(iparammap::const_iterator it=mapOfValues.begin(); it != mapOfValues.end(); it++){
    vallist+=*(*it).second;
  }
  for(imatrixmap::const_iterator m=mapOfMatrices.begin(); m != mapOfMatrices.end(); m++){
    vallist+=store4x4AndDiagonal(*((*m).second));
  }
  return vallist;
}


Inspectable::ilayerlist Inspectable::getStructuralLayers() const {
  return std::list<ILayer>();
}

Inspectable::iconnectionlist Inspectable::getStructuralConnections() const {
  return std::list<IConnection>();
}

void Inspectable::addInspectableValue(const iparamkey& key, iparamval* val){
  mapOfValues[key]=val;
}

void Inspectable::addInspectableMatrix(const iparamkey& key, matrix::Matrix* m) {
  mapOfMatrices[key]=m;
}


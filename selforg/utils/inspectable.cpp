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
 *   Revision 1.6  2009-06-29 13:24:13  robot12
 *   add function getInternalParamsPtr for new class inspectableproxy
 *
 *   Revision 1.5  2008/11/17 14:59:09  martius
 *   removed debug print
 *
 *   Revision 1.4  2008/11/14 09:16:15  martius
 *   small things
 *
 *   Revision 1.3  2008/08/01 14:42:03  guettler
 *   we try the trip to hell! make selforg AVR compatible...good luck (first changes)
 *
 *   Revision 1.2  2008/04/29 09:55:30  guettler
 *   -class uses now a list of pairs instead of a map
 *   -debug printouts removed
 *
 *   Revision 1.1  2008/04/29 07:39:24  guettler
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

Inspectable::~Inspectable(){}

Inspectable::Inspectable() {}


#ifndef AVR

Inspectable::iparamkeylist Inspectable::getInternalParamNames() const {
  iparamkeylist keylist;
  fprintf(stderr,"Start");
  FOREACHC(iparampairlist, mapOfValues, it){
    keylist+=(*it).first;
  }
  for(imatrixpairlist::const_iterator m=mapOfMatrices.begin(); m != mapOfMatrices.end(); m++){
    keylist+=store4x4AndDiagonalFieldNames(*((*m).second),(*m).first);
  }
  return keylist;
}


Inspectable::iparamvallist Inspectable::getInternalParams() const {
  iparamvallist vallist;
  for(iparampairlist::const_iterator it=mapOfValues.begin(); it != mapOfValues.end(); it++){
    vallist+=*(*it).second;
  }
  for(imatrixpairlist::const_iterator m=mapOfMatrices.begin(); m != mapOfMatrices.end(); m++){
    vallist+=store4x4AndDiagonal(*((*m).second));
  }
  return vallist;
}

Inspectable::iparamvalptrlist Inspectable::getInternalParamsPtr() const {
	  iparamvalptrlist vallist;
	  for(iparampairlist::const_iterator it=mapOfValues.begin(); it != mapOfValues.end(); it++){
	    vallist+=(*it).second;
	  }
	  // be carefully matrix will be ignored
	  return vallist;
}


Inspectable::ilayerlist Inspectable::getStructuralLayers() const {
  return std::list<ILayer>();
}

Inspectable::iconnectionlist Inspectable::getStructuralConnections() const {
  return std::list<IConnection>();
}

void Inspectable::addInspectableValue(const iparamkey key, iparamval* val){
  mapOfValues+=iparampair(key,val);
}

void Inspectable::addInspectableMatrix(const iparamkey key, matrix::Matrix* m) {
  mapOfMatrices+=imatrixpair(key,m);
}

#else

// TODO: implement 4x4store functions for matrices, but not essential (simply do not use them for avr controllers)

Inspectable::iparamkeylist Inspectable::getInternalParamNames() const {
	return ikeylist;
}


Inspectable::iparamvallist Inspectable::getInternalParams() const {
	return ivallist;
}


// TODO: implement getStructuralLayers and getStructuralConnections

void Inspectable::addInspectableValue(const iparamkey key, iparamval* val){
	if (numberParameters<maxNumberEntries) {
		ikeylist[numberParameters]=key;
		ivallist[numberParameters]=val;
	}
}

// TODO: implement addInspectableMatrix
void Inspectable::addInspectableMatrix(const iparamkey key, matrix::Matrix* m) {}


#endif



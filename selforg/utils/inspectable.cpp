/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
 *    joergweide84@aol.com (robot12)                                       *
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
 *   Revision 1.16  2011-10-13 14:36:29  martius
 *   stl_adds removeElement
 *   zoo: adding and removing robots
 *
 *   Revision 1.15  2011/03/21 17:46:12  guettler
 *   enhanced inspectable interface:
 *   - support for inspectable children of a inspectable
 *   - some new helper functions
 *
 *   Revision 1.14  2010/10/18 15:08:35  martius
 *   matrices are now explicitly const (as it should be)
 *
 *   Revision 1.13  2010/04/28 07:57:41  guettler
 *   getInternalParams: additional check if m->first is valid
 *
 *   Revision 1.12  2009/10/14 09:58:42  martius
 *   added support for description strings that are exported using the infolines
 *
 *   Revision 1.11  2009/08/10 07:37:48  guettler
 *   -Inspectable interface now supports to add infoLines itself.
 *    These lines are then outprinted line by line to the PlotOption once,
 *    preceded by a #I.
 *   -Restart functionality of PlotOptionEngine added (e.g. closePipes(), reInit()).
 *
 *   Revision 1.10  2009/08/07 09:32:16  martius
 *   changed order between matrices and params
 *
 *   Revision 1.9  2009/08/05 22:49:24  martius
 *   removed AVR stuff
 *
 *   Revision 1.8  2009/08/05 08:19:53  martius
 *   addInspectableMatrix allows to specify optionally whether all or only 4x4+Diagonal is used
 *
 *   Revision 1.7  2009/07/15 13:01:15  robot12
 *   one bugfixe
 *
 *   Revision 1.6  2009/06/29 13:24:13  robot12
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
#include "stl_adds.h"

Inspectable::~Inspectable(){}

Inspectable::Inspectable(const iparamkey& name) : name(name), parent(0) {}


Inspectable::iparamkeylist Inspectable::getInternalParamNames() const {
  iparamkeylist keylist;
  FOREACHC(imatrixpairlist, mapOfMatrices, m){
    if(m->second.first->isVector()){
      keylist+=storeVectorFieldNames(*(m->second.first), m->first);    
    } else {
      if(m->second.second)
	keylist+=store4x4AndDiagonalFieldNames(*(m->second.first), m->first);
      else 
	keylist+=storeMatrixFieldNames(*(m->second.first), m->first);    
    }
  }
  FOREACHC(iparampairlist, mapOfValues, it){
    keylist+=(*it).first;
  }
  return keylist;
}


Inspectable::iparamvallist Inspectable::getInternalParams() const {
  iparamvallist vallist;
  FOREACHC(imatrixpairlist, mapOfMatrices, m){
    if(m->second.first && (m->second.first->isVector() || !m->second.second)){    
      vallist+= m->second.first->convertToList();
    } else {
	vallist+=store4x4AndDiagonal(*(m->second.first));
    }      
  }
  FOREACHC(iparampairlist, mapOfValues, it){
    vallist+=*(it->second);
  }
  return vallist;
}

Inspectable::iparamvalptrlist Inspectable::getInternalParamsPtr() const {
  iparamvalptrlist vallist;
  FOREACHC(iparampairlist, mapOfValues, it){  
    vallist+=it->second;
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

void Inspectable::addInspectableValue(const iparamkey& key, iparamval* val, 
                                      const std::string& descr) {
  mapOfValues+=iparampair(key,val);
  if(!descr.empty())
    addInspectableDescription(key, descr);
}

void Inspectable::addInspectableMatrix(const iparamkey& key, const matrix::Matrix* m, 
                                       bool only4x4AndDiag, const std::string& descr) {
  mapOfMatrices+=imatrixpair(key, std::pair<const matrix::Matrix*, bool>(m, only4x4AndDiag) );
  if(!descr.empty())
    addInspectableDescription(key+"_", descr);
}

void Inspectable::addInspectableDescription(const iparamkey& key, const std::string& descr){
  addInfoLine("D " + key + " " + descr);
}


void Inspectable::addInfoLine(std::string infoLine)
{
  infoLineStringList.push_back(infoLine);
}


void Inspectable::addInfoLines(std::list<std::string> infoLineList)
{
  FOREACHC(std::list<std::string>, infoLineList, line)
  {
    infoLineStringList.push_back(*line);
  }
}

const std::list<std::string>& Inspectable::getInfoLines() const
{
  return infoLineStringList;
}

void Inspectable::removeInfoLines() {
  infoLineStringList.clear();
}


void Inspectable::addInspectable(Inspectable* insp) {
  listOfInspectableChildren.push_back(insp);
  insp->parent=this;
}

void Inspectable::removeInspectable(Inspectable* insp) {
  removeElement(listOfInspectableChildren, insp);
  insp->parent=0;
}

void Inspectable::setNameOfInspectable(const iparamkey& _name) {
    name = _name;
}


const Inspectable::iparamkey Inspectable::getNameOfInspectable() const {
    return name;
}


const Inspectable::inspectableList& Inspectable::getInspectables() const {
  return listOfInspectableChildren;
}


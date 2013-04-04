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

void Inspectable::addInspectableValue(const iparamkey& key, iparamval const* val,
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


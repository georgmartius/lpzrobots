/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   Revision 1.1  2005-09-27 11:03:34  fhesse
 *   sensorbank added
 *
 *                                                                         *
 ***************************************************************************/

#include <ode/ode.h>

#include "raysensorbank.h"

RaySensorBank::RaySensorBank(){
  bank.resize(0);
  initialized=false;
};

RaySensorBank::~RaySensorBank(){
};

void RaySensorBank::init(dSpaceID parent_space, rayDrawMode drawmode){
  sensor_space = dSimpleSpaceCreate ( parent_space );
  initialized=true;  
}; 

unsigned int RaySensorBank::registerSensor(RaySensor* raysensor, dBodyID body, 
					   const Position& pos, const dMatrix3 rotation, double range){
  raysensor->init(sensor_space, body, pos, rotation, range);
  bank.push_back(raysensor);  
  return bank.size();
};

void RaySensorBank::reset(){
  for (unsigned int i=0; i<bank.size(); i++){
    bank[i]->reset();
  }
};  
  
bool RaySensorBank::sense(dGeomID object){
  bool sth_sensed=false;
  for (unsigned int i=0; i<bank.size(); i++){
    if (bank[i]->sense(object)){
      sth_sensed=true;
    }
  }
  return sth_sensed;
};

double RaySensorBank::get(unsigned int index){
  //Todo: assert(index<bank.size())
  return bank[index]->get();
};

int RaySensorBank::get(double* sensorarray, unsigned int start, unsigned int array_size){
  int counter=0;
  for(unsigned int i=start; (i<array_size) && ((i-start)<bank.size()); i++){
    sensorarray[i]=bank[i-start]->get();
    counter++;
  }
  return counter;
};

dSpaceID RaySensorBank::getSpaceID(){
  return sensor_space;
};

void RaySensorBank::draw(){
  for (unsigned int i=0; i<bank.size(); i++){
    bank[i]->draw();
  }
};
  





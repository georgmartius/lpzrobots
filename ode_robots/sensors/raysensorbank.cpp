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

#include <assert.h>
#include <ode-dbl/ode.h>
#include <osg/Matrix>

#include "raysensorbank.h"

using namespace osg;

namespace lpzrobots {

  RaySensorBank::RaySensorBank()
  {
    initialized=false;
  };

  RaySensorBank::~RaySensorBank()
  {
    clear();

    if (initialized)
      //      this->odeHandle.deleteSpace(); this is automatically done if the parent space is deleted

    initialized = false;
  };

  void RaySensorBank::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle)
  {
    this->odeHandle = odeHandle;
    this->osgHandle = osgHandle;

    this->odeHandle.createNewSimpleSpace(odeHandle.space, true);
    initialized=true; 
  }; 

  unsigned int RaySensorBank::registerSensor(RaySensor* raysensor, Primitive* body, 
					     const osg::Matrix& pose, float range,
					     RaySensor::rayDrawMode drawMode){
    raysensor->init(odeHandle, osgHandle, body, pose, range, drawMode);
    bank.push_back(raysensor);  
    return bank.size();
  };

  void RaySensorBank::reset(){
    for (unsigned int i=0; i<bank.size(); i++)
    {
      bank[i]->reset();
    }



  };  
  
  double RaySensorBank::get(unsigned int index){
    assert(index<bank.size());
    return bank[index]->get();
  };

  int RaySensorBank::get(double* sensorarray, unsigned int array_size){
    int counter=0;
    for(unsigned int i=0; (i<array_size) && (i<bank.size()); i++){
      sensorarray[i]=bank[i]->get();
      counter++;
    }
    return counter;
  };

  int RaySensorBank::getSensorNumber(){
    return bank.size();
  }

  void RaySensorBank::setRange(unsigned int index, float range){
    assert(index<bank.size());
    return bank[index]->setRange(range);    
  }

  void RaySensorBank::setRange(float range){
    for(unsigned int i=0; i<bank.size(); i++){
      bank[i]->setRange(range);
    }
  }


  dSpaceID RaySensorBank::getSpaceID(){
    return odeHandle.space;
  };

  void RaySensorBank::update(){
    for (unsigned int i=0; i<bank.size(); i++){
      bank[i]->update();
    }
  };

  // delete all registered sensors.
  void RaySensorBank::clear(){
    for (unsigned int i=0; i<bank.size(); i++)
    {
      if(bank[i])
        delete bank[i];      
    }
    bank.clear();
    //odeHandle.deleteSpace();
  }


}
  





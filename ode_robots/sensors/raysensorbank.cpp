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
 *   Revision 1.3  2006-07-14 12:23:43  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.2.4.4  2006/01/12 15:14:57  martius
 *   indentation and clear routine
 *
 *   Revision 1.2.4.3  2005/12/14 15:37:19  martius
 *   sensors are working with osg
 *
 *   Revision 1.2.4.2  2005/12/14 12:43:07  martius
 *   moved to osg
 *
 *   Revision 1.2.4.1  2005/12/13 18:11:53  martius
 *   sensors ported, but not yet finished
 *
 *   Revision 1.2  2005/09/27 13:59:26  martius
 *   ir sensors are working now
 *
 *   Revision 1.1  2005/09/27 11:03:34  fhesse
 *   sensorbank added
 *
 *                                                                         *
 ***************************************************************************/

#include <assert.h>
#include <ode/ode.h>
#include <osg/Matrix>

#include "raysensorbank.h"

using namespace osg;

namespace lpzrobots {

  RaySensorBank::RaySensorBank(){
    initialized=false;
  };

  RaySensorBank::~RaySensorBank(){
    clear();
  };

  void RaySensorBank::init(const OdeHandle& odeHandle, const OsgHandle& osgHandle){
    this->odeHandle = odeHandle;
    this->osgHandle = osgHandle;
    this->odeHandle.space = dSimpleSpaceCreate ( this->odeHandle.space );
    initialized=true; 
  }; 

  unsigned int RaySensorBank::registerSensor(RaySensor* raysensor, Primitive* body, 
					     const Matrix& pose, double range,
					     RaySensor::rayDrawMode drawMode){
    raysensor->init(odeHandle, osgHandle, body, pose, range, drawMode);
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
    for (unsigned int i=0; i<bank.size(); i++){
      if(bank[i]) delete bank[i];
    }
    bank.clear();
  }


}
  





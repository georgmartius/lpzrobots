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
 *   Revision 1.7  2009-05-11 17:03:07  martius
 *   minor substance change
 *
 *   Revision 1.6  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.5  2008/05/07 16:45:51  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.4  2008/04/23 07:17:16  martius
 *   makefiles cleaned
 *   new also true realtime factor displayed,
 *    warning if out of sync
 *   drawinterval in full speed is 10 frames, independent of the speed
 *
 *   Revision 1.3  2007/11/07 13:21:15  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.2  2007/09/06 18:47:59  martius
 *   createNewSimpleSpace used
 *
 *   Revision 1.1  2007/08/24 11:49:25  martius
 *   initial
 *
 *                                                                 *
 ***************************************************************************/

#include <ode/ode.h>
#include <assert.h>
#include <osg/Matrix>

#include "fourwheeled.h"
#include "irsensor.h"
#include "primitive.h"
#include "osgprimitive.h"

using namespace osg;
using namespace std;

namespace lpzrobots {

  FourWheeled::FourWheeled(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
			   FourWheeledConf conf, const std::string& name)
    : Nimm4(odeHandle, osgHandle, name, conf.size, conf.force, conf.speed, conf.sphereWheels), conf(conf)
  {
    length=conf.size/2.0; // length of body
    
    wheelsubstance=conf.wheelSubstance;
  };


  FourWheeled::~FourWheeled(){
    destroy();
  }

  int FourWheeled::getSensorNumber(){ 
    return Nimm4::getSensorNumber() + irSensorBank.size();
  }

  int FourWheeled::getSensors(sensor* sensors, int sensornumber){
    int len = Nimm4::getSensors(sensors,sensornumber);

    // ask sensorbank for sensor values (from infrared sensors)
    //  sensor+len is the starting point in the sensors array
    if (conf.irFront || conf.irSide || conf.irBack){
      len += irSensorBank.get(sensors+len, sensornumber-len);
    }
    return len;
  };

  void FourWheeled::update(){
    Nimm4::update();
    bumpertrans->update();
    // update sensorbank with infrared sensors
    irSensorBank.update();
  }

  void FourWheeled::doInternalStuff(GlobalData& globalData){
    irSensorBank.reset(); // reset sensorbank (infrared sensors)
  }

  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void FourWheeled::create(const osg::Matrix& pose){
    Nimm4::create(pose);
    // create frame to not fall on back

    
    bumper = new Box(0.1 , width+2*wheelthickness+radius, length+0.7*width);
    bumper->setTexture("Images/wood.rgb");
    bumpertrans = new Transform(object[0], bumper,				
				Matrix::translate(width*0.6-radius, 0, 0));
    bumpertrans->init(odeHandle, 0, osgHandle);        

    

    /* initialize sensorbank (for use of infrared sensors)
     * sensor values (if sensors used) are saved in the vector of
     * sensorvalues in the following order:
     * front left
     * front right
     * right middle
     * rear right
     * rear left
     * left  middle
    */
    irSensorBank.init(odeHandle, osgHandle);
    if (conf.irFront){ // add front left and front right infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
	IRSensor* sensor = new IRSensor();
	irSensorBank.registerSensor(sensor, object[0],
				    Matrix::rotate(i*M_PI/10, Vec3(1,0,0)) *
				    Matrix::translate(0,-i*width/10,length/2 + width/2 - width/60 ),
				    conf.irRangeFront, RaySensor::drawAll);
      }
    }
    if (conf.irSide){ // add right infrared sensor to sensorbank if required
      IRSensor* sensor = new IRSensor();
      irSensorBank.registerSensor(sensor, object[0],
				  //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
				  Matrix::rotate(M_PI/2, Vec3(1,0,0)) *
				  Matrix::translate(0,-width/2, 0 ),
				  conf.irRangeSide, RaySensor::drawAll);    
    }
    if (conf.irBack){ // add rear right and rear left infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
	IRSensor* sensor = new IRSensor();
	irSensorBank.registerSensor(sensor, object[0],
				    Matrix::rotate(-i*M_PI/10, Vec3(1,0,0)) *
				    Matrix::rotate(i*M_PI, Vec3(0,1,0)) *
				    Matrix::translate(0,i*width/10,-(length/2 + width/2 - width/60) ),
				    conf.irRangeBack, RaySensor::drawAll);
      }
    }
    if (conf.irSide){ // add left infrared sensor to sensorbank if required
	IRSensor* sensor = new IRSensor();
	irSensorBank.registerSensor(sensor, object[0],
				    //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
				    Matrix::rotate(-M_PI/2, Vec3(1,0,0)) *
				    Matrix::translate(0,width/2, 0),
				    conf.irRangeSide, RaySensor::drawAll);      
    }
  };


  /** destroys vehicle and space
   */
  void FourWheeled::destroy(){
    if (created)
      irSensorBank.clear();
    Nimm4::destroy();
  }

}


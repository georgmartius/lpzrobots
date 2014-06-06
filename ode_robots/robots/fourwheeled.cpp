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

#include <ode-dbl/ode.h>
#include <assert.h>
#include <osg/Matrix>

#include "fourwheeled.h"
#include "joint.h"
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

  int FourWheeled::getSensorNumberIntern(){
    if(conf.twoWheelMode){
      assert(Nimm4::getSensorNumberIntern() == 4);
      return 2;
    }else
      return Nimm4::getSensorNumberIntern();
  }

  int FourWheeled::getSensorsIntern(double* sensors, int sensornumber){
    int len = 0;
    if(conf.twoWheelMode){
      sensor nimm4s[4];
      Nimm4::getSensorsIntern(nimm4s,4);
      sensors[len++] = (nimm4s[0]+nimm4s[2])/2;
      sensors[len++] = (nimm4s[1]+nimm4s[3])/2;
    } else {
      len = Nimm4::getSensorsIntern(sensors,sensornumber);
    }

    return len;
  };

  int FourWheeled::getMotorNumberIntern(){
    if(conf.twoWheelMode)
      return 2;
    else
      return Nimm4::getMotorNumberIntern();
  }

  void FourWheeled::setMotorsIntern(const double* motors, int motornumber){
    if(conf.twoWheelMode){
      motor nimm4m[4];
      nimm4m[0] = motors[0];
      nimm4m[2] = motors[0];
      nimm4m[1] = motors[1];
      nimm4m[3] = motors[1];
      Nimm4::setMotorsIntern(nimm4m,4);
    }else
       Nimm4::setMotorsIntern(motors,motornumber);

  }


  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void FourWheeled::create(const osg::Matrix& pose){
    Nimm4::create(pose);
    // create frame to not fall on back

    if(conf.useBumper){
      bumper = new Box(0.1 , width+2*wheelthickness+radius, length+0.7*width);
      bumper->setTexture("Images/wood.rgb");
      bumpertrans = new Transform(objects[0], bumper,
                                  Matrix::translate(width*0.6-radius, 0, 0));
      bumpertrans->init(odeHandle, 0, osgHandle);
      objects.push_back(bumpertrans);
    }else if(conf.useButton){
      bumper = new Box(width*0.6 , width*0.7, 0.1);
      bumper->setTexture("Images/wood.rgb");
      bumpertrans = new Transform(objects[0], bumper,
                                  Matrix::translate(0,0, -length*0.9));
      bumpertrans->init(odeHandle, 0, osgHandle.changeColor(1,1,0));
      objects.push_back(bumpertrans);
    }



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
    RaySensorBank* irSensorBank = new RaySensorBank();
    irSensorBank->setInitData(odeHandle, osgHandle, TRANSM(0,0,0));
    if (conf.irFront){ // add front left and front right infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
        IRSensor* sensor = new IRSensor();
        irSensorBank->registerSensor(sensor, objects[0],
                                    Matrix::rotate(i*M_PI/10, Vec3(1,0,0)) *
                                    Matrix::translate(0,-i*width/10,length/2 + width/2 - width/60 ),
                                    conf.irRangeFront, RaySensor::drawAll);
      }
    }
    if (conf.irSide){ // add right infrared sensor to sensorbank if required
      IRSensor* sensor = new IRSensor();
      irSensorBank->registerSensor(sensor, objects[0],
                                  //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                                  Matrix::rotate(M_PI/2, Vec3(1,0,0)) *
                                  Matrix::translate(0,-width/2, 0 ),
                                  conf.irRangeSide, RaySensor::drawAll);
    }
    if (conf.irBack){ // add rear right and rear left infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
        IRSensor* sensor = new IRSensor();
        irSensorBank->registerSensor(sensor, objects[0],
                                    Matrix::rotate(-i*M_PI/10, Vec3(1,0,0)) *
                                    Matrix::rotate(i*M_PI, Vec3(0,1,0)) *
                                    Matrix::translate(0,i*width/10,-(length/2 + width/2 - width/60) ),
                                    conf.irRangeBack, RaySensor::drawAll);
      }
    }
    if (conf.irSide){ // add left infrared sensor to sensorbank if required
        IRSensor* sensor = new IRSensor();
        irSensorBank->registerSensor(sensor, objects[0],
                                    //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
                                    Matrix::rotate(-M_PI/2, Vec3(1,0,0)) *
                                    Matrix::translate(0,width/2, 0),
                                    conf.irRangeSide, RaySensor::drawAll);
    }
    addSensor(shared_ptr<Sensor>(irSensorBank));
  };


  // returns the joint with index i
  Joint* FourWheeled::getJoint(int i){
    if(i>3)i=3;
    if(i<0)i=0;
    return joints[i];
  }


  /** destroys vehicle and space
   */
  void FourWheeled::destroy(){
    Nimm4::destroy();
  }

}


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
#include <selforg/matrix.h>
#include <osg/Matrix>
#include "barrel2masses.h"

#include "irsensor.h"
#include "osgprimitive.h" // get access to graphical (OSG) primitives
#include "mathutils.h"


using namespace osg;
using namespace std;

namespace lpzrobots {

  /**
   *constructor
   **/
  Barrel2Masses::Barrel2Masses ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                                 const Sphererobot3MassesConf& conf, const std::string& name,
                                 double transparency)
    : Sphererobot3Masses ( odeHandle, osgHandle, conf,
                           name, transparency)
  {
    numberaxis=2;
    this->conf.irAxis3 = false;
  }

  Barrel2Masses::~Barrel2Masses()
  {
  }

  int Barrel2Masses::getSensorsIntern( sensor* sensors, int sensornumber )
  {
    int len=0;
    matrix::Matrix A = odeRto3x3RotationMatrix ( dBodyGetRotation ( objects[Base]->getBody() ) );
    if(conf.motorsensor){
      for ( unsigned int n = 0; n < numberaxis; n++ ) {
        sensors[len] = servo[n]->get()*0.5; // we half them to decrease their influence to the control
        len++;
      }
    }

    FOREACH(list<Sensor*>, conf.sensors, i){
      len += (*i)->get(sensors+len, sensornumber-len);
    }

//     if(conf.axisZsensor){
//       // z-coordinate of local x and y axis in world coordinates
//       len += A.row(2).columns(0,1).convertToBuffer(sensors+len, sensornumber-len);
//     }
//     if(conf.axisXYZsensor){
//       // rotation matrix - 9 (vectors of all axis in world coordinates
//       len += A.convertToBuffer(sensors + len , sensornumber -len);
//     }

    // reading ir sensorvalues
    if (conf.irAxis1 || conf.irAxis2){
      len += irSensorBank.get(sensors+len, sensornumber-len);
    }
    return len;
  }

  void Barrel2Masses::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

    // create vehicle space and add it to the top level space
    odeHandle.createNewSimpleSpace(parentspace,true);
    Color c(osgHandle.color);
    c.alpha() = transparency;
    OsgHandle osgHandle_Base = osgHandle.changeColor(c);
    OsgHandle osgHandleX[servono];
    osgHandleX[0] = osgHandle.changeColor(Color(1.0, 0.0, 0.0));
    osgHandleX[1] = osgHandle.changeColor(Color(0.0, 1.0, 0.0));

    objects[Base] = new Cylinder(conf.diameter/2, conf.diameter);
    objects[Base]->init(odeHandle, conf.spheremass, osgHandle_Base);
    objects[Base]->setPose(pose);

    Pos p(pose.getTrans());
    Primitive* pendular[servono];

    //definition of the 2 Slider-Joints, which are the controled by the robot-controler
    for ( unsigned int n = 0; n < numberaxis; n++ ) {
      pendular[n] = new Sphere(conf.pendulardiameter/2);
      pendular[n]->init(odeHandle, conf.pendularmass, osgHandleX[n],
                        Primitive::Body | Primitive::Draw); // without geom

      pendular[n]->setPose(Matrix::translate(0,0,(n==0?-1:1)*conf.axesShift)*pose);

      OneAxisJoint* j = new SliderJoint(objects[Base], pendular[n],
                                 p, Axis((n==0), (n==1), (n==2))*pose );
      j->init(odeHandle, osgHandle, false);
      // the Stop parameters are messured from the initial position!
//       j->setParam ( dParamLoStop, -1.2*conf.diameter*conf.pendularrange );
//       j->setParam ( dParamHiStop, 1.2*conf.diameter*conf.pendularrange );
      j->setParam ( dParamStopCFM, 0.1);
      j->setParam ( dParamStopERP, 0.9);
      j->setParam ( dParamCFM, 0.001);
      servo[n] = new SliderServo(j,
                                 -conf.diameter*conf.pendularrange,
                                 conf.diameter*conf.pendularrange,
                                 //                                 conf.pendularmass*100,10,1);
                                       conf.pendularmass*conf.motorpowerfactor,0.1,0.5,
                                 50 /*maximal velocity*/);
      joints[n]=j;

      axis[n] = new OSGCylinder(conf.diameter/100, conf.diameter - conf.diameter/100);
      axis[n]->init(osgHandleX[n], OSGPrimitive::Low);
      axis[n]->setMatrix(Matrix::translate(0,0,(n==0?-1:1)*conf.axesShift)*pose);
      objects[Pendular1+n] = pendular[n];
    }

    double sensorrange = conf.irsensorscale * conf.diameter;
    RaySensor::rayDrawMode drawMode = conf.drawIRs ? RaySensor::drawAll : RaySensor::drawSensor;

    irSensorBank.setInitData(odeHandle, osgHandle, TRANSM(0,0,0) );
    irSensorBank.init(0);
    if (conf.irAxis1){
      for(int i=-1; i<2; i+=2){
        IRSensor* sensor = new IRSensor(1.5);
        Matrix R = Matrix::rotate(i*M_PI/2, 1, 0, 0) * Matrix::translate(0,-i*conf.diameter/2,0 );
        irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irAxis2){
      for(int i=-1; i<2; i+=2){
        IRSensor* sensor = new IRSensor(1.5);
        Matrix R = Matrix::rotate(i*M_PI/2, 0, 1, 0) * Matrix::translate(i*conf.diameter/2,0,0 );
        irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
      }
    }

    FOREACH(list<Sensor*>, conf.sensors, i){
      (*i)->init(objects[Base]);
    }

  }

}


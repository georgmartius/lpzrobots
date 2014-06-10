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
#include "discus.h"

#include <ode_robots/irsensor.h>
#include <ode_robots/osgprimitive.h> // get access to graphical (OSG) primitives
#include <ode_robots/mathutils.h>


using namespace osg;
using namespace std;

namespace lpzrobots {


  void DiscusConf::destroy(){
    for(list<Sensor*>::iterator i = sensors.begin(); i != sensors.end(); i++){
      if(*i) delete *i;
    }
    sensors.clear();
  }

  const int Discus::maxservono;

  /**
   *constructor
   **/
  Discus::Discus ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                                           const DiscusConf& conf,
                                           const std::string& name,
                                           double transparency)
    : OdeRobot ( odeHandle, osgHandle, name,
                 "$Id$"),
      conf(conf), transparency(transparency)
  {
    init();
  }

  /**
   *constructor
   **/
  Discus::Discus ( const OdeHandle& odeHandle,
                   const OsgHandle& osgHandle,
                   const DiscusConf& conf,
                   const std::string& name,
                   const std::string& revision,
                   double transparency)
    : OdeRobot ( odeHandle, osgHandle, name, revision),
      conf(conf),transparency(transparency)
  {
    init();
  }


  void Discus::init(){
    created = false;
    memset(object, 0,sizeof(void*) * Last);
    memset(joint, 0,sizeof(void*) * maxservono);
    memset(axis, 0,sizeof(void*) * maxservono);
    memset(servo, 0,sizeof(void*) * maxservono);

    this->conf.pendulardiameter = conf.diameter/10;
  }

  Discus::~Discus()
  {
    destroy();
    if(conf.irSensorTempl) delete conf.irSensorTempl;
  }

  void Discus::update()
  {
    for (int i=0; i < Last; i++) {
      if(object[i]) object[i]->update();
    }
    Matrix pose(object[Base]->getPose());
    for (unsigned int i=0; i < conf.numAxes; i++) {
      if(axis[i]){
        axis[i]->setMatrix(Matrix::rotate(M_PI/2, (i==1), (i==0), (i==2)) * pose);
      }
    }
    irSensorBank.update();
  }

  /**
   *Writes the sensor values to an array in the memory.
   *@param sensor* pointer to the array
   *@param sensornumber length of the sensor array
   *@return number of actually written sensors
   **/
  int Discus::getSensorsIntern( sensor* sensors, int sensornumber )
  {
    int len=0;
    assert(created);
    if(!conf.motor_ir_before_sensors){
      FOREACH(list<Sensor*>, conf.sensors, i){
        len += (*i)->get(sensors+len, sensornumber-len);
      }
    }

    if(conf.motorsensor){
      for ( unsigned int n = 0; n < conf.numAxes; n++ ) {
        sensors[len] = servo[n]->get() * 0.5;  // we half them to decrease their influence to the control
        len++;
      }
    }

    // reading ir sensorvalues
    if (conf.irAxis1 || conf.irAxis2 || conf.irAxis3 || conf.irRing || conf.irSide){
      len += irSensorBank.get(sensors+len, sensornumber-len);
    }

    if(conf.motor_ir_before_sensors){
      FOREACH(list<Sensor*>, conf.sensors, i){
        len += (*i)->get(sensors+len, sensornumber-len);
      }
    }


    return len;
  }

  void Discus::setMotorsIntern( const double* motors, int motornumber ) {
    int len = min((unsigned)motornumber, conf.numAxes);
    for ( int n = 0; n < len; n++ ) {
      servo[n]->set(motors[n]);
    }
  }


  void Discus::placeIntern(const osg::Matrix& pose){
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.diameter/2));
    create(p2);
  };

  void Discus::sense(GlobalData& globalData) {
    OdeRobot::sense(globalData);
    // reset ir sensors to maximum value
    irSensorBank.sense(globalData);
  }

  void Discus::doInternalStuff(GlobalData& global){
    // slow down rotation around z axis because friction does not catch it.
    dBodyID b = getMainPrimitive()->getBody();
    //    double friction = odeHandle.substance.roughness;
    const double* vel = dBodyGetAngularVel( b);
    // deaccelerates the robot
    if(conf.brake){
      dBodyAddTorque ( b , -conf.brake*vel[0] , -conf.brake*vel[1] , -conf.brake*vel[2] );
    }
  }

  int Discus::getMotorNumberIntern(){
    return conf.numAxes;
  }

  int Discus::getSensorNumberIntern() {
    int s=0;
    FOREACHC(list<Sensor*>, conf.sensors, i){
      s += (*i)->getSensorNumber();
    }
    return conf.motorsensor * conf.numAxes + s + irSensorBank.getSensorNumber();
  }


  /** creates vehicle at desired position and orientation
  */
  void Discus::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

    // create vehicle space and add it to the top level space
    odeHandle.createNewSimpleSpace(parentspace,true);
    Color c(osgHandle.color);
    c.alpha() = transparency;
    OsgHandle osgHandle_Base = osgHandle.changeColor(c);
    OsgHandle osgHandleX[3];
    osgHandleX[0] = osgHandle.changeColor(Color(1.0, 0.0, 0.0));
    osgHandleX[1] = osgHandle.changeColor(Color(0.0, 1.0, 0.0));
    osgHandleX[2] = osgHandle.changeColor(Color(0.0, 0.0, 1.0));

    //    object[Base] = new InvisibleSphere(conf.diameter/2);
    object[Base] = new Cylinder(conf.diameter/2, conf.relativewidth * conf.diameter);
    object[Base]->init(odeHandle, conf.spheremass, osgHandle_Base);
    object[Base]->setPose(pose);
    // attach stabilizer
    Primitive* stab = new Capsule(conf.diameter/2 * conf.stabdiameter, conf.relativewidth * conf.diameter);
    object[Stabilizer] = new Transform(object[Base], stab, osg::Matrix::translate(0,0,0));
    object[Stabilizer]->init(odeHandle, 0, osgHandle_Base);


    Pos p(pose.getTrans());
    Primitive* pendular[conf.numAxes];
    memset(pendular, 0, sizeof(void*) * conf.numAxes);

    double stabilizerlength = (conf.stabdiameter+conf.relativewidth) * conf.diameter;
    //definition of the n Slider-Joints, which are the controled by the robot-controler
    for ( unsigned int n = 0; n < conf.numAxes; n++ ) {
      pendular[n] = new Sphere(conf.pendulardiameter/2);
      pendular[n]->init(odeHandle, conf.pendularmass, osgHandleX[n],
                        Primitive::Body | Primitive::Draw); // without geom
      pendular[n]->setPose(pose);

      joint[n] = new SliderJoint(object[Base], pendular[n],
                                 p, Axis((n==0), (n==1), (n==2))*pose);
      joint[n]->init(odeHandle, osgHandle, false);
      joint[n]->setParam ( dParamStopCFM, 0.1);
      joint[n]->setParam ( dParamStopERP, 0.9);
      joint[n]->setParam ( dParamCFM, 0.001);
      double range =  (n==2 ? stabilizerlength * conf.pendularrangeN : conf.diameter*conf.pendularrange);
      servo[n] = new SliderServo(joint[n],
                                 -range, range,
                                 conf.pendularmass*conf.motorpowerfactor,0.1,0.5);
      axis[n] = new OSGCylinder(conf.diameter/100, (n==2 ? stabilizerlength : conf.diameter) -
                                conf.diameter/100);
      axis[n]->init(osgHandleX[n], OSGPrimitive::Low);
      object[Pendular1+n] = pendular[n];
    }

    double sensorrange = conf.irsensorscale * conf.diameter;
    RaySensor::rayDrawMode drawMode = conf.drawIRs ? RaySensor::drawAll : RaySensor::drawSensor;
    double sensors_inside=0.02;
    if(conf.irSensorTempl==0){
      conf.irSensorTempl=new IRSensor(conf.irCharacter);
    }
    irSensorBank.setInitData(odeHandle, osgHandle, TRANSM(0,0,0) );
    irSensorBank.init(0);
    if (conf.irAxis1){
      for(int i=-1; i<2; i+=2){
        RaySensor* sensor = conf.irSensorTempl->clone();
        Matrix R = Matrix::rotate(i*M_PI/2, 1, 0, 0) *
          Matrix::translate(0,-i*(conf.diameter/2-sensors_inside),0 );
        irSensorBank.registerSensor(sensor, object[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irAxis2){
      for(int i=-1; i<2; i+=2){
        RaySensor* sensor = conf.irSensorTempl->clone();
        Matrix R = Matrix::rotate(i*M_PI/2, 0, 1, 0) *
          Matrix::translate(i*(conf.diameter/2-sensors_inside),0,0 );
        //        dRFromEulerAngles(R,i*M_PI/2,-i*M_PI/2,0);
        irSensorBank.registerSensor(sensor, object[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irAxis3){
      for(int i=-1; i<2; i+=2){
        RaySensor* sensor = conf.irSensorTempl->clone();
        double stabilizerlength = (conf.stabdiameter+conf.relativewidth) * conf.diameter;
        Matrix R = Matrix::rotate( i==1 ? 0 : M_PI, 1, 0, 0) *
          Matrix::translate(0,0,i*(stabilizerlength/2-sensors_inside));
        irSensorBank.registerSensor(sensor, object[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irRing){
      for(double i=0; i<2*M_PI; i+=M_PI/6){  // 12 sensors
        RaySensor* sensor = conf.irSensorTempl->clone();
        Matrix R = Matrix::rotate( M_PI/2, 1, 0, 0) *
          Matrix::translate(0,-conf.diameter/2+sensors_inside,0) *
          Matrix::rotate( i, 0, 0, 1);
        irSensorBank.registerSensor(sensor, object[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irSide){
      for(double i=0; i<2*M_PI; i+=M_PI/2){
        RaySensor* sensor = conf.irSensorTempl->clone();
        Matrix R = Matrix::translate(0,0,conf.stabdiameter*conf.diameter/2) *
          Matrix::rotate( M_PI/8, 0, 1, 0) *
          Matrix::translate(0,0,conf.diameter/2*conf.relativewidth-sensors_inside) *
          Matrix::rotate( i, 0, 0, 1);
        irSensorBank.registerSensor(sensor, object[Base], R, sensorrange, drawMode);
        sensor = new IRSensor(conf.irCharacter);// and the other side
        irSensorBank.registerSensor(sensor, object[Base],
                                    R * Matrix::rotate( M_PI, 0, 1, 0),
                                    sensorrange, drawMode);
      }
    }

    FOREACH(list<Sensor*>, conf.sensors, i){
      (*i)->init(object[Base]);
    }

  created=true;
  }


  /** destroys vehicle and space
   */
  void Discus::destroy(){
    if (created){
      for (unsigned int i=0; i<conf.numAxes; i++){
        if(joint[i]) delete joint[i];
        if(servo[i]) delete servo[i];
        if(axis[i]) delete axis[i];
      }
      for (int i=0; i<Last; i++){
        if(object[i]) delete object[i];
      }
      irSensorBank.clear();
      odeHandle.deleteSpace();
    }
    created=false;
  }

}

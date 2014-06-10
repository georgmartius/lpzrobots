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
#include "sphererobot3masses.h"

#include "irsensor.h"
#include "osgprimitive.h" // get access to graphical (OSG) primitives
#include "mathutils.h"


using namespace osg;
using namespace std;

namespace lpzrobots {


  void Sphererobot3MassesConf::destroy(){
    for(list<Sensor*>::iterator i = sensors.begin(); i != sensors.end(); i++){
      if(*i) delete *i;
    }
    sensors.clear();
  }

  const int Sphererobot3Masses::servono;

  /**
   *constructor
   **/
  Sphererobot3Masses::Sphererobot3Masses ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                                           const Sphererobot3MassesConf& conf,
                                           const std::string& name,
                                           double transparency)
    : OdeRobot ( odeHandle, osgHandle, name,
                 "$Id$"),
      conf(conf), transparency(transparency)
  {
    numberaxis=3;
    init();

    addParameter("pendularrange",&this->conf.pendularrange,0,0.4,
                 "range of the masses along the sliders");
  }

  /**
   *constructor
   **/
  Sphererobot3Masses::Sphererobot3Masses ( const OdeHandle& odeHandle,
                                           const OsgHandle& osgHandle,
                                           const Sphererobot3MassesConf& conf,
                                           const std::string& name,
                                           const std::string& revision,
                                           double transparency)
    : OdeRobot ( odeHandle, osgHandle, name, revision),
      conf(conf),transparency(transparency)
  {
    numberaxis=3;
    init();
  }


  void Sphererobot3Masses::init(){
    created = false;
    objects.resize(Last);
    joints.resize(servono);
    memset(axis, 0,sizeof(void*) * servono);
    memset(servo, 0,sizeof(void*) * servono);

    this->conf.pendulardiameter = conf.diameter/7;
  }

  Sphererobot3Masses::~Sphererobot3Masses()
  {
    destroy();
    if(conf.irSensorTempl) delete conf.irSensorTempl;
    FOREACH(std::list<Sensor*>, conf.sensors, s){
      if(*s) delete *s;
    }
  }

  void Sphererobot3Masses::update()
  {
    for (int i=0; i < Last; i++) {
      if(objects[i]) objects[i]->update();
    }
    Matrix pose(objects[Base]->getPose());
    for (int i=0; i < servono; i++) {
      if(axis[i]){
        axis[i]->setMatrix(Matrix::rotate(M_PI/2, (i==1), (i==0), (i==2)) * Matrix::translate(0 ,0, (i==0?-1:1)*conf.axesShift)* pose);
      }
    }
    irSensorBank.update();
    FOREACH(std::list<Sensor*>, conf.sensors, s){
      (*s)->update();
    }
  }

  /**
   *Writes the sensor values to an array in the memory.
   *@param sensor* pointer to the array
   *@param sensornumber length of the sensor array
   *@return number of actually written sensors
   **/
  int Sphererobot3Masses::getSensorsIntern( sensor* sensors, int sensornumber )
  {
    int len=0;
    assert(created);
    if(!conf.motor_ir_before_sensors){
      FOREACH(list<Sensor*>, conf.sensors, i){
        len += (*i)->get(sensors+len, sensornumber-len);
      }
    }

    if(conf.motorsensor){
      for ( unsigned int n = 0; n < numberaxis; n++ ) {
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

  void Sphererobot3Masses::setMotorsIntern( const double* motors, int motornumber ) {
    int len = min((unsigned)motornumber, numberaxis);
    for ( int n = 0; n < len; n++ ) {
      servo[n]->set(motors[n]);
    }
  }


  void Sphererobot3Masses::placeIntern(const osg::Matrix& pose){
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.diameter/2));
    create(p2);
  };

  void Sphererobot3Masses::sense(GlobalData& globalData) {
    // reset ir sensors to maximum value
    irSensorBank.sense(globalData);
    OdeRobot::sense(globalData);
  }


  void Sphererobot3Masses::doInternalStuff(GlobalData& global){
    OdeRobot::doInternalStuff(global);
    // slow down rotation around z axis because friction does not catch it.
    dBodyID b = getMainPrimitive()->getBody();
    double friction = odeHandle.substance.roughness;
    const double* vel = dBodyGetAngularVel( b);
    if(fabs(vel[2])>0.2){
      dBodyAddTorque ( b , 0 , 0 , -0.05*friction*vel[2] );
    }
    // deaccelerates the robot
    if(conf.brake){
      dBodyAddTorque ( b , -conf.brake*vel[0] , -conf.brake*vel[1] , -conf.brake*vel[2] );
    }

    FOREACH(list<Sensor*>, conf.sensors, s){
      (*s)->sense(global);
    }

  }

  int Sphererobot3Masses::getMotorNumberIntern(){
    return numberaxis;
  }

  int Sphererobot3Masses::getSensorNumberIntern() {
    int s=0;
    FOREACHC(list<Sensor*>, conf.sensors, i){
      s += (*i)->getSensorNumber();
    }
    return conf.motorsensor * numberaxis + s + irSensorBank.getSensorNumber();
  }


  /** creates vehicle at desired position and orientation
  */
  void Sphererobot3Masses::create(const osg::Matrix& pose){
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

    //    objects[Base] = new InvisibleSphere(conf.diameter/2);
    objects[Base] = new Sphere(conf.diameter/2);
        //objects[Base] = new Box(conf.diameter, conf.diameter, conf.diameter);
    objects[Base]->init(odeHandle, conf.spheremass, osgHandle_Base);
    objects[Base]->setPose(pose);

    Pos p(pose.getTrans());
    Primitive* pendular[servono];
    memset(pendular, 0, sizeof(void*) * servono);

    //definition of the 3 Slider-Joints, which are the controled by the robot-controler
    for ( unsigned int n = 0; n < numberaxis; n++ ) {
      pendular[n] = new Sphere(conf.pendulardiameter/2);
      pendular[n]->init(odeHandle, conf.pendularmass, osgHandleX[n],
                        Primitive::Body | Primitive::Draw); // without geom
      pendular[n]->setPose(Matrix::translate(0,0,(n==0?-1:1)*conf.axesShift)*pose);

      joints[n] = new SliderJoint(objects[Base], pendular[n],
                                 p, Axis((n==0), (n==1), (n==2))*pose);
      joints[n]->init(odeHandle, osgHandle, false);
      // the Stop parameters are messured from the initial position!
      // the stops are set by the servo
      //joints[n]->setParam ( dParamLoStop, -1.1*conf.diameter*conf.pendularrange );
      //      joints[n]->setParam ( dParamHiStop, 1.1*conf.diameter*conf.pendularrange );

      joints[n]->setParam ( dParamStopCFM, 0.1);
      joints[n]->setParam ( dParamStopERP, 0.9);
      joints[n]->setParam ( dParamCFM, 0.001);
      // see also setParam() for the stops
      servo[n] = new SliderServo(dynamic_cast<OneAxisJoint*>(joints[n]),
                                 -conf.diameter*conf.pendularrange,
                                 conf.diameter*conf.pendularrange,
      //                         conf.pendularmass*conf.motorpowerfactor,10,1);
                                 conf.pendularmass*conf.motorpowerfactor,0.1,0.5);

      axis[n] = new OSGCylinder(conf.diameter/100, conf.diameter - conf.diameter/100);
      axis[n]->init(osgHandleX[n], OSGPrimitive::Low);
    }
    objects[Pendular1] = pendular[0];
    objects[Pendular2] = pendular[1];
    objects[Pendular3] = pendular[2];

    double sensorrange = conf.irsensorscale * conf.diameter;
    RaySensor::rayDrawMode drawMode = conf.drawIRs;
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
        irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irAxis2){
      for(int i=-1; i<2; i+=2){
        RaySensor* sensor = conf.irSensorTempl->clone();
        Matrix R = Matrix::rotate(i*M_PI/2, 0, 1, 0) *
          Matrix::translate(i*(conf.diameter/2-sensors_inside),0,0 );
        //        dRFromEulerAngles(R,i*M_PI/2,-i*M_PI/2,0);
        irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irAxis3){
      for(int i=-1; i<2; i+=2){
        RaySensor* sensor = conf.irSensorTempl->clone();
        Matrix R = Matrix::rotate( i==1 ? 0 : M_PI, 1, 0, 0) *
          Matrix::translate(0,0,i*(conf.diameter/2-sensors_inside));
        irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irRing){
      for(double i=0; i<2*M_PI; i+=M_PI/6){  // 12 sensors
        RaySensor* sensor = conf.irSensorTempl->clone();
        Matrix R = Matrix::translate(0,0,conf.diameter/2-sensors_inside) *
          Matrix::rotate( i, 0, 1, 0);
        irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irSide){
      for(double i=0; i<2*M_PI; i+=M_PI/2){
        RaySensor* sensor = conf.irSensorTempl->clone();
        Matrix R = Matrix::translate(0,0,conf.diameter/2-sensors_inside) *
          Matrix::rotate( M_PI/2-M_PI/8, 1, 0, 0) *  Matrix::rotate( i, 0, 1, 0);
        irSensorBank.registerSensor(sensor, objects[Base], R, sensorrange, drawMode);
        sensor = new IRSensor(conf.irCharacter);// and the other side
        irSensorBank.registerSensor(sensor, objects[Base],
                                    R * Matrix::rotate( M_PI, 0, 0, 1),
                                    sensorrange, drawMode);
      }
    }

    FOREACH(list<Sensor*>, conf.sensors, i){
      (*i)->init(objects[Base]);
    }

  created=true;
  }


  /** destroys vehicle and space
   */
  void Sphererobot3Masses::destroy(){
    if (created){
      for (int i=0; i<servono; i++){
        if(servo[i]) delete servo[i];
        if(axis[i]) delete axis[i];
      }
      irSensorBank.clear();
      odeHandle.deleteSpace();
    }
    created=false;
  }

  void Sphererobot3Masses::notifyOnChange(const paramkey& key){
    for (int i=0; i<servono; i++){
      if(servo[i]) servo[i]->setMinMax(-conf.diameter*conf.pendularrange,
                                       conf.diameter*conf.pendularrange);
    }
  }

}

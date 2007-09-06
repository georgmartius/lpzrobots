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
 *   Revision 1.17  2007-09-06 18:48:00  martius
 *   createNewSimpleSpace used
 *
 *   Revision 1.16  2007/08/24 11:57:30  martius
 *   additional sensors can be before or after motor and ir sensors
 *
 *   Revision 1.15  2007/07/17 07:22:28  martius
 *   removed invisible primitives
 *
 *   Revision 1.14  2007/07/03 13:05:23  martius
 *   new servo constants
 *
 *   Revision 1.13  2007/06/21 16:24:27  martius
 *   joints are deleted before objects
 *
 *   Revision 1.12  2007/04/05 15:11:43  martius
 *   angular speed tracking
 *
 *   Revision 1.11  2007/04/03 16:27:31  der
 *   new IR shape
 *   new servo parameters
 *
 *   Revision 1.10  2007/03/26 13:17:43  martius
 *   changes servo params
 *
 *   Revision 1.9  2007/02/23 15:14:17  martius
 *   *** empty log message ***
 *
 *   Revision 1.8  2007/01/03 15:01:09  fhesse
 *   created=true; added (at end of create())
 *
 *   Revision 1.7  2006/12/21 11:43:05  martius
 *   commenting style for doxygen //< -> ///<
 *   new sensors for spherical robots
 *
 *   Revision 1.6  2006/12/01 16:20:40  martius
 *   *** empty log message ***
 *
 *   Revision 1.5  2006/11/17 13:44:50  martius
 *   corrected z-axes sensor problem
 *   there are two sensors for this situation
 *
 *   Revision 1.4  2006/08/08 17:04:46  martius
 *   added new sensor model
 *
 *   Revision 1.3  2006/07/20 17:19:45  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.2  2006/07/14 12:23:42  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.9  2006/07/10 12:05:02  martius
 *   Matrixlib now in selforg
 *   no namespace std in header files
 *
 *   Revision 1.1.2.8  2006/06/29 16:39:56  robot3
 *   -you can now see bounding shapes if you type ./start -drawboundings
 *   -includes cleared up
 *   -abstractobstacle and abstractground have now .cpp-files
 *
 *   Revision 1.1.2.7  2006/06/25 17:00:33  martius
 *   Id
 *
 *   Revision 1.1.2.6  2006/06/25 16:57:16  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.5  2006/05/09 04:24:34  robot5
 *   *** empty log message ***
 *
 *   Revision 1.1.2.4  2006/03/29 15:09:27  martius
 *   Z-Sensors have been wrong all the time :-)
 *
 *   Revision 1.1.2.3  2006/02/20 10:50:20  martius
 *   pause, random, windowsize, Ctrl-keys
 *
 *   Revision 1.1.2.2  2006/01/11 19:30:28  martius
 *   transparent hull
 *
 *   Revision 1.1.2.1  2006/01/10 17:15:04  martius
 *   was sphererobotarms
 *   moved to osg
 *
 *   Revision 1.18.4.2  2005/11/15 12:29:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.18.4.1  2005/11/14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.18  2005/11/09 13:26:57  martius
 *   irsensorrange
 *
 *   Revision 1.17  2005/11/09 13:24:42  martius
 *   added GPL
 *
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
					   const Sphererobot3MassesConf& conf, const std::string& name,
					   double transparency)
    : OdeRobot ( odeHandle, osgHandle, name, "$Id$"), 
      conf(conf), transparency(transparency) 
  {
    numberaxis=3;
    init();
  }

  /**
   *constructor
   **/
  Sphererobot3Masses::Sphererobot3Masses ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
					   const Sphererobot3MassesConf& conf, const std::string& name,
					   const std::string& revision,
					   double transparency)
    : OdeRobot ( odeHandle, osgHandle, name, revision), conf(conf),transparency(transparency)
  {
    numberaxis=3;
    init();
  }


  void Sphererobot3Masses::init(){
    created = false;
    memset(object, 0,sizeof(void*) * Last);
    memset(joint, 0,sizeof(void*) * servono);
    memset(axis, 0,sizeof(void*) * servono); 
    memset(servo, 0,sizeof(void*) * servono);
    
    this->conf.pendulardiameter = conf.diameter/7;
  }
  
	
  Sphererobot3Masses::~Sphererobot3Masses()
  {
    destroy(); 
    if(conf.irSensorTempl) delete conf.irSensorTempl;
  }

  void Sphererobot3Masses::update()
  {
    for (int i=0; i < Last; i++) { 
      if(object[i]) object[i]->update();
    }
    Matrix pose(object[Base]->getPose());
    for (int i=0; i < servono; i++) { 
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
  int Sphererobot3Masses::getSensors ( sensor* sensors, int sensornumber )
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

  void Sphererobot3Masses::setMotors ( const motor* motors, int motornumber ) {
    int len = min((unsigned)motornumber, numberaxis);
    for ( int n = 0; n < len; n++ ) {
      servo[n]->set(motors[n]);
    }
  }


  void Sphererobot3Masses::place(const osg::Matrix& pose){
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(0, 0, conf.diameter/2)); 
    create(p2);
  };


  void Sphererobot3Masses::doInternalStuff(const GlobalData& global){
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

    irSensorBank.reset();
  }

  /**
   *This is the collision handling function for sphere robots.
   *This overwrides the function collisionCallback of the class robot.
   *@param data
   *@param o1 first geometrical object, which has taken part in the collision
   *@param o2 second geometrical object, which has taken part in the collision
   *@return true if the collision was threated  by the robot, false if not
   **/
  bool Sphererobot3Masses::collisionCallback(void *data, dGeomID o1, dGeomID o2) {
//     //checks if either of both of the collision objects are part of the robot
//      if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space) {
//        if(o1 == (dGeomID)odeHandle.space) irSensorBank.sense(o2);
//        if(o2 == (dGeomID)odeHandle.space) irSensorBank.sense(o1);
//      }

//       // inner space collisions are not treated!
//       int i,n;  
//       const int N = 40;
//       dContact contact[N];
    
//       n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
//       for (i=0; i<n; i++) {
// 	if( contact[i].geom.g1 == object[Base]->getGeom() || contact[i].geom.g2 == object[Base]->getGeom() ){ 
// 	  // only treat collisions with envelop
// 	  contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactApprox1;
// 	  //	  dContactSoftERP | dContactSoftCFM | 
// 	  contact[i].surface.mu = 2.0;
// 	  contact[i].surface.slip1 = 0.005;
// 	  contact[i].surface.slip2 = 0.005;
// 	  //	contact[i].surface.soft_erp = 1; // 0.95;
// 	  //	contact[i].surface.soft_cfm = 0.00001;
// 	  dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
// 	  dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
// 	}
//       }
//       return true;
//     } else {
      return false;
//     }
  }


  /**
   *Returns the number of motors used by the snake.
   *@return number of motors
   **/
  int Sphererobot3Masses::getMotorNumber(){
    return numberaxis;
  }

  /**
   *Returns the number of sensors used by the robot.
   *@return number of sensors
   **/
  int Sphererobot3Masses::getSensorNumber() {
    int s=0;
    FOREACHC(list<Sensor*>, conf.sensors, i){
      s += (*i)->getSensorNumber();
    }
    return conf.motorsensor * numberaxis + s + irSensorBank.getSensorNumber();
  }


  /** creates vehicle at desired position 
      @param pos struct Position with desired position
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

    //    object[Base] = new InvisibleSphere(conf.diameter/2);
    object[Base] = new Sphere(conf.diameter/2);
	//object[Base] = new Box(conf.diameter, conf.diameter, conf.diameter);
    object[Base]->init(odeHandle, conf.spheremass, osgHandle_Base);
    object[Base]->setPose(pose);    

    Pos p(pose.getTrans());
    Primitive* pendular[servono];
    memset(pendular, 0, sizeof(void*) * servono);

    //definition of the 3 Slider-Joints, which are the controled by the robot-controler
    for ( unsigned int n = 0; n < numberaxis; n++ ) {
      pendular[n] = new Sphere(conf.pendulardiameter/2);
      pendular[n]->init(odeHandle, conf.pendularmass, osgHandleX[n], 
			Primitive::Body | Primitive::Draw); // without geom
      pendular[n]->setPose(pose);    

      joint[n] = new SliderJoint(object[Base], pendular[n],
				 p, Axis((n==0), (n==1), (n==2))*pose);
      joint[n]->init(odeHandle, osgHandle, false);
      // the Stop parameters are messured from the initial position!
      joint[n]->setParam ( dParamLoStop, -1.1*conf.diameter*conf.pendularrange );
      joint[n]->setParam ( dParamHiStop, 1.1*conf.diameter*conf.pendularrange );
      joint[n]->setParam ( dParamStopCFM, 0.1);
      joint[n]->setParam ( dParamStopERP, 0.9);
      joint[n]->setParam ( dParamCFM, 0.001);
      servo[n] = new SliderServo(joint[n], 
				 -conf.diameter*conf.pendularrange, 
				 conf.diameter*conf.pendularrange, 
				 //				 conf.pendularmass*conf.motorpowerfactor,10,1); 
				 conf.pendularmass*conf.motorpowerfactor,0.1,0.5); 
      
      axis[n] = new OSGCylinder(conf.diameter/100, conf.diameter - conf.diameter/100);
      axis[n]->init(osgHandleX[n], OSGPrimitive::Low);
    }
    object[Pendular1] = pendular[0]; 
    object[Pendular2] = pendular[1]; 
    object[Pendular3] = pendular[2]; 

    double sensorrange = conf.irsensorscale * conf.diameter;
    RaySensor::rayDrawMode drawMode = conf.drawIRs ? RaySensor::drawAll : RaySensor::drawSensor;
    double sensors_inside=0.02;
    if(conf.irSensorTempl==0){
      conf.irSensorTempl=new IRSensor(conf.irCharacter);
    }
    irSensorBank.init(odeHandle, osgHandle );
    if (conf.irAxis1){
      for(int i=-1; i<2; i+=2){
	RaySensor* sensor = conf.irSensorTempl->clone();
	Matrix R = Matrix::rotate(i*M_PI/2, 1, 0, 0) * Matrix::translate(0,-i*(conf.diameter/2-sensors_inside),0 );
	irSensorBank.registerSensor(sensor, object[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irAxis2){
      for(int i=-1; i<2; i+=2){
	RaySensor* sensor = conf.irSensorTempl->clone();
	Matrix R = Matrix::rotate(i*M_PI/2, 0, 1, 0) * Matrix::translate(i*(conf.diameter/2-sensors_inside),0,0 );
	//	dRFromEulerAngles(R,i*M_PI/2,-i*M_PI/2,0);      
	irSensorBank.registerSensor(sensor, object[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irAxis3){
      for(int i=-1; i<2; i+=2){
	RaySensor* sensor = conf.irSensorTempl->clone();
	Matrix R = Matrix::rotate( i==1 ? 0 : M_PI, 1, 0, 0) * Matrix::translate(0,0,i*(conf.diameter/2-sensors_inside));
	irSensorBank.registerSensor(sensor, object[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irRing){
      for(double i=0; i<2*M_PI; i+=M_PI/6){  // 12 sensors
	RaySensor* sensor = conf.irSensorTempl->clone();
	Matrix R = Matrix::translate(0,0,conf.diameter/2-sensors_inside) * Matrix::rotate( i, 0, 1, 0);
	irSensorBank.registerSensor(sensor, object[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irSide){
      for(double i=0; i<2*M_PI; i+=M_PI/2){
	RaySensor* sensor = conf.irSensorTempl->clone();
	Matrix R = Matrix::translate(0,0,conf.diameter/2-sensors_inside) * Matrix::rotate( M_PI/2-M_PI/8, 1, 0, 0) *  Matrix::rotate( i, 0, 1, 0);
	irSensorBank.registerSensor(sensor, object[Base], R, sensorrange, drawMode); 
	sensor = new IRSensor(conf.irCharacter);// and the other side	
	irSensorBank.registerSensor(sensor, object[Base], R * Matrix::rotate( M_PI, 0, 0, 1), sensorrange, drawMode); 
      }
    }

    FOREACH(list<Sensor*>, conf.sensors, i){
      (*i)->init(object[Base]);
    }
 
  created=true;
  }


  /** destroys vehicle and space
   */
  void Sphererobot3Masses::destroy(){
    if (created){
      for (int i=0; i<servono; i++){
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

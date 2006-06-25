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
 *   Revision 1.1.2.6  2006-06-25 16:57:16  martius
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
#include <matrix.h>
#include <osg/Matrix>
#include "sphererobot3masses.h"

#include "irsensor.h"
#include "invisibleprimitive.h"

using namespace osg;

namespace lpzrobots {

  const int Sphererobot3Masses::servono;

  /**
   *constructor
   **/
  Sphererobot3Masses::Sphererobot3Masses ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
					   const Sphererobot3MassesConf& conf, const std::string& name,
					   double transparency)
    : OdeRobot ( odeHandle, osgHandle, name, "$ID$"), conf(conf)
  {

    created = false;
    memset(object, 0,sizeof(void*) * Last);
    memset(joint, 0,sizeof(void*) * servono);
    memset(axis, 0,sizeof(void*) * servono);
    memset(servo, 0,sizeof(void*) * servono);
    
    this->conf.pendulardiameter = conf.diameter/7;
    this->transparency=transparency;	

  }
	
  Sphererobot3Masses::~Sphererobot3Masses()
  {
    destroy(); 
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
    matrix::Matrix A = odeRto3x3RotationMatrix ( dBodyGetRotation ( object[Base]->getBody() ) );

    if(conf.motorsensor){
      for ( int n = 0; n < servono; n++ ) {
	sensors[len] = servo[n]->get() * 0.2;
	len++;
      }
    }
    if(conf.axisZsensor){
      // z-coordinate of axis position in world coordinates
      //      len += A.row(2).convertToBuffer(sensors+len, sensornumber-len);  

      // world coordinates of z-axis
      len += A.column(2).convertToBuffer(sensors+len, sensornumber-len);  
    }
    if(conf.axisXYZsensor){
      // rotation matrix - 9 (vectors of all axis in world coordinates
      len += A.convertToBuffer(sensors + len , sensornumber -len);
    }
    
    //   // angular velocities (local coord.) - 3
    //   Matrix angVelOut(3, 1, dBodyGetAngularVel( object[ Base ].body ));
    //   Matrix angVelIn = A*angVelOut;
    //   len += angVelIn.convertToBuffer(sensors+len,  sensornumber-len );

    // reading ir sensorvalues
    if (conf.irAxis1 || conf.irAxis2 || conf.irAxis3){
      len += irSensorBank.get(sensors+len, sensornumber-len);
    }
  
    return len;
  }

  /**
   *Reads the actual motor commands from an array, and sets all motors of the snake to this values.
   *It is a linear allocation.
   *@param motors pointer to the array, motor values are scaled to [-1,1] 
   *@param motornumber length of the motor array
   **/
  void Sphererobot3Masses::setMotors ( const motor* motors, int motornumber ) {
    int len = min(motornumber, servono);
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
    //checks if either of both of the collision objects are part of the robot
    if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space) {
      if(o1 == (dGeomID)odeHandle.space) irSensorBank.sense(o2);
      if(o2 == (dGeomID)odeHandle.space) irSensorBank.sense(o1);

      // inner space collisions are not treated!
      int i,n;  
      const int N = 40;
      dContact contact[N];
    
      n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      for (i=0; i<n; i++) {
	if( contact[i].geom.g1 == object[Base]->getGeom() || contact[i].geom.g2 == object[Base]->getGeom() ){ 
	  // only treat collisions with envelop
	  contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactApprox1;
	  //	  dContactSoftERP | dContactSoftCFM | 
	  contact[i].surface.mu = 2.0;
	  contact[i].surface.slip1 = 0.005;
	  contact[i].surface.slip2 = 0.005;
	  //	contact[i].surface.soft_erp = 1; // 0.95;
	  //	contact[i].surface.soft_cfm = 0.00001;
	  dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
	  dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
	}
      }
      return true;
    } else {
      return false;
    }
  }


  /**
   *Returns the number of motors used by the snake.
   *@return number of motors
   **/
  int Sphererobot3Masses::getMotorNumber(){
    return servono;
  }

  /**
   *Returns the number of sensors used by the robot.
   *@return number of sensors
   **/
  int Sphererobot3Masses::getSensorNumber() {
    return conf.motorsensor * servono + conf.axisZsensor * servono + conf.axisXYZsensor * servono * 3 
      + (conf.irAxis1 + conf.irAxis2 + conf.irAxis3) * 2;
  }


  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  void Sphererobot3Masses::create(const osg::Matrix& pose){
    if (created) {
      destroy();
    }

    // create vehicle space and add it to the top level space
    odeHandle.space = dSimpleSpaceCreate (parentspace);
    Color c(osgHandle.color);
    c.alpha() = transparency;
    OsgHandle osgHandle_Base = osgHandle.changeColor(c);
    OsgHandle osgHandleX[3];
    osgHandleX[0] = osgHandle.changeColor(Color(1.0, 0.0, 0.0));
    osgHandleX[1] = osgHandle.changeColor(Color(0.0, 1.0, 0.0));
    osgHandleX[2] = osgHandle.changeColor(Color(0.0, 0.0, 1.0));

    //    object[Base] = new InvisibleSphere(conf.diameter/2);
    object[Base] = new Sphere(conf.diameter/2);
    //object[Base] = new InvisibleBox(conf.diameter, conf.diameter, conf.diameter);
    object[Base]->init(odeHandle, conf.spheremass, osgHandle_Base);
    object[Base]->setPose(pose);    

    Pos p(pose.getTrans());
    Primitive* pendular[3];

    //definition of the 3 Slider-Joints, which are the controled by the robot-controler
    for ( unsigned int n = 0; n < 3; n++ ) {
      pendular[n] = new Sphere(conf.pendulardiameter/2);
      pendular[n]->init(odeHandle, conf.pendularmass, osgHandleX[n], 
			Primitive::Body | Primitive::Draw); // without geom
      pendular[n]->setPose(pose);    

      joint[n] = new SliderJoint(object[Base], pendular[n],
				 p, osg::Vec3((n==0), (n==1), (n==2)));
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
				 conf.pendularmass); 
      
      axis[n] = new OSGCylinder(conf.diameter/100, conf.diameter - conf.diameter/100);
      axis[n]->init(osgHandleX[n], OSGPrimitive::Low);
    }
    object[Pendular1] = pendular[0]; 
    object[Pendular2] = pendular[1]; 
    object[Pendular3] = pendular[2]; 

    double sensorrange = conf.irsensorscale * conf.diameter;
    RaySensor::rayDrawMode drawMode = conf.drawIRs ? RaySensor::drawAll : RaySensor::drawSensor;

    irSensorBank.init(odeHandle, osgHandle );
    if (conf.irAxis1){
      for(int i=-1; i<2; i+=2){
	IRSensor* sensor = new IRSensor(1.5);
	Matrix R = Matrix::rotate(i*M_PI/2, 1, 0, 0) * Matrix::translate(0,-i*conf.diameter/2,0 );
	irSensorBank.registerSensor(sensor, object[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irAxis2){
      for(int i=-1; i<2; i+=2){
	IRSensor* sensor = new IRSensor(1.5);
	Matrix R = Matrix::rotate(i*M_PI/2, 0, 1, 0) * Matrix::translate(i*conf.diameter/2,0,0 );
	//	dRFromEulerAngles(R,i*M_PI/2,-i*M_PI/2,0);      
	irSensorBank.registerSensor(sensor, object[Base], R, sensorrange, drawMode);
      }
    }
    if (conf.irAxis3){
      for(int i=-1; i<2; i+=2){
	IRSensor* sensor = new IRSensor(1.5);
	Matrix R = Matrix::rotate( i==1 ? 0 : M_PI, 1, 0, 0) * Matrix::translate(0,0,i*conf.diameter/2);
	irSensorBank.registerSensor(sensor, object[Base], R, sensorrange, drawMode);
      }
    }
  }


  /** destroys vehicle and space
   */
  void Sphererobot3Masses::destroy(){
    if (created){
      for (int i=0; i<Last; i++){
	if(object[i]) delete object[i];
      }
      for (int i=0; i<servono; i++){
	if(joint[i]) delete joint[i];
	if(servo[i]) delete servo[i];
	if(axis[i]) delete axis[i];
      }
      irSensorBank.clear();
      dSpaceDestroy(odeHandle.space);
    }
    created=false;
  }

}


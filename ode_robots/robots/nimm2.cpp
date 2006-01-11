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
 *   Revision 1.21.4.10  2006-01-11 18:21:48  martius
 *   bumpers are moving
 *   wheel texture is okay
 *
 *   Revision 1.21.4.9  2006/01/10 17:16:22  martius
 *   sensorbank cleared on destroy
 *
 *   Revision 1.21.4.8  2005/12/29 16:47:40  martius
 *   joint has getPosition
 *
 *   Revision 1.21.4.7  2005/12/15 17:04:08  martius
 *   Primitives are not longer inherited from OSGPrimitive, moreover
 *   they aggregate them.
 *   Joint have better getter and setter
 *
 *   Revision 1.21.4.6  2005/12/14 15:37:09  martius
 *   robots are working with osg
 *
 *   Revision 1.21.4.5  2005/12/13 18:11:39  martius
 *   still trying to port robots
 *
 *   Revision 1.21.4.4  2005/12/06 17:38:17  martius
 *   *** empty log message ***
 *
 *   Revision 1.21.4.3  2005/11/16 11:26:52  martius
 *   moved to selforg
 *
 *   Revision 1.21.4.2  2005/11/15 12:29:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.21.4.1  2005/11/14 17:37:17  martius
 *   moved to selforg
 *
 *   Revision 1.21  2005/11/08 11:35:56  martius
 *   removed check for sensorbank because rays are disabled now
 *
 *   Revision 1.20  2005/11/04 14:43:27  martius
 *   added GPL
 *
 *                                                                 *
 ***************************************************************************/

#include <ode/ode.h>
#include <assert.h>
#include <osg/Matrix>

#include "nimm2.h"
#include "irsensor.h"

using namespace osg;

namespace lpzrobots {

  Nimm2::Nimm2(const OdeHandle& odehandle, const OsgHandle& osgHandle, const Nimm2Conf& conf = getDefaultConf()):
    OdeRobot(odehandle, osgHandle, "Nimm2"), conf(conf) { 

    created=false;
  
    //Nimm2 color ;-)
    this->osgHandle.color = Color(2, 156/255.0, 0, 1.0f);
  
    max_force   = conf.force*conf.size*conf.size;

    height=conf.size;

    width=conf.size/2; 
    radius=conf.size/4+conf.size/600;
    wheelthickness=conf.size/20;
    cmass=8*conf.size;  
    wmass=conf.size;  
    if(conf.singleMotor){
      sensorno=1; 
      motorno=1;  
    } else {
      sensorno=2; 
      motorno=2;  
    }

    if (conf.cigarMode){
      length=conf.size*2.0;         // long body
      wheeloffset= -length/4;  // wheels at the end of the cylinder, and the opposite endas the bumper
      number_bumpers=2;        // if wheels not at center only one bumper
      cmass=4*conf.size;
      max_force   = 2*conf.force*conf.size*conf.size;
    }
    else{
      length=conf.size/3;    // short body 
      wheeloffset=0.0;  // wheels at center of body
      number_bumpers=2; // if wheels at center 2 bumpers (one at each end)
    }

    sensorno+= conf.irFront * 2;
  };


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  void Nimm2::setMotors(const motor* motors, int motornumber){
    assert(created);
    assert(motornumber == motorno);
    if(conf.singleMotor){
      joint[0]->setParam(dParamVel2, motors[0]*conf.speed);       
      joint[0]->setParam(dParamFMax2,max_force);
      joint[0]->setParam(dParamVel2, motors[0]*conf.speed);       
      joint[0]->setParam(dParamFMax2,max_force);    
    } else {
      for (int i=0; i<2; i++){ 
	joint[0]->setParam(dParamVel2, motors[i]*conf.speed);       
	joint[0]->setParam(dParamFMax2,max_force);
      }
    }
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Nimm2::getSensors(sensor* sensors, int sensornumber){
    assert(created);
  
    int len = conf.singleMotor ? 1 : 2;
    for (int i=0; i<len; i++){
      sensors[i]=joint[i]->getPosition2Rate();
      sensors[i]/=conf.speed;  //scaling
    }
    // ask sensorbank for sensor values
    //  sensor+len is the starting point in the sensors array
    len += irSensorBank.get(sensors+len, sensornumber-len);
    return len;
  };


  void Nimm2::place(const Matrix& pose){
    // the position of the robot is the center of the body (without wheels)
    // to set the vehicle on the ground when the z component of the position is 0
    // width*0.6 is added (without this the wheels and half of the robot will be in the ground)    
    Matrix p2;
    p2 = pose * Matrix::translate(Vec3(0, 0, width*0.6)); 
    create(p2);
    
  };

  /** returns a vector with the positions of all segments of the robot
      @param poslist vector of positions (of all robot segments) 
      @return length of the list
  */
  int Nimm2::getSegmentsPosition(vector<Position> &poslist){
    assert(created);
    for (int i=0; i<3; i++){
      poslist.push_back(Position(dBodyGetPosition(object[i]->getBody())));
    }   
    return 3;
  };  



  /**
   * draws the vehicle
   */

  void Nimm2::update(){
    assert(created); // robot must exist
  
    for (int i=0; i<3; i++) { 
      object[i]->update();
    }
    for (int i=0; i < 2; i++) { 
      joint[i]->update();
    }
    if (conf.bumper){    
      for (int i=0; i<number_bumpers; i++){
	bumper[i].trans->update();
      }
    }
    irSensorBank.update();  
  }


  void Nimm2::mycallback(void *data, dGeomID o1, dGeomID o2){
    // Nimm2* me = (Nimm2*)data;  
    // o1 and o2 are member of the space

    // we ignore the collisions
  }

  bool Nimm2::collisionCallback(void *data, dGeomID o1, dGeomID o2){
    //checks if one of the collision objects is part of the robot
    assert(created);
    bool colwithme = false;  
    if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space ){
      if(o1 == (dGeomID)odeHandle.space) irSensorBank.sense(o2);
      if(o2 == (dGeomID)odeHandle.space) irSensorBank.sense(o1);

      bool colwithbody;  
      int i,n;  
      const int N = 10;
      dContact contact[N];
      //    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
      for (i=0; i<n; i++){
	colwithme = true; // there is at least one collision with some part of the robot (not sensors)
	colwithbody = false;
	if( contact[i].geom.g1 == object[0]->getGeom() || contact[i].geom.g2 == object[0]->getGeom() ||
	    ( bumper[0].trans && bumper[1].trans) && (
	    contact[i].geom.g1 == bumper[0].trans->getGeom() || 
	    contact[i].geom.g2 == bumper[0].trans->getGeom() ||
	    contact[i].geom.g1 == bumper[1].trans->getGeom() || 
	    contact[i].geom.g2 == bumper[1].trans->getGeom()) ){
	
	  colwithbody = true;
	}
	contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	  dContactSoftERP | dContactSoftCFM | dContactApprox1;
	contact[i].surface.slip1 = 0.005;
	contact[i].surface.slip2 = 0.005;
	if(colwithbody){
	  contact[i].surface.mu = 0.1; // small friction of smooth body
	  contact[i].surface.soft_erp = 0.9;
	  contact[i].surface.soft_cfm = 0.001;
	}else{
	  contact[i].surface.mu = 1.1; //large friction
	  contact[i].surface.soft_erp = 0.9;
	  contact[i].surface.soft_cfm = 0.001;
	}
	dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
	dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
      }        
    }
    return colwithme;
  }

  void Nimm2::doInternalStuff(const GlobalData& globalData){
    // dSpaceCollide(car_space, this, mycallback); // checks collisions in the car_space only (not needed)
    irSensorBank.reset();
  }

  /** creates vehicle at desired position 
      @param pos struct Position with desired position
  */
  void Nimm2::create(const Matrix& pose){
    if (created) {
      destroy();
    }

    // create vehicle space and add it to the top level space
    odeHandle.space = dSimpleSpaceCreate (parentspace);
  
    // create body
    Capsule* cap = new Capsule(width/2, length);
    cap->init(odeHandle, cmass, osgHandle);    
    // rotate and place body (here by 90° around the y-axis)
    cap->setPose(Matrix::rotate(M_PI/2, 0, 1, 0) * pose);
    cap->getOSGPrimitive()->setTexture("Images/wood.rgb");
    object[0]=cap;

    // bumper
    if (conf.bumper){    
      for (int i=0; i<number_bumpers; i++){
	bumper[i].bump = new Capsule(width/4, 2*radius+width/2);      
	bumper[i].trans = new Transform(object[0], bumper[i].bump, 
					Matrix::rotate(M_PI/2.0, Vec3(1, 0, 0)) * 
					Matrix::translate(0, 0, i==0 ? -(length/2) : (length/2)));
	bumper[i].trans->init(odeHandle, 0, osgHandle); 
      }
    }

    // wheel bodies
    OsgHandle osgHandleWheels(osgHandle);
    osgHandleWheels.color = Color(1.0,1.0,1.0);
    for (int i=1; i<3; i++) {
      if(conf.sphereWheels) {
	Sphere* wheel = new Sphere(radius);      
	wheel->init(odeHandle, wmass, osgHandleWheels);
      
	wheel->setPose(Matrix::rotate(M_PI/2.0, 1, 0, 0) * 
		       Matrix::translate(wheeloffset, (i==2 ? -1 : 1) * (width*0.5+wheelthickness), 0) *
		       pose); 
	wheel->getOSGPrimitive()->setTexture("Images/tire.rgb");
	object[i] = wheel;
      }else{
	//       Cylinder* wheel = new Cylinder(radius);      
	//       wheel->init(odeHandle, wmass, osgHandleWheels);
      
	//       wheel->setPose(Matrix::rotate(M_PI/2.0, Vec3(1,0,0)) * 
	// 		     Matrix::translate(wheeloffset, (i==2 ? -1 : 1) * (width*0.5+wheelthickness), 0)*
	//                   pose);
	//       object[i] = wheel;
      }
    }
  
    for (int i=0; i<2; i++) {
      joint[i] = new Hinge2Joint(object[0], object[i+1], object[i+1]->getPosition(), 
				 Vec3(0, 0, 1), Vec3(0, -1, 0));
      joint[i]->init(odeHandle, osgHandleWheels, true, 2.01 * radius);
      // set stops to make sure wheels always stay in alignment
      joint[i]->setParam(dParamLoStop,0);
      joint[i]->setParam(dParamHiStop,0);
    }

    irSensorBank.init(odeHandle, osgHandle);

    if (conf.irFront){
      for(int i=-1; i<2; i+=2){
	IRSensor* sensor = new IRSensor();
	irSensorBank.registerSensor(sensor, object[0], 
				    Matrix::rotate(i*M_PI/10, Vec3(0,0,1)) * 
				    Matrix::translate(0,i*width/10,length/2 + width/2 - width/60 ), 
				    2, RaySensor::drawAll);
      }
    }
    // TODO Back , Side sensors

    created=true;
  }; 


  /** destroys vehicle and space
   */
  void Nimm2::destroy(){
    if (created){
      for (int i=0; i<3; i++){
	if(object[i]) delete object[i];
      }
      for (int i=0; i<2; i++){
	if(joint[i]) delete joint[i];
      }
      for (int i=0; i<2; i++){
	if(bumper[i].bump) delete bumper[i].bump;
	if(bumper[i].trans) delete bumper[i].trans;
      }
      irSensorBank.clear();
      dSpaceDestroy(odeHandle.space);
    }
    created=false;
  }

}


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
 *   Revision 1.36  2007-11-07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.35  2007/09/27 16:01:28  der
 *   added to nimm2 the box version
 *
 *   Revision 1.34  2007/09/06 18:47:59  martius
 *   createNewSimpleSpace used
 *
 *   Revision 1.33  2007/08/23 15:53:14  martius
 *   rubber wheels
 *
 *   Revision 1.32  2007/08/23 15:40:27  martius
 *   removed ir derivative and collition control
 *   irsenors don't need explicit sense call
 *
 *   Revision 1.31  2007/06/21 16:23:52  martius
 *   collision threatment with general collision detection
 *    maybe modify body substance
 *   joints are deleted before objects
 *
 *   Revision 1.30  2007/05/07 21:12:19  robot3
 *   added experimental force sensors
 *
 *   Revision 1.29  2007/03/06 10:11:04  fhesse
 *   food removed
 *
 *   Revision 1.28  2007/03/05 10:49:32  fhesse
 *   food at position x=0, y=0 added
 *   motorcommand y set to zero when near food (controller doesn't know)
 *   after eating_time food is empty and motorcommand executed again
 *
 *   Revision 1.27  2007/02/23 15:14:17  martius
 *   *** empty log message ***
 *
 *   Revision 1.26  2006/12/11 18:24:36  martius
 *   memory freed
 *
 *   Revision 1.25  2006/11/23 10:25:47  fhesse
 *   side and rear infrared sensors added
 *   orientation of front ir sensors modified
 *
 *   Revision 1.24  2006/09/21 22:09:58  martius
 *   collision for mesh
 *
 *   Revision 1.23  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.22  2006/07/14 12:23:40  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.21.4.20  2006/06/29 16:39:55  robot3
 *   -you can now see bounding shapes if you type ./start -drawboundings
 *   -includes cleared up
 *   -abstractobstacle and abstractground have now .cpp-files
 *
 *   Revision 1.21.4.19  2006/06/25 17:00:32  martius
 *   Id
 *
 *   Revision 1.21.4.18  2006/06/25 16:57:14  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.21.4.17  2006/03/31 12:12:41  fhesse
 *   documentation improved
 *
 *   Revision 1.21.4.16  2006/02/01 18:33:39  martius
 *   use Axis type for Joint axis. very important, since otherwise Vec3 * pose is not the right direction vector anymore
 *
 *   Revision 1.21.4.15  2006/01/31 15:40:23  martius
 *   irRange configurable
 *   even higher and body is allways on th ground
 *
 *   Revision 1.21.4.14  2006/01/26 18:37:20  martius
 *   *** empty log message ***
 *
 *   Revision 1.21.4.13  2006/01/18 16:46:24  martius
 *   enabled coloring via osgHandle
 *
 *   Revision 1.21.4.12  2006/01/17 17:02:19  martius
 *   faster, stronger, more friction
 *
 *   Revision 1.21.4.11  2006/01/13 12:25:44  martius
 *   typo in setmotors
 *
 *   Revision 1.21.4.10  2006/01/11 18:21:48  martius
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
#include "osgprimitive.h"

using namespace osg;
using namespace std;


namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff, and default configuration
  Nimm2::Nimm2(const OdeHandle& odehandle, const OsgHandle& osgHandle,
	       const Nimm2Conf& conf, const string& name)
    : OdeRobot(odehandle, osgHandle, name, "$Id$"), conf(conf) {
	    contactPoints=0;

    // robot not created up to now
    created=false;

    // Nimm2 color ;-)
    this->osgHandle.color = Color(2, 156/255.0, 0, 1.0f);
    // can be overwritten in main.cpp of simulation with setColor
    
    // maximal used force is calculated from the force and size given in the configuration
    max_force   = conf.force*conf.size*conf.size;

    addParameter("speed", &this->conf.speed);
    addParameter("max_force", &max_force);

    height=conf.size;

    width=conf.size/2;  // radius of body
    radius=(width/2) * conf.wheelSize;  //radius of wheels
    wheelthickness=conf.size/10; // thickness of the wheels (if cylinder used, no spheres)
    cmass=4*conf.size;    // mass of body
    wmass=conf.size/5.0;  // mass of wheels
    if(conf.singleMotor){ //-> one dimensional robot
      sensorno=1;
      motorno=1;
    } else { // -> both wheels actuated independently
      sensorno=2;
      motorno=2;
    }

    if (conf.cigarMode){
      length=conf.size*2.0;    // long body
      wheeloffset= -length/4;  // wheels at the end of the cylinder, and the opposite endas the bumper
      number_bumpers=2;        // if wheels not at center only one bumper
      cmass=4*conf.size;
      max_force   = 2*conf.force*conf.size*conf.size;
    }
    else{
      length=conf.size/2;     // short body
      wheeloffset=0.0;        // wheels at center of body
      number_bumpers=2;       // if wheels at center 2 bumpers (one at each end)
    }

    // increase sensornumber by 2 if front infrared sensors are used
    sensorno+= conf.irFront * 2;
    // increase sensornumber by 4 if side infrared sensors are used
    sensorno+= conf.irSide * 4;
    // increase sensornumber by 2 if rear infrared sensors are used
    sensorno+= conf.irBack * 2;

	visForce = conf.visForce;
	if (visForce) {
		sumForce=0;
	}
  };


  Nimm2::~Nimm2(){
    destroy();
  }


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Nimm2::setMotors(const motor* motors, int motornumber){
    assert(created);
    assert(motornumber == motorno);
    if(conf.singleMotor){ // set the same motorcommand to both wheels
      joint[0]->setParam(dParamVel2, motors[0]*conf.speed); // set velocity
      joint[0]->setParam(dParamFMax2,max_force);            // set maximal force
      joint[1]->setParam(dParamVel2, motors[0]*conf.speed);
      joint[1]->setParam(dParamFMax2,max_force);
    } else {
      for (int i=0; i<2; i++){ // set different motorcommands to the wheels
	joint[i]->setParam(dParamVel2, motors[i]*conf.speed);
	joint[i]->setParam(dParamFMax2,max_force);
      }
    }
  };

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
//   sensor ir_old[4];
//   sensor ir_tmp[4];

  int Nimm2::getSensors(sensor* sensors, int sensornumber){
    assert(created); 

    // choose sensornumber according to number of motors
    // - one motorcommand -> one sensorvalue
    // - motors indepently controlled -> two sensorvalues
    int len = conf.singleMotor ? 1 : 2;
    for (int i=0; i<len; i++){
      sensors[i]=joint[i]->getPosition2Rate();  // readout wheel velocity
      sensors[i]/=conf.speed;  //scaling
    }
    // ask sensorbank for sensor values (from infrared sensors)
    //  sensor+len is the starting point in the sensors array
    if (conf.irFront || conf.irSide || conf.irBack){
      len += irSensorBank.get(sensors+len, sensornumber-len);
//       for (int i=0; i<4; i++){
//         ir_tmp[i]=sensors[2+i];
// 	sensors[2+i]=ir_tmp[i]-ir_old[i];
// 	ir_old[i]=ir_tmp[i];
//       }
    }
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
   * updates the osg notes and sensorbank
   */
  void Nimm2::update(){
    assert(created); // robot must exist

    for (int i=0; i<3; i++) { // update objects
      object[i]->update();
    }
    for (int i=0; i < 2; i++) { // update joints
      joint[i]->update();
    }
    if (conf.bumper){ // if bumper used update transform objects
      for (int i=0; i<number_bumpers; i++){
	bumper[i].trans->update();
      }
    }

    // update sensorbank with infrared sensors
    irSensorBank.update();

  }

   bool Nimm2::collisionCallback(void *data, dGeomID o1, dGeomID o2){
     return false;
   }

//   bool Nimm2::collisionCallback(void *data, dGeomID o1, dGeomID o2){
//     //checks if one of the collision objects is part of the robot
//     assert(created);
//     bool colwithme = false;

// 	  if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space ){
// // 	    // collision between anything and me was detected
// //       if(o1 == (dGeomID)odeHandle.space) irSensorBank.sense(o2);
// //       if(o2 == (dGeomID)odeHandle.space) irSensorBank.sense(o1);

//       bool colwithbody;
//       int i,n;
// 	    const int N = 50; // number of created contact points
// 	    dContact contact[N]; // array for contact points
//       //    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
// 	    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact)); // let ode create contacts of collision
// 		if (visForce)
// 			contactPoints+= n;
//       for (i=0; i<n; i++){
// 		colwithme = true; // there is at least one collision with some part of the robot (not sensors)
// 		colwithbody = false;
// 		if( contact[i].geom.g1 == object[0]->getGeom() || contact[i].geom.g2 == object[0]->getGeom() ||
// 	    	( bumper[0].trans && bumper[1].trans) && (
// 	    	contact[i].geom.g1 == bumper[0].trans->getGeom() ||
// 	    	contact[i].geom.g2 == bumper[0].trans->getGeom() ||
// 	    	contact[i].geom.g1 == bumper[1].trans->getGeom() ||
// 	    	contact[i].geom.g2 == bumper[1].trans->getGeom()) ){
// 			  colwithbody = true;
// 		}
// 		contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
// 	  	dContactSoftERP | dContactSoftCFM | dContactApprox1;
// 		// one could try to make the body sliping along its axis by using
// 		//  sin(alpha), cos(alpha) for sliping params (only for body collisions)
// 		contact[i].surface.slip1 = 0.005; // sliping in x
// 		contact[i].surface.slip2 = 0.005; // sliping in y
// 		if(colwithbody){
// 	  		contact[i].surface.mu = 0.1; // small friction of smooth body
// 		  	// contact[i].surface.soft_erp = 0.8;
// 	  		//contact[i].surface.soft_cfm = 0.1;
// 	  		//contact[i].surface.soft_erp = 0.99;
// 	  		//contact[i].surface.soft_cfm = 0.01;

// 	  		contact[i].surface.soft_erp = 0.8;
// 	  		contact[i].surface.soft_cfm = 0.01;
// 		}else{
// 			// collision with external world!
// 		  	contact[i].surface.mu = 5.0; //large friction
// 	  		contact[i].surface.soft_erp = 0.8;
// 	  		contact[i].surface.soft_cfm = 0.01;
// 			if (visForce) {
// 				sumForce+=contact[i].geom.depth;
// 			}
// 		}
// 		dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
// 		dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2));
//       }
//     }
//     return colwithme;
//   }

  void Nimm2::doInternalStuff(GlobalData& globalData){
    // dSpaceCollide(car_space, this, mycallback); // checks collisions in the car_space only (not needed)
    irSensorBank.reset(); // reset sensorbank (infrared sensors)
	  if (visForce) {
		  sumForce=0;
		  contactPoints=0;
	  }
  }

  /** creates vehicle at desired position
      @param pos struct Position with desired position
  */
  void Nimm2::create(const Matrix& pose){
    if (created) {
      destroy();
    }

    // create vehicle space and add it to the top level space
    // robot will be inserted in the vehicle space
    odeHandle.createNewSimpleSpace(parentspace,true);

    OdeHandle wheelHandle(odeHandle);
    wheelHandle.substance.toRubber(40);

    // create body
    // - create cylinder for main body (with radius and length)
    // - init cylinder with odehandle, mass and osghandle
    // - rotate and place body (here by 90 around the y-axis)
    // - set texture for cylinder
    // - put it into object[0]

    if (conf.boxMode) {
      Box* box = new Box(width/4*3,width/2, length/2);
      box->init(odeHandle, cmass, osgHandle);
      box->setPose(Matrix::translate(0, 0, -1) * Matrix::rotate(M_PI/2, 0, 1, 0) * pose);
      box->getOSGPrimitive()->setTexture("Images/wood.rgb");
      box->substance.toMetal(0);
      object[0]=box;
    } else {
      Capsule* cap = new Capsule(width/2, length);
      cap->init(odeHandle, cmass, osgHandle);
      cap->setPose(Matrix::rotate(M_PI/2, 0, 1, 0) * pose);
      cap->getOSGPrimitive()->setTexture("Images/wood.rgb");
      object[0]=cap;
    }

    // create bumper if required
    // - create cylinder with radius and length
    // - position bumper relative to main body
    //  (using transform object "glues" it together without using joints, see ODE documentation)
    // - init cylinder with odehandle, mass and osghandle
    if (conf.bumper){
      for (int i=0; i<number_bumpers; i++){
	bumper[i].bump = new Capsule(width/4, 2*radius+width/2);
	bumper[i].trans = new Transform(object[0], bumper[i].bump,
					Matrix::rotate(M_PI/2.0, Vec3(1, 0, 0)) *
					Matrix::translate(0, 0, i==0 ? -(length/2) : (length/2)));
	bumper[i].trans->init(odeHandle, 0, osgHandle);
      }
    }

    // create wheel bodies
    OsgHandle osgHandleWheels(osgHandle);    // new osghandle with color for wheels
    osgHandleWheels.color = Color(1.0,1.0,1.0);
    for (int i=1; i<3; i++) {
      if(conf.sphereWheels) { // for spherical wheels
	Sphere* wheel = new Sphere(radius);      // create spheres
	wheel->init(wheelHandle, wmass, osgHandleWheels); // init with odehandle, mass, and osghandle

	wheel->setPose(Matrix::rotate(M_PI/2.0, 1, 0, 0) *
		       Matrix::translate(wheeloffset,
					 (i==2 ? -1 : 1) * (width*0.5+wheelthickness), 0) *
		       pose); // place wheels
	wheel->getOSGPrimitive()->setTexture("Images/tire.rgb"); // set texture for wheels
	object[i] = wheel;
      }else{ // for "normal" wheels
	Cylinder* wheel = new Cylinder(radius, wheelthickness);
	wheel->init(wheelHandle, wmass, osgHandleWheels);
	wheel->setPose(Matrix::rotate(M_PI/2.0, Vec3(1,0,0)) *
		       Matrix::translate(wheeloffset,
					 (i==2 ? -1 : 1) * (width*0.5+wheelthickness), 0)* pose);
	wheel->getOSGPrimitive()->setTexture("Images/tire.rgb"); // set texture for wheels
	object[i] = wheel;
      }
    }


    // set joints between wheels and body (see ODE documentation)
    // - create joint
    // - init joint
    // - set stop parameters
    for (int i=0; i<2; i++) {
      joint[i] = new Hinge2Joint(object[0], object[i+1], object[i+1]->getPosition(),
				 Axis(0, 0, 1)*pose, Axis(0, -1, 0)*pose);
      joint[i]->init(odeHandle, osgHandleWheels, true, conf.sphereWheels ? 2.01 * radius : wheelthickness*1.05 );
      // set stops to make sure wheels always stay in alignment
      joint[i]->setParam(dParamLoStop,0);
      joint[i]->setParam(dParamHiStop,0);
    }


    /* initialize sensorbank (for use of infrared sensors)
     * sensor values (if sensors used) are saved in the vector of
     * sensorvalues in the following order:
     * front left
     * front right
     * right front
     * right rear
     * rear rigth
     * rear left
     * left rear
     * left front
    */
    irSensorBank.init(odeHandle, osgHandle);
    if (conf.irFront){ // add front left and front right infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
	IRSensor* sensor = new IRSensor();
	irSensorBank.registerSensor(sensor, object[0],
				    Matrix::rotate(i*M_PI/10, Vec3(1,0,0)) *
				    Matrix::translate(0,-i*width/10,length/2 + width/2 - width/60 ),
				    conf.irRange, RaySensor::drawAll);
      }
    }
    if (conf.irSide){ // add right front and right rear infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
	IRSensor* sensor = new IRSensor();
	if (conf.bumper){ // if bumpers used place on bumper
	  irSensorBank.registerSensor(sensor, object[0],
				      //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
				      Matrix::rotate(M_PI/2, Vec3(1,0,0)) *
				      Matrix::translate(0,-width,-i*(length/2) ),
				      conf.irRange, RaySensor::drawAll);

	}else{ // place on body
	  irSensorBank.registerSensor(sensor, object[0],
				      //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
				      Matrix::rotate(M_PI/2, Vec3(1,0,0)) *
				      Matrix::translate(0,-width/2,i*(length/2) ),
				      conf.irRange, RaySensor::drawAll);
	}
      }
    }
    if (conf.irBack){ // add rear right and rear left infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
	IRSensor* sensor = new IRSensor();
	irSensorBank.registerSensor(sensor, object[0],
				    Matrix::rotate(-i*M_PI/10, Vec3(1,0,0)) *
				    Matrix::rotate(i*M_PI, Vec3(0,1,0)) *
				    Matrix::translate(0,i*width/10,-(length/2 + width/2 - width/60) ),
				    conf.irRange, RaySensor::drawAll);
      }
    }
    if (conf.irSide){ // add left rear and left front infrared sensor to sensorbank if required
      for(int i=-1; i<2; i+=2){
	IRSensor* sensor = new IRSensor();
	if (conf.bumper){ // if bumpers used place on bumper
	  irSensorBank.registerSensor(sensor, object[0],
				      //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
				      Matrix::rotate(-M_PI/2, Vec3(1,0,0)) *
				      Matrix::translate(0,width,i*(length/2) ),
				      conf.irRange, RaySensor::drawAll);

	} else { // else place onb body
	  irSensorBank.registerSensor(sensor, object[0],
				      //Matrix::rotate(i*M_PI/2, Vec3(0,0,1)) *
				      Matrix::rotate(-M_PI/2, Vec3(1,0,0)) *
				      Matrix::translate(0,width/2,i*(length/2) ),
				      conf.irRange, RaySensor::drawAll);
	}
      }
    }
    created=true;
  };


  /** destroys vehicle and space
   */
  void Nimm2::destroy(){
    if (created){
      irSensorBank.clear();
      for (int i=0; i<2; i++){
	if(joint[i]) delete joint[i];
      }
      for (int i=0; i<2; i++){
	//	if(bumper[i].bump) delete bumper[i].bump; is done by transform primitive
	if(bumper[i].trans) delete bumper[i].trans;
      }
      for (int i=0; i<3; i++){ 
	if(object[i]) delete object[i];
      }
      odeHandle.deleteSpace();
    }
    created=false;
  }
/*
	std::list<Inspectable::iparamkey> Nimm2::getInternalParamNames() const{
		std::list<Inspectable::iparamkey> keylist;
		if (visForce) {
			keylist+=std::string("SumForce");
			std::cout << "returning: SumForce!" << std::endl;
		}
		std::cout << "beep name.";
		return keylist;
	}


	std::list<Inspectable::iparamval> Nimm2::getInternalParams() const {
		std::list<Inspectable::iparamval> vallist;
		if (visForce) {
			vallist+=sumForce;
			std::cout << "SumForce =" << sumForce << std::endl;
		}
		std::cout << "beep val.";
		return vallist;
	}
*/
}


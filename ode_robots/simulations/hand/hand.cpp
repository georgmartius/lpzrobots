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
 *   Revision 1.2  2006-09-21 16:15:57  der
 *   *** empty log message ***
 *
 *   Revision 1.9.4.6  2006/06/25 17:00:32  martius
 *   Id
 *
 *   Revision 1.9.4.5  2006/06/25 16:57:13  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.9.4.4  2005/12/30 22:54:38  martius
 *   removed parentspace!
 *
 *   Revision 1.9.4.3  2005/12/21 17:34:59  martius
 *   moved to osg
 *
 *   Revision 1.9.4.2  2005/11/15 12:29:26  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.9.4.1  2005/11/14 17:37:17  martius
 *   moved to selforg
 *
 *   Revision 1.9  2005/11/09 13:26:31  martius
 *   added factorSensors
 *
 *   Revision 1.8  2005/10/06 17:14:24  martius
 *   switched to stl lists
 *
 *   Revision 1.7  2005/09/22 12:24:37  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.6  2005/08/31 17:18:15  fhesse
 *   setTextures added, Mass is now sphere (not box anymore)
 *
 *   Revision 1.5  2005/08/29 06:41:22  martius
 *   kosmetik
 *
 *   Revision 1.4  2005/08/03 20:35:28  martius
 *   *** empty log message ***
 *
 *   Revision 1.3  2005/07/27 13:23:09  martius
 *   new color and position construction
 *
 *   Revision 1.2  2005/07/26 17:04:21  martius
 *   lives in its own space now
 *
 *   Revision 1.1  2005/07/21 12:17:04  fhesse
 *   new hurling snake, todo: add collision space, clean up, comment
 *
 *         
 *                                                                 *
 ***************************************************************************/
#include <osg/Matrix>
#include "mathutils.h"
#include "hand.h"
#include <osgprimitive.h>

//using namespace std;

namespace lpzrobots {

  /**
   * Constructor
   * @param w world in which robot should be created
   * @param s space in which robot should be created
   * @param c contactgroup for collision treatment
   */

 Hand::Hand(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const HandConf& conf, const std::string& name)
    : OdeRobot(odeHandle, osgHandle, name, "$Id$"), conf(conf), oldp(0,0,0){
/*  
Hand::Hand(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const string& name)
    : OdeRobot(odeHandle, osgHandle, name, "$Id$"), oldp(0,0,0){
*/
    factorForce=3.0;
    
    frictionGround=0.3;
    velocity=2;
    power=5;

    created=false;

    this->osgHandle.color=Color(0,1,1);

   // NUM= 10;		/* number of spheres */
 //   SIDE= 0.2;		/* side length of a box */
   // MASS= 0.5;		/* mass of a sphere*/
     RADIUS= 0.1732f *2;	/* sphere radius */
     sensor_number=15;
  //  sensorno = 2;
  //  motorno  = 2;



  };

 
  void Hand::update(){
    assert(created); // robot must exist
  
    for (std::vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
      if(*i) (*i)->update();
    }

    for (std::vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
      if(*i) (*i)->update();
    }
   
   if(contact_joint_created){

	for (std::vector<OSGPrimitive*>::iterator i = osg_objects.begin(); i!=  osg_objects.end(); i++)
	 {
 	if(*i) delete *i;
      }
      osg_objects.clear();
contact_joint_created=false;
	}

// update sensorbank with infrared sensors
    irSensorBank.update();
  			}

  void Hand::place(const osg::Matrix& pose){
    // lift the snake about its radius
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(osg::Vec3(1, 14, RADIUS)); 
    create(p2);    
  };

  void Hand::doInternalStuff(const GlobalData& global){
    // mycallback is called for internal collisions! Only once per step
    dSpaceCollide(odeHandle.space, this, mycallback);
    irSensorBank.reset(); // reset sensorbank (infrared sensors) 
  }

    // internal collisions
  void Hand::mycallback(void *data, dGeomID o1, dGeomID o2){

// connected bodies are allowed to intersect
// if( dAreConnected(dGeomGetBody(o1), dGeomGetBody(o2)) )
// return;


//if(o1 == (dGeomID)odeHandle.space) me->irSensorBank.sense(o2);
//if(o2 == (dGeomID)odeHandle.space) me->irSensorBank.sense(o1);

    Hand* me = (Hand*)data;  


    int i=0;
    int o1_index= -1;
    int o2_index= -1;
    for (std::vector<Primitive*>::iterator n = me->objects.begin(); n!= me->objects.end(); n++, i++){      
      if( (*n)->getGeom() == o1)
	o1_index=i;
      if( (*n)->getGeom() == o2)
	o2_index=i;
    }

if(o1_index >= 0 && o2_index >= 0){
	


    int n;
  const int N = 10;
  dContact contact[N];
  n=dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
if(n >0){
	for (int i=0; i<n; i++) {

		contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
			dContactSoftERP | dContactSoftCFM | dContactApprox1;
		contact[i].surface.mu = dInfinity;
		contact[i].surface.slip1 = 0.1;
		contact[i].surface.slip2 = 0.1;
		contact[i].surface.soft_erp = 0.5;
		contact[i].surface.soft_cfm = 0.3;
      dJointID c = dJointCreateContact( me->odeHandle.world, me->odeHandle.jointGroup, &contact[i]);
      dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;	      

//    for (std::vector<Primitive*>::iterator u = me->objects.begin()+2; u!= me->objects.end(); u++ ){      
for (unsigned int u=2; u < me->objects.size(); u++){
if((contact[i].geom.g1 == me->objects[1]->getGeom() || contact[i].geom.g2 == me->objects[1]->getGeom() )&& (contact[i].geom.g1 == me->objects[u]->getGeom() || contact[i].geom.g2 == me->objects[u]->getGeom()))
{
contact[i].geom.depth=5;
}

//in this just not let the ir-sensors to enter in the palm

if ((contact[i].geom.g1 == me->objects[1]->getGeom() || contact[i].geom.g2 == me->objects[1]->getGeom()) && (contact[i].geom.g1 == me->ir_sensors[u-2]->getGeomID() || contact[i].geom.g2 ==  me->ir_sensors[u-2]->getGeomID()))
{
contact[i].geom.depth=15;
}

}
/*
for (unsigned int u=2; u < me->objects.size(); u++){
//(contact[i].geom.g1 == me->objects[1]->getGeom() || contact[i].geom.g2 == me->objects[1]->getGeom() )
if (((contact[i].geom.g1 == me->objects[1]->getGeom() || contact[i].geom.g2 == me->objects[1]->getGeom()) || (contact[i].geom.g1 == me->objects[0]->getGeom() || contact[i].geom.g2 == me->objects[0]->getGeom())|| 
contact[i].geom.g1 == me->objects[u]->getGeom() || contact[i].geom.g2 == me->objects[u]->getGeom()) && (contact[i].geom.g1 == me->ir_sensors[u-2]->getGeomID() || contact[i].geom.g2 ==  me->ir_sensors[u-2]->getGeomID()))
{
contact[i].geom.depth=0.0005;
}

}
*/
//|| (contact[i].geom.g1 == me->ir_sensors[u-2]->getGeomID() || contact[i].geom.g2 ==  me->ir_sensors[u-2]->getGeomID()))
//|| contact[i].geom.g1 == me->irSensorBank.sense(o2)->getGeomID() || contact[i].geom.g2 == me->irSensorBank.sense(o2)->getGeomID()
    }
  }

}
}
  bool Hand::collisionCallback(void *data, dGeomID o1, dGeomID o2){

OSGBox* OsgContactPoint;


    //checks if one of the collision objects is part of the robot
    if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space){

      // the rest is for collisions of some snake elements with the rest of the world
     	/*
  // only things colliding with hand space are considered
  int g1 = (o1 == (dGeomID)odeHandle.space|| o2 == (dGeomID)odeHandle.space);
  if (!g1) return;
*/

      if(o1 == (dGeomID)odeHandle.space) irSensorBank.sense(o2);
      if(o2 == (dGeomID)odeHandle.space) irSensorBank.sense(o1);

  int n;
  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
	for (int i=0; i<n; i++) {

		/*
		// connected bodies are allowed to intersect
		if( dAreConnected(dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2)) )
			return;
			*/

if( contact[i].geom.g1 == objects[1]->getGeom() || contact[i].geom.g2 == objects[1]->getGeom())
{
contact[i].geom.depth=0.008;
}
//if all finger-segmens should be able to penetrate an object
//for (unsigned int u=2; u<objects.size(); u++){

for (unsigned int u=4; u<objects.size(); u+=3){
if (contact[i].geom.g1 == objects[u]->getGeom() || contact[i].geom.g2 == objects[u]->getGeom())
{
contact[i].geom.depth=0.008;
}
}
/*
else
if( (contact[i].geom.g1 == objects[1]->getGeom() || contact[i].geom.g2 == objects[1]->getGeom()) && ((contact[i].geom.g1 == objects[3]->getGeom() || contact[i].geom.g2 == objects[3]->getGeom())||(contact[i].geom.g1 == objects[4]->getGeom() || contact[i].geom.g2 ==objects[4]->getGeom())))
{
contact[i].geom.depth=2;
} 
*/

/*
else if ( (contact[i].geom.g1 == objects[1]->getGeom() || contact[i].geom.g2 == objects[1]->getGeom()) && (for(int u=5; u<18; u+=3){contact[i].geom.g1 == objects[u]->getGeom() || contact[i].geom.g2 == objects[u]->getGeom()
} )
{
contact[i].geom.depth=1;
}
*/

/*
{
contact[i].geom.g1 == objects[u]->getGeom() || contact[i].geom.g2 == objects[u]->getGeom();
}

(
( 
for(int u=5; u<18; u+=3)
{
contact[i].geom.g1 == objects[u]->getGeom() || contact[i].geom.g2 == objects[u]->getGeom();
}
)
||
(
for (int u=18; u<23; u+=1)
{
contact[i].geom.g1 == objects[u]->getGeom() || contact[i].geom.g2 == objects[u]->getGeom();
}
)
)
)
{
contact[i].geom.depth=1;
}
*/
		contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
			dContactSoftERP | dContactSoftCFM | dContactApprox1;
		contact[i].surface.mu = dInfinity;
		contact[i].surface.slip1 = 0.1;
		contact[i].surface.slip2 = 0.1;
		contact[i].surface.soft_erp = 0.5;
		contact[i].surface.soft_cfm = 0.3;
	dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
	dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;

if(conf.show_contacts)
{
    dMatrix3 RI;
    dRSetIdentity (RI);
    OsgContactPoint = new OSGBox(0.02, 0.02, 0.02);
    OsgContactPoint->init(osgHandle);
    OsgContactPoint->setMatrix(osg::Matrix::translate(osg::Vec3(0,0,0.2/2.0)) * osgPose(contact[i].geom.pos,RI));
    OsgContactPoint->setColor(Color(0.0, 0.0, 1.0));
osg_objects.push_back(OsgContactPoint);
contact_joint_created=true;
 //delete OsgContactPoint;
 
}


      }
}
//sleep(0);

      return true;
    }
    return false;
  }

  

  /** returns actual sensorvalues
      @param sensors sensors scaled to [-1,1] 
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Hand::getSensors(sensor* sensors, int sensornumber){
 //   int len = (sensornumber < sensorno)? sensornumber : sensorno;
//int len = min(sensornumber/2, (int)joints.size()+2); 
for (int i = 0; i < sensor_number; i++){


if (i==0){
// It can be used if you want to use the coordinats of bodies to set sensors.
/*
    Pos p(objects[1]->getPosition());      //read actual position
    Pos s = (p - oldp)*conf.factorSensor;
    
    sensors[0]=s.x();
    sensors[1]=s.y();
    oldp=p; 
*/
sensors[0]=((AngularMotor*)palm_motor_joint)->getParam(dParamVel2);
sensors[1]=((AngularMotor*)palm_motor_joint)->getParam(dParamVel3);
//	sensors[0]=((AngularMotor1Axis*)palm_motor_joint)->get(0);
//      sensors[1]=((AngularMotor1Axis*)palm_motor_joint)->get(1);

//	sensors[0]=((AngularMotor3AxisEuler*)palm_motor_joint)->get(0);
//      sensors[1]=((AngularMotor3AxisEuler*)palm_motor_joint)->get(1);

}
else if(i==1){
/*    Pos p(objects[2]->getPosition());      //read actual position
    Pos s = (p - oldp)*conf.factorSensor;
    
    sensors[2]=s.x();
    sensors[3]=s.y();
    sensors[4]=s.z();
    oldp=p; */
sensors[2]=((AngularMotor*)thumb_motor_joint)->getParam(dParamVel);
sensors[3]=((AngularMotor*)thumb_motor_joint)->getParam(dParamVel2);
sensors[4]=((AngularMotor*)thumb_motor_joint)->getParam(dParamVel3);
//	sensors[2]=((AngularMotor3AxisEuler*)thumb_motor_joint)->get(0);
//    sensors[3]=((AngularMotor3AxisEuler*)thumb_motor_joint)->get(2);

}
else  {

switch(conf.set_typ_of_motor)
{
  case(With_servo_motor):
	sensors[3+i] =  servos[i-2]->get();
  break;
  case(Without_servo_motor):
	sensors[3+i] = conf.factorSensor * ((HingeJoint*)joints[i])->getPosition1Rate();
  break;
  default:
	sensors[3+i] = conf.factorSensor * ((HingeJoint*)joints[i])->getPosition1Rate();
  break;

}
//	sensors[2*i+1] = conf.factorSensor * ((HingeJoint*)joints[i])->getPosition1();
	}
}
//  sensors[n]   = conf.factorSensor * ((HingeJoint*)joints[n])->getPosition1Rate();
int len=sensor_number;
if(conf.ir_sensor_used)
{
len += irSensorBank.get(sensors+sensor_number, sensornumber-sensor_number);
}
  return len+3 ;
  }


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1] 
      @param motornumber length of the motor array
  */
  void Hand::setMotors(const motor* motors, int motornumber){    

assert(created);
// there will always be an even number of motors
    // (two sensors/motors per joint)
//    int len = min(motornumber/2, (int)joints.size()+2);
for (int i = 0; i < sensor_number; i++){

//------------ for servor-sensors----------------------------------------------------

//servos[i]->set(motors[2*i], motors[2*i+1]);


if (i==0){


//dBodyAddForce (objects[1]->getBody(),motors[i]*factorForce,motors[i+1]*factorForce,0);
((AngularMotor3AxisEuler*)palm_motor_joint)->set(1, motors[i]* velocity);
((AngularMotor3AxisEuler*)palm_motor_joint)->set(2, motors[i+1]* velocity);

}

else if(i==1){

((AngularMotor3AxisEuler*)thumb_motor_joint)->set(0, motors[i+1]* velocity);
((AngularMotor3AxisEuler*)thumb_motor_joint)->set(1, motors[i+2]* velocity);
((AngularMotor3AxisEuler*)thumb_motor_joint)->set(2, motors[i+3]* velocity);

//dBodyAddForce (objects[2]->getBody(),motors[i+1]*factorForce,motors[i+2]*factorForce,0);
}
else{

switch(conf.set_typ_of_motor)
{
case(Without_servo_motor):
((HingeJoint*)joints[i])->setParam ( dParamVel , motors[3+i]* velocity );
((HingeJoint*)joints[i])->setParam(dParamFMax, power); 
break;
case(With_servo_motor):
servos[i-2]->set(motors[3+i]);
servos[i-2]->setPower(conf.servo_motor_Power);
break;
default:
((HingeJoint*)joints[i])->setParam ( dParamVel , motors[3+i]* velocity );
((HingeJoint*)joints[i])->setParam(dParamFMax, power); 
break;

}
//	      ((HingeJoint*)joints[i])->addTorque(factorForce * motors[3+i]);
 	      //((HingeJoint*)joints[i])->addTorque(factorForce * motors[2*i+1]);
}
    //  dBodyAddForce (object[NUM-1].body,motors[0]*factorForce,motors[1]*factorForce,motors[2]*factorForce);
/* for (int i=0; i<NUM; i++){
  dBodyAddForce (object[i]->getBody(),motors[i]*factorForce,motors[i+1]*factorForce,0);
}*/
/*    dBodyAddForce (object[NUM-1]->getBody(),motors[0]*factorForce,motors[1]*factorForce,0);
  dBodyAddForce (object[0]->getBody(),motors[0]*factorForce,motors[1]*factorForce,0);*/
//}
  }
}

  /** returns number of sensors
   */
  int Hand::getSensorNumber(){
if(conf.ir_sensor_used)
{
    return sensor_number+3+conf.number_of_ir_sensors;
}
else
{
    return sensor_number+3;
}
  }

  /** returns number of motors
   */
  int Hand::getMotorNumber(){
    return sensor_number+3;
  }


  /** returns a vector with the positions of all segments of the robot
      @param vector of positions (of all robot segments) 
      @return length of the list
  */
/*
  int Hand::getSegmentsPosition(vector<Position> &poslist){
    int number_objects=0;// or also with objects.size()
    Position pos;
for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++     		){
      if(*i) { Pos p = (*i)->getPosition();
	         poslist.push_back(p.toPosition());
		   ++number_objects;
		 } 
    }
    
	return number_objects; // return objects.size();
  }

*/

  void Hand::create(const osg::Matrix& pose){   
    if (created){
      destroy();
    }

//initizialization
double power=5;
// double z=2; // z shift added to whole thing
 /*thumb1=0;
 thumb2=0;
 thumb3=0;
 finger_force=0;
 palm_torque=0;
 gripmode=lateral;*/



 

    // create vehicle space and add it to parentspace
    odeHandle.space = dSimpleSpaceCreate (parentspace);

/*
Primitive* ground;
ground = new Plane();
ground->init ( odeHandle , 1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
ground->setPose(osg::Matrix::translate(0.0,0.0,0.0));
objects.push_back(ground);	
*/
//--------------------forearm-------------------------/////////////////////
Primitive*  forearm = new Capsule(0.5,5);

//Anstatt von odeHandle hand_space
  forearm-> init ( odeHandle , 1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
	  forearm->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) *	     
		      	    osg::Matrix::translate( conf.invert*(-3.7+conf.x),(0+conf.y),(0.5+conf.z)));//(*) pose wurde an der Stelle weggelassen, kann dann spaeter dazugenommen werden.

//  forearm->getOSGPrimitive()->setTexture("Images/finger_handflaeche2.png"); 
objects.push_back(forearm);

//--------------------palm------------------------------------------------------
 Primitive* palm = new Box(1.4,1.4,0.3);//Cylinder(1.0,0.3);
// Primitive* palm = new Box(2.0,1.8,0.3);
//Anstatt von odeHandle hand_space
  palm-> init (odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);

  palm->setPose(osg::Matrix::translate(conf.invert*(0+conf.x),(0+conf.y),(0.5+conf.z)));
palm->getOSGPrimitive()->setTexture("Images/light_chess.rgb");
//  palm->getOSGPrimitive()->setTexture("Images/ground.rgb"); 

objects.push_back(palm);

//-----box for the palm as underground, in order ir-sensors work properly--------
Primitive* palm_cylinder = new Cylinder(1.0,0.3);
Primitive* box_in_cylinder_palm = new Transform(objects[1],palm_cylinder, osg::Matrix::translate(0, 0,0));
box_in_cylinder_palm -> init (odeHandle , 0 , osgHandle);


//-------------------BallJoint between forearm and palm-------------------------

 Joint* forearm_palm = new BallJoint(forearm, palm,
 					      Pos(conf.invert*(-1.2+conf.x), (0+conf.y), (0.5+conf.z)));//(*) pose wurde an dieser Stelle weggelassen, kann dann aber spaeter dazugenommen werden.
  forearm_palm->init(odeHandle, osgHandle, true, 0.2);//Die groesse muss hier neu eingestellt werden, bei Ode nicht einstellbar!!!
joints.push_back(forearm_palm);

//-------------AngularMotor for BallJoint---------------------------------------
Axis axis1 = Axis(1,0,0);
Axis axis3 = Axis(0,0,1);
//int axis_number=0;

//double velocity=palm_torque*M_PI/2;


palm_motor_joint = new AngularMotor3AxisEuler(odeHandle, (BallJoint*) forearm_palm, 
			   axis1, axis3, power);
//palm_motor_joint->set(axis_number, conf.velocity)

//palm_motor_joint->init(odeHandle, osgHandle, true, 0.2 );

 
//has to be set if you want hold the hand unshaked
((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop, 0);
((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop, 0);
((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop2, 0);
((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop2, 0);
((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop3, 0);
((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop3,  0);

/*
((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop2, -M_PI/360);
((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop2,  M_PI/360);
((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop3, -M_PI/360);
((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop3,  M_PI/360);
*/
/*
((AngularMotor3AxisEuler*)palm_motor_joint)->setHiLoStopParams(-M_PI/360, M_PI/360,  2);
*/
frictionmotors.push_back(palm_motor_joint);
//frictionmotors.push_back(new AngularMotor3AxisEuler(odeHandle, (BallJoint*) forearm_palm, axis1, axis3, conf.power));

//------------------------thumb ----------------------------------------------
Primitive *thumb_b,*thumb_t,*thumb_c;

thumb_b = new Capsule(0.2,0.1);
thumb_b -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
thumb_b ->setPose(osg::Matrix::rotate(conf.invert*2*M_PI, osg::Vec3(1, 0, 0),conf.invert*2*M_PI, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1)) * osg::Matrix::translate(conf.invert*(-0.6+conf.x),(-0.8+conf.y),  (conf.invert==-1 ?(0.4+conf.z) : (0.6+conf.z))));

break;

case(Is_Draw_under_180_degree):

//---ToDo
//thumb_b ->setPose(osg::Matrix::translate(conf.invert*(0), (0), (0.59))*(index_b->getPose()) * osg::Matrix::rotate(conf.invert*M_PI/2, 1, 0, 0));

//index_b ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(1.0+conf.x), (-0.6+conf.y), (0.5+conf.z)));

//thumb_b ->setPose(osg::Matrix::rotate(conf.invert*M_PI/2, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1)) * osg::Matrix::translate(conf.invert*(-0.6+conf.x),(-0.8+conf.y), (0.5+conf.z)));

//thumb_b ->setPose(osg::Matrix::rotate(conf.invert*M_PI/2, osg::Vec3(1, 0, 0),conf.finger_winkel, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1)) * osg::Matrix::translate((-forearm->getPosition()+palm->getPosition())/3.6)*(palm->getPose())*osg::Matrix::translate(-1.6,-0.8,0));
if (conf.finger_winkel==M_PI/2)
{
thumb_b ->setPose(osg::Matrix::rotate(M_PI/2, osg::Vec3(1, 0, 0),conf.invert*conf.finger_winkel, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1)) *(palm->getPose())*osg::Matrix::translate(conf.invert*-0.6,-0.8,0));
}
else if (conf.finger_winkel==M_PI/6)
{
thumb_b ->setPose(osg::Matrix::rotate(conf.finger_winkel, osg::Vec3(1, 0, 0),0.0, osg::Vec3(0, 1, 0),-2*M_PI,osg::Vec3(0, 0, 1)) *(palm->getPose())*osg::Matrix::translate(conf.invert*-0.6,-0.8,0));
}
/*
if (conf.finger_winkel==M_PI/2)
{
ring_b->setPose(osg::Matrix::translate(conf.invert*(0), (0.9), conf.invert*(0.3))*(index_b->getPose()));
}
else if (conf.finger_winkel==2*M_PI)
{
ring_b->setPose(osg::Matrix::translate(conf.invert*(0.3), (0.9), conf.invert*(0))*(index_b->getPose()));
}
else if (conf.finger_winkel==M_PI/4)
{
ring_b->setPose(osg::Matrix::translate(conf.invert*(0.25), (0.9), conf.invert*(0.25))*(index_b->getPose()));
}
else if (conf.finger_winkel==M_PI/3)
{
ring_b->setPose(osg::Matrix::translate(conf.invert*(0.13), (0.9), conf.invert*(0.17))*(index_b->getPose()));
}
else if (conf.finger_winkel==M_PI/6)
{
ring_b->setPose(osg::Matrix::translate(conf.invert*(0.20), (0.9), conf.invert*(0.20))*(index_b->getPose()));
}
*/

break;

default:
break;
}


objects.push_back(thumb_b);
if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_thumb_b = new IRSensor();
      ir_sensors.push_back(sensor_thumb_b);
	irSensorBank.registerSensor(sensor_thumb_b, objects[2], 
				   osg::Matrix::rotate(-M_PI/2, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1)) * 
	  osg::Matrix::translate(conf.invert*(-0), 0.2, 0.15), conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}
// der minus beim 2-ten PMI zeigt den Sensor von der anderen Seite.
thumb_c = new Capsule(0.2,0.5);
thumb_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

thumb_c ->setPose(osg::Matrix::rotate(conf.invert*2*M_PI, osg::Vec3(1, 0, 0),conf.invert*2*M_PI, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1)) * osg::Matrix::translate(conf.invert*(-0.6+conf.x),(-0.8+conf.y), conf.invert==-1 ?(-0.1+conf.z):(1.1+conf.z)));

break;

case(Is_Draw_under_180_degree):

//thumb_c ->setPose(osg::Matrix::rotate(conf.invert*M_PI/2, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1)) * osg::Matrix::translate(conf.invert*(-0.6+conf.x),(-1.4+conf.y), (0.5+conf.z)));

thumb_c ->setPose(osg::Matrix::translate(conf.invert*(0), (0), (0.47))*(thumb_b->getPose()));


break;

default:
break;
}
objects.push_back(thumb_c);

if(conf.ir_sensor_used)
{
// for the left corner osg::Matrix::translate(conf.invert*(1.5+conf.x),(-0.6+conf.y), (-1.78+conf.z)), 
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_thumb_c = new IRSensor();
      ir_sensors.push_back(sensor_thumb_c);
	irSensorBank.registerSensor(sensor_thumb_c, objects[3], 
				   osg::Matrix::rotate(-M_PI/2, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1)) * 
osg::Matrix::translate(conf.invert*(0), 0.2, 0), conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}

thumb_t = new Capsule(0.2,0.5);
thumb_t -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

thumb_t ->setPose(osg::Matrix::rotate(conf.invert*2*M_PI, osg::Vec3(1, 0, 0),conf.invert*2*M_PI, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1))  * osg::Matrix::translate(conf.invert*(-0.6+conf.x) ,(-0.8+conf.y), conf.invert==-1 ?  (-0.8+conf.z) : (1.8+conf.z)));

break;

case(Is_Draw_under_180_degree):

thumb_t ->setPose(osg::Matrix::rotate(conf.invert*M_PI/2, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1))  * osg::Matrix::translate(conf.invert*(-0.6+conf.x) ,(-2.2+conf.y), (0.5+conf.z)));

thumb_t ->setPose(osg::Matrix::translate(conf.invert*(0), (0), (0.50))*(thumb_c->getPose()));

break;

default:
break;
}

objects.push_back(thumb_t);

if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_thumb_t = new IRSensor();
      ir_sensors.push_back(sensor_thumb_t);
	irSensorBank.registerSensor(sensor_thumb_t, objects[4], 
				   osg::Matrix::rotate(-M_PI/2, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1)) * 
				  osg::Matrix::translate(conf.invert*(0), 0.2, 0), conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}

Primitive* thumb_nail = new Cylinder(0.19,0.0001);
Primitive* fix_thumb_nail= new Transform(objects[4],thumb_nail, osg::Matrix::translate(-0.39, 0,0.205)*osg::Matrix::rotate(M_PI/2+M_PI/58,1,0,0)*osg::Matrix::rotate(M_PI/2,0,1,0));
fix_thumb_nail -> init (odeHandle , 0 , osgHandle);



//-------ball joint between palm and thumb----------------------------------------
Joint* palm_thumb = new BallJoint(palm, thumb_b,
 					      Pos(conf.invert*(-0.4+conf.x),  (-0.75+conf.y), (0.5+conf.z)));//(*) pose wurde an dieser Stelle weggelassen, kann dann aber spaeter dazugenommen werden.
palm_thumb->init(odeHandle, osgHandle, false, 0.2);//Die groesse muss hier neu eingestellt werden, bei Ode nicht einstellbar!!!
joints.push_back(palm_thumb);

//-------hinge joints for thumb-----------------------------------------------------
HingeJoint* thumb_ct, *thumb_bc;

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

 thumb_ct = new HingeJoint(thumb_c, thumb_t,
 					      Pos(conf.invert*(-0.6+conf.x), (-0.8+conf.y), conf.invert==-1 ? (-0.45+conf.z):(1.4+conf.z)), Axis(0, conf.invert*(-1), 0));

break;

case(Is_Draw_under_180_degree):

//thumb_ct = new HingeJoint(thumb_c, thumb_t, (thumb_c->getPosition()+thumb_t->getPosition())/2, Axis(0, 1, 0));


//thumb_ct = new HingeJoint(thumb_c, thumb_t, Pos(conf.invert*(-0.6+conf.x), (-1.8+conf.y), (0.5+conf.z)), Axis(0, 0, conf.invert*(-1)));

thumb_ct = new HingeJoint(thumb_c, thumb_t,
 		(thumb_c->getPosition()+thumb_t->getPosition())/2,  Axis(0, 0, conf.invert*(-1)));

break;

default:
break;
}

thumb_ct->init(odeHandle, osgHandle, true, 0.2 );
thumb_ct->setParam(dParamLoStop, -M_PI/360);
thumb_ct->setParam(dParamHiStop,  M_PI/2);
joints.push_back(thumb_ct);



HingeServo* servo =  new HingeServo(thumb_ct, conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo);


switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

thumb_bc = new HingeJoint(thumb_b, thumb_c,
 					      Pos(conf.invert*(-0.6+conf.x), (-0.8+conf.y), conf.invert==-1 ? (0.25+conf.z):(0.75+conf.z)), Axis(0, conf.invert*(-1), 0));

break;

case(Is_Draw_under_180_degree):

//thumb_bc = new HingeJoint(thumb_b, thumb_c, (thumb_b->getPosition()+thumb_c->getPosition())/2, Axis(0, 1, 0));


//thumb_bc = new HingeJoint(thumb_b, thumb_c,Pos(conf.invert*(-0.55+conf.x), (-1.0+conf.y), (0.5+conf.z)), Axis(0, 0, conf.invert*(-1)));

thumb_bc = new HingeJoint(thumb_b, thumb_c,
 		(thumb_b->getPosition()+thumb_b->getPosition()+thumb_c->getPosition())/3,  Axis(0, 0, conf.invert*(-1)));

break;

default:
break;
}


thumb_bc->init(odeHandle, osgHandle, true, 0.2 );
thumb_bc->setParam(dParamLoStop, -M_PI/360);
thumb_bc->setParam(dParamHiStop,  M_PI/2);
joints.push_back(thumb_bc);

HingeServo* servo_bc =  new HingeServo((HingeJoint*)thumb_bc, conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_bc);


//------------AngularMotor for BallJoint ---------------

 thumb_motor_joint = new AngularMotor3AxisEuler(odeHandle, (BallJoint*)palm_thumb, 
			   axis1, axis3, power);
/*
thumb_motor_joint->init(hand_space, osgHandle, true, 0.2 );
*/

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

if(conf.invert==-1){
((AngularMotor*)thumb_motor_joint)->setParam(dParamHiStop, conf.invert*(-M_PI/2));
((AngularMotor*)thumb_motor_joint)->setParam(dParamLoStop, conf.invert*(M_PI/2-M_PI/20));
}
else{
((AngularMotor*)thumb_motor_joint)->setParam(dParamLoStop, -M_PI/2);
((AngularMotor*)thumb_motor_joint)->setParam(dParamHiStop,  M_PI/2-M_PI/20);
}

break;

case(Is_Draw_under_180_degree):

if(conf.invert==-1){
((AngularMotor*)thumb_motor_joint)->setParam(dParamHiStop, conf.invert*(-M_PI/360));
((AngularMotor*)thumb_motor_joint)->setParam(dParamLoStop, conf.invert*(M_PI-M_PI/10));
}
else{
((AngularMotor*)thumb_motor_joint)->setParam(dParamLoStop, -M_PI/360);
((AngularMotor*)thumb_motor_joint)->setParam(dParamHiStop,  M_PI-M_PI/10);
}

break;

default:
break;
}

((AngularMotor*)thumb_motor_joint)->setParam(dParamLoStop2, -M_PI/360);
((AngularMotor*)thumb_motor_joint)->setParam(dParamHiStop2,  M_PI/360);
((AngularMotor*)thumb_motor_joint)->setParam(dParamLoStop3, -M_PI/360);
((AngularMotor*)thumb_motor_joint)->setParam(dParamHiStop3,  M_PI/360);


frictionmotors.push_back(thumb_motor_joint);
//frictionmotors.push_back(new AngularMotor3AxisEuler(odeHandle, (BallJoint*)palm_thumb, axis1, axis3, conf.power));

//-----------index finger--------------------------------------------------

Primitive *index_b, *index_c, *index_t;

index_b = new Capsule(0.2,0.6);
index_b -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
index_b ->setPose(osg::Matrix::rotate(2*M_PI, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.6+conf.x), (-0.6+conf.y), conf.invert == -1 ? (conf.invert*-0.1+conf.z) : (0.9+conf.z)));
break;

case(Is_Draw_under_180_degree):
//palm->setPose(osg::Matrix::translate(conf.invert*(0+conf.x),(0.05+conf.y),(0.5+conf.z)));
if (conf.finger_winkel==M_PI/2)
{ 
index_b ->setPose(osg::Matrix::rotate(conf.finger_winkel, 0, 1, 0) *osg::Matrix::translate(conf.invert*(1.0), (-0.6), (-0.05)) *(palm->getPose()));
}
else if (conf.finger_winkel==2*M_PI)
{
index_b ->setPose(osg::Matrix::rotate(conf.finger_winkel, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.65), (-0.6), (conf.invert*0.35))*(palm->getPose()));

}
else if (conf.finger_winkel==M_PI/4)
{
index_b ->setPose(osg::Matrix::rotate(conf.finger_winkel, 0, 1, 0) * osg::Matrix::translate(conf.invert==-1 ? conf.invert*(0.9): conf.invert*(0.8), (-0.6), conf.invert==-1 ? conf.invert*(0.3) : (0.2))*(palm->getPose()));
}
else if (conf.finger_winkel==M_PI/3)
{ 
index_b ->setPose(osg::Matrix::rotate(conf.finger_winkel, 0, 1, 0) *osg::Matrix::translate(conf.invert*(1.0), (-0.6), conf.invert*(0.2)) *(palm->getPose()));
}
else if (conf.finger_winkel==M_PI/6)
{ 
index_b ->setPose(osg::Matrix::rotate(conf.finger_winkel, 0, 1, 0) *osg::Matrix::translate(conf.invert*(0.8), (-0.6), conf.invert==-1 ?conf.invert*(0.35):(0.3)) *(palm->getPose()));
}

//(0.9), (-0.6), (-0.3)

//index_b ->setPose(osg::Matrix::rotate(conf.finger_winkel, 0, 1, 0) * osg::Matrix::translate(Pos(index_b->getPosition()-(index_c->getPosition()-index_b->getPosition())/1.8)));

//index_b ->setPose(osg::Matrix::rotate(conf.finger_winkel, 0, 1, 0) * osg::Matrix::translate((-forearm->getPosition()+palm->getPosition())/3.6)*(palm->getPose())*osg::Matrix::translate(0,-0.6,0));

//index_b ->setPose(osg::Matrix::rotate(conf.finger_winkel, 0, 1, 0) * osg::Matrix::translate(sin(conf.finger_winkel)+0.4, (-0.6), cos(conf.finger_winkel)-0.6)*(palm->getPose()));

//index_b ->setPose(osg::Matrix::rotate(conf.finger_winkel, 0, 1, 0) * osg::Matrix::translate(conf.invert*sin(conf.finger_winkel),(-0.6),cos(conf.finger_winkel))*(palm->getPose()));

break;

default:
break;
}
objects.push_back(index_b);



if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_index_b = new IRSensor();
      ir_sensors.push_back(sensor_index_b);
	irSensorBank.registerSensor(sensor_index_b, objects[5], 
				  osg::Matrix::rotate(conf.invert*-M_PI/2, 0, 1, 0)  * 
  				  osg::Matrix::translate(conf.invert*(-0.2), 0, 0), 
 conf.irRange, conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}



index_c = new Capsule(0.2,0.6);
index_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

index_c ->setPose(osg::Matrix::rotate(2*M_PI, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.6+conf.x), (-0.6+conf.y), conf.invert == -1 ? (conf.invert*0.7+conf.z) :(1.65+conf.z)));

break;

case(Is_Draw_under_180_degree):

  
index_c ->setPose(osg::Matrix::translate((0), (0), conf.invert*(0.59))*(index_b->getPose()));

//osg::Matrix::translate(Pos(index_b->getPosition()))*osg::Matrix::rotate(index_b->getPosition())
//*osg::Matrix::translate(Pos(index_b->getRotation())

break;

default:
break;
}

objects.push_back(index_c);

if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_index_c = new IRSensor();
      ir_sensors.push_back(sensor_index_c);
	irSensorBank.registerSensor(sensor_index_c, objects[6], 
				   osg::Matrix::rotate(conf.invert*-M_PI/2, 0, 1, 0)  * 
				  osg::Matrix::translate(conf.invert*(-0.2), 0, 0), conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}

index_t = new Capsule(0.2,0.5);
index_t -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

index_t ->setPose(osg::Matrix::rotate(2*M_PI, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.6+conf.x),(-0.6+conf.y), conf.invert == -1 ? (conf.invert*1.5+conf.z) :(2.45+conf.z)));
break;

case(Is_Draw_under_180_degree):

//index_t ->setPose(osg::Matrix::translate(conf.invert*(0), (0), (1.41))*(index_b->getPose()));

index_t ->setPose(osg::Matrix::translate((0), (0), conf.invert*(0.59))*(index_c->getPose()));

//index_t ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(2.35+conf.x),(-0.6+conf.y), (0.5+conf.z)));
break;

default:
break;
}

objects.push_back(index_t);

if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_index_t = new IRSensor();
      ir_sensors.push_back(sensor_index_t);
	irSensorBank.registerSensor(sensor_index_t, objects[7], 
				   osg::Matrix::rotate(conf.invert*-M_PI/2, 0, 1, 0)  * 
				  osg::Matrix::translate(conf.invert*(-0.2), 0, 0), conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));

}

Primitive* index_nail = new Cylinder(0.19,0.0001);
Primitive* fix_index_nail= new Transform(objects[7],index_nail, osg::Matrix::translate(-0.35, 0,0.205)*osg::Matrix::rotate(conf.invert*M_PI/2,0,1,0));
fix_index_nail -> init (odeHandle , 0 , osgHandle);


//----------------index finger joints----------------------------------------------
Joint *palm_index, *index_bc, *index_ct;

//palm_index = new HingeJoint(palm, index_b,Pos(conf.invert*(0.6+conf.x), (-0.6+conf.y), (0.5+conf.z)), Axis(0, 1, 0));

palm_index = new HingeJoint(palm, index_b, index_b->getPosition()-(index_c->getPosition()-index_b->getPosition())/1.8, Axis(0, 1, 0));


palm_index ->init(odeHandle, osgHandle, true, 0.2 );

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
palm_index ->setParam(dParamLoStop,  -M_PI/2);
palm_index ->setParam(dParamHiStop,  M_PI/360);
break;

case(Is_Draw_under_180_degree):
//palm_index ->setParam(dParamLoStop,  -M_PI/2);
//palm_index ->setParam(dParamHiStop,  M_PI/360);
if(conf.finger_winkel==M_PI/2)
{
palm_index ->setParam(dParamLoStop,  -M_PI/360);
palm_index ->setParam(dParamHiStop,  M_PI/2);
}
else if (conf.finger_winkel==2*M_PI)
{
palm_index ->setParam(dParamLoStop,  -M_PI/2);
palm_index ->setParam(dParamHiStop,  M_PI/360);
}
else if (conf.finger_winkel==M_PI/4)
{
palm_index ->setParam(dParamLoStop,  -M_PI/4);
palm_index ->setParam(dParamHiStop,  M_PI/4);
}
else if (conf.finger_winkel==M_PI/3)
{
palm_index ->setParam(dParamLoStop,  -M_PI/6);
palm_index ->setParam(dParamHiStop,  M_PI/3);
}
else if (conf.finger_winkel==M_PI/6)
{
palm_index ->setParam(dParamLoStop,  -M_PI/3);
palm_index ->setParam(dParamHiStop,  M_PI/6);
}

break;

default:
break;
}


palm_index ->setParam (dParamBounce, 0.9 );
joints.push_back(palm_index);

HingeServo* servo_palm_index =  new HingeServo((HingeJoint*)palm_index, conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power); 
    servos.push_back(servo_palm_index);

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

index_bc = new HingeJoint(index_b, index_c,
 					      Pos(conf.invert*(0.6+conf.x),(-0.6+conf.y),conf.invert == -1 ? (conf.invert*0.3+conf.z) :(1.23+conf.z)), Axis(0, 1, 0));

break;

case(Is_Draw_under_180_degree):

index_bc = new HingeJoint(index_b, index_c,
 		(index_b->getPosition()+index_c->getPosition())/2, Axis(0, 1, 0));
// * osg::Matrix::translate(Pos(index_b->getPosition()))
//Pos(conf.invert*(1.35+conf.x),(-0.6+conf.y),(0.5+conf.z))
break;

default:
break;
}


index_bc ->init(odeHandle, osgHandle, true, 0.2 );
index_bc ->setParam(dParamHiStop,  M_PI/2);
index_bc ->setParam(dParamLoStop, -M_PI/360);
index_bc ->setParam (dParamBounce, 0.9 );
joints.push_back(index_bc);

HingeServo* servo_index_bc =  new HingeServo((HingeJoint*)index_bc, conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_index_bc);


switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

//index_ct = new HingeJoint(index_c, index_t,
// 		((index_c->getPosition()+index_t->getPosition())/2), Axis(0, 1, 0));
// * osg::Matrix::translate(Pos(index_b->getPosition()))

index_ct = new HingeJoint(index_c, index_t,Pos(conf.invert*(0.6+conf.x),(-0.6+conf.y),conf.invert == -1 ? (conf.invert*1.1+conf.z) :(2.05+conf.z)), Axis(0, 1, 0));

break;

case(Is_Draw_under_180_degree):

//index_ct = new HingeJoint(index_c, index_t, Pos(conf.invert*(2.1+conf.x), (-0.6+conf.y),(0.5+conf.z)), Axis(0, 1, 0));

index_ct = new HingeJoint(index_c, index_t,
 		((index_c->getPosition()+index_t->getPosition())/2), Axis(0, 1, 0)); 
//* osg::Matrix::translate(Pos(index_c->getPosition())));

break;

default:
break;
}




index_ct ->init(odeHandle, osgHandle, true, 0.2 );
index_ct ->setParam(dParamHiStop,  M_PI/2);
index_ct ->setParam(dParamLoStop, -M_PI/360);
index_ct ->setParam (dParamBounce, 0.9 );
joints.push_back(index_ct);

HingeServo* servo_index_ct =  new HingeServo((HingeJoint*)index_ct, conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_index_ct);
 

//---------middle finger----------------------------------------------------------  

Primitive *middle_b,*middle_c,*middle_t;


middle_b = new Capsule(0.2,0.6);
middle_b -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

middle_b->setPose(osg::Matrix::rotate(2*M_PI, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.95+conf.x), (-0.15+conf.y), conf.invert == -1 ? (conf.invert*0+conf.z) :(0.95+conf.z)));


break;

case(Is_Draw_under_180_degree):

//middle_b->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(1.35+conf.x), (-0.15+conf.y), (0.5+conf.z)));
if (conf.finger_winkel==M_PI/2)
{
middle_b->setPose(osg::Matrix::translate(conf.invert*(0), (0.45), conf.invert*(0.3))*(index_b->getPose()));
}
else if (conf.finger_winkel==2*M_PI)
{
middle_b->setPose(osg::Matrix::translate(conf.invert*(0.3), (0.45), conf.invert*(0))*(index_b->getPose()));
}
else if (conf.finger_winkel==M_PI/4)
{
middle_b->setPose(osg::Matrix::translate(conf.invert*(0.25), (0.45), conf.invert*(0.25))*(index_b->getPose()));
}
else if (conf.finger_winkel==M_PI/3)
{
middle_b->setPose(osg::Matrix::translate(conf.invert*(0.13), (0.45), conf.invert*(0.17))*(index_b->getPose()));
}
else if (conf.finger_winkel==M_PI/6)
{
middle_b->setPose(osg::Matrix::translate(conf.invert*(0.20), (0.45), conf.invert*(0.20))*(index_b->getPose()));
}

//middle_b ->setPose(osg::Matrix::rotate(conf.finger_winkel, 0, 1, 0) * osg::Matrix::translate(sin(conf.finger_winkel),(-0.15),cos(conf.finger_winkel))*(palm->getPose()));

//middle_b->setPose(osg::Matrix::translate(conf.invert*(0), (0.45), (0))*(palm->getPose()));

break;

default:
break;
}

objects.push_back(middle_b);
if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_middle_b = new IRSensor();
      ir_sensors.push_back(sensor_middle_b);
	irSensorBank.registerSensor(sensor_middle_b, objects[8], 
				   osg::Matrix::rotate(conf.invert*-M_PI/2, 0, 1, 0)  * 
				  				  osg::Matrix::translate(conf.invert*(-0.2), 0, 0),  conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}

middle_c = new Capsule(0.2,0.7);
middle_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
middle_c ->setPose(osg::Matrix::rotate(2*M_PI, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.95+conf.x), (-0.15+conf.y), conf.invert == -1 ? (conf.invert*0.85+conf.z) :(1.8+conf.z)));

break;

case(Is_Draw_under_180_degree):

//middle_c ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(2.21+conf.x), (-0.15+conf.y), (0.5+conf.z)));

middle_c->setPose(osg::Matrix::translate(conf.invert*(0), (0), conf.invert*(0.69))*(middle_b->getPose()));

break;

default:
break;
}

objects.push_back(middle_c);

if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_middle_c = new IRSensor();
      ir_sensors.push_back(sensor_middle_c);
	irSensorBank.registerSensor(sensor_middle_c, objects[9], 
				   osg::Matrix::rotate(conf.invert*-M_PI/2, 0, 1, 0)  * 
				  						  osg::Matrix::translate(conf.invert*(-0.2), 0, 0), conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}

middle_t = new Capsule(0.2,0.7);
middle_t -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

middle_t ->setPose(osg::Matrix::rotate(2*M_PI, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.95+conf.x),(-0.15+conf.y), conf.invert == -1 ? (conf.invert*1.65+conf.z) :(2.6+conf.z)));


break;

case(Is_Draw_under_180_degree):

middle_t->setPose(osg::Matrix::translate(conf.invert*(0), (0), conf.invert*(0.69))*(middle_c->getPose()));

//middle_t ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(2.9+conf.x),(-0.15+conf.y), (0.5+conf.z)));

break;

default:
break;
}

objects.push_back(middle_t);

if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_middle_t = new IRSensor();
      ir_sensors.push_back(sensor_middle_t);
	irSensorBank.registerSensor(sensor_middle_t, objects[10], 
				   osg::Matrix::rotate(conf.invert*-M_PI/2, 0, 1, 0)  * 
								  osg::Matrix::translate(conf.invert*(-0.2), 0, 0),  conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}

Primitive* middle_nail = new Cylinder(0.19,0.0001);
Primitive* fix_middle_nail= new Transform(objects[10],middle_nail, osg::Matrix::translate(-0.4, 0,0.205)*osg::Matrix::rotate(conf.invert*M_PI/2,0,1,0));
fix_middle_nail -> init (odeHandle , 0 , osgHandle);

//-------------------------middle finger joints-------------------------------------
Joint *palm_middle, *middle_bc, *middle_ct;

//palm_middle = new HingeJoint(palm, middle_b, (palm->getPosition()+middle_b->getPosition())/2, Axis(0, 1, 0));

palm_middle = new HingeJoint(palm, middle_b,Pos(conf.invert*(0.9+conf.x), (-0.15+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
palm_middle ->init(odeHandle, osgHandle, true, 0.2 );

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
palm_middle->setParam(dParamLoStop, -M_PI/2);
palm_middle->setParam(dParamHiStop,  M_PI/360);
break;

case(Is_Draw_under_180_degree):
if (conf.finger_winkel==M_PI/2)
{
palm_middle->setParam(dParamLoStop, -M_PI/360);
palm_middle->setParam(dParamHiStop,  M_PI/2);
}
else if (conf.finger_winkel==2*M_PI)
{
palm_middle ->setParam(dParamLoStop,  -M_PI/2);
palm_middle ->setParam(dParamHiStop,  M_PI/360);
}
else if (conf.finger_winkel==M_PI/4)
{
palm_middle ->setParam(dParamLoStop,  -M_PI/4);
palm_middle ->setParam(dParamHiStop,  M_PI/4);
}
else if (conf.finger_winkel==M_PI/3)
{
palm_middle ->setParam(dParamLoStop,  -M_PI/6);
palm_middle ->setParam(dParamHiStop,  M_PI/3);
}
else if (conf.finger_winkel==M_PI/6)
{
palm_middle ->setParam(dParamLoStop,  -M_PI/3);
palm_middle ->setParam(dParamHiStop,  M_PI/6);
}


break;

default:
break;
}


joints.push_back(palm_middle);

HingeServo* servo_palm_middle =  new HingeServo((HingeJoint*)palm_middle, conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_palm_middle);

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
middle_bc = new HingeJoint(middle_b, middle_c,
 					      Pos(conf.invert*(0.95+conf.x), (-0.15+conf.y), conf.invert == -1 ? (conf.invert*0.45+conf.z) :(1.38+conf.z)), Axis(0, 1, 0));

break;

case(Is_Draw_under_180_degree):

middle_bc = new HingeJoint(middle_b, middle_c,
 		(middle_b->getPosition()+middle_c->getPosition())/2, Axis(0, 1, 0));

//middle_bc = new HingeJoint(middle_b, middle_c, Pos(conf.invert*(1.87+conf.x), (-0.15+conf.y), (0.5+conf.z)), Axis(0, 1, 0));

break;

default:
break;
}

middle_bc ->init(odeHandle, osgHandle, true, 0.2 );
middle_bc->setParam(dParamHiStop,  M_PI/2);
middle_bc->setParam(dParamLoStop, -M_PI/360);
joints.push_back(middle_bc);

HingeServo* servo_middle_bc =  new HingeServo((HingeJoint*)middle_bc, conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_middle_bc);

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
middle_ct = new HingeJoint(middle_c, middle_t,
    					  Pos(conf.invert*(0.95+conf.x), (-0.15+conf.y), conf.invert == -1 ? (conf.invert*1.25+conf.z) :(2.2+conf.z)), Axis(0, 1, 0));

break;

case(Is_Draw_under_180_degree):

//middle_ct = new HingeJoint(middle_c, middle_t, Pos(conf.invert*(2.6+conf.x), (-0.15+conf.y), (0.5+conf.z)), Axis(0, 1, 0));

middle_ct = new HingeJoint(middle_c, middle_t,
 		(middle_c->getPosition()+middle_t->getPosition())/2, Axis(0, 1, 0));

break;

default:
break;
}

middle_ct ->init(odeHandle, osgHandle, true, 0.2 );
middle_ct->setParam(dParamHiStop,  M_PI/2);
middle_ct->setParam(dParamLoStop, -M_PI/360);
joints.push_back(middle_ct);

HingeServo* servo_middle_ct =  new HingeServo((HingeJoint*)middle_ct, conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_middle_ct);

//---------------------------ring finger------------------------------------------- 
Primitive *ring_b, *ring_c, *ring_t;

ring_b = new Capsule(0.2,0.6);
ring_b-> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
ring_b ->setPose(osg::Matrix::rotate(2*M_PI, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.9+conf.x), (0.3+conf.y), conf.invert == -1 ? (conf.invert*-0.1+conf.z) : (0.9+conf.z)));
break;

case(Is_Draw_under_180_degree):

//ring_b->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(1.3+conf.x), (0.3+conf.y), (0.5+conf.z)));

if (conf.finger_winkel==M_PI/2)
{
ring_b->setPose(osg::Matrix::translate(conf.invert*(0), (0.9), conf.invert*(0.3))*(index_b->getPose()));
}
else if (conf.finger_winkel==2*M_PI)
{
ring_b->setPose(osg::Matrix::translate(conf.invert*(0.3), (0.9), conf.invert*(0))*(index_b->getPose()));
}
else if (conf.finger_winkel==M_PI/4)
{
ring_b->setPose(osg::Matrix::translate(conf.invert*(0.25), (0.9), conf.invert*(0.25))*(index_b->getPose()));
}
else if (conf.finger_winkel==M_PI/3)
{
ring_b->setPose(osg::Matrix::translate(conf.invert*(0.13), (0.9), conf.invert*(0.17))*(index_b->getPose()));
}
else if (conf.finger_winkel==M_PI/6)
{
ring_b->setPose(osg::Matrix::translate(conf.invert*(0.20), (0.9), conf.invert*(0.20))*(index_b->getPose()));
}


//ring_b ->setPose(osg::Matrix::rotate(conf.finger_winkel, 0, 1, 0) * osg::Matrix::translate(sin(conf.finger_winkel),(0.3),cos(conf.finger_winkel))*(palm->getPose()));

break;

default:
break;
}

objects.push_back(ring_b);

if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_ring_b = new IRSensor();
      ir_sensors.push_back(sensor_ring_b);
	irSensorBank.registerSensor(sensor_ring_b, objects[11], 
				   osg::Matrix::rotate(conf.invert*-M_PI/2, 0, 1, 0)  * 
				 				  osg::Matrix::translate(conf.invert*(-0.2), 0, 0),  conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}

ring_c = new Capsule(0.2,0.6);
ring_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
ring_c ->setPose(osg::Matrix::rotate(2*M_PI, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.9+conf.x), (0.3+conf.y), conf.invert == -1 ? (conf.invert*0.7+conf.z) :(1.65+conf.z)));
break;

case(Is_Draw_under_180_degree):


//ring_c ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(2.0+conf.x), (0.3+conf.y), (0.5+conf.z)));

ring_c->setPose(osg::Matrix::translate(conf.invert*(0), (0), conf.invert*(0.59))*(ring_b->getPose()));

break;

default:
break;
}

objects.push_back(ring_c);

if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_ring_c = new IRSensor();
      ir_sensors.push_back(sensor_ring_c);
	irSensorBank.registerSensor(sensor_ring_c, objects[12], 
				   osg::Matrix::rotate(conf.invert*-M_PI/2, 0, 1, 0)  * 
				 				  osg::Matrix::translate(conf.invert*(-0.2), 0, 0),  conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}

ring_t = new Capsule(0.2,0.5);
ring_t -> init ( odeHandle, 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

ring_t ->setPose(osg::Matrix::rotate(2*M_PI, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.9+conf.x), (0.3+conf.y), conf.invert == -1 ? (conf.invert*1.5+conf.z) :(2.45+conf.z)));

break;

case(Is_Draw_under_180_degree):

ring_t->setPose(osg::Matrix::translate(conf.invert*(0), (0), conf.invert*(0.59))*(ring_c->getPose()));

//ring_t ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(2.75+conf.x),(0.3+conf.y), (0.5+conf.z)));
break;

default:
break;
}

objects.push_back(ring_t);

if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_ring_t = new IRSensor();
      ir_sensors.push_back(sensor_ring_t);
	irSensorBank.registerSensor(sensor_ring_t, objects[13], 
				   osg::Matrix::rotate(conf.invert*-M_PI/2, 0, 1, 0)  * 
								  osg::Matrix::translate(conf.invert*(-0.2), 0, 0),  conf.irRange,
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}

Primitive* ring_nail = new Cylinder(0.19,0.0001);
Primitive* fix_ring_nail= new Transform(objects[13],ring_nail, osg::Matrix::translate(-0.37, 0,0.205)*osg::Matrix::rotate(conf.invert*M_PI/2,0,1,0));
fix_ring_nail -> init (odeHandle , 0 , osgHandle);



//-----------ring finger joints----------------------------------------------------
Joint *palm_ring, *ring_bc, *ring_ct;


palm_ring = new HingeJoint(palm, ring_b,
 					      Pos(conf.invert*(0.9+conf.x), (0.3+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
palm_ring->init(odeHandle, osgHandle, false, 0.2 );

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
palm_ring ->setParam(dParamLoStop,  -M_PI/2);
palm_ring ->setParam(dParamHiStop,   M_PI/360);

break;

case(Is_Draw_under_180_degree):
if (conf.finger_winkel==M_PI/2)
{
palm_ring ->setParam(dParamLoStop,  -M_PI/360);
palm_ring->setParam(dParamHiStop,  M_PI/2);
}
else if (conf.finger_winkel==2*M_PI)
{
palm_ring ->setParam(dParamLoStop,  -M_PI/2);
palm_ring ->setParam(dParamHiStop,  M_PI/360);
}
else if (conf.finger_winkel==M_PI/4)
{
palm_ring ->setParam(dParamLoStop,  -M_PI/4);
palm_ring ->setParam(dParamHiStop,  M_PI/4);
}
else if (conf.finger_winkel==M_PI/3)
{
palm_ring ->setParam(dParamLoStop,  -M_PI/6);
palm_ring ->setParam(dParamHiStop,  M_PI/3);
}
else if (conf.finger_winkel==M_PI/6)
{
palm_ring ->setParam(dParamLoStop,  -M_PI/3);
palm_ring ->setParam(dParamHiStop,  M_PI/6);
}

break;

default:
break;
}

joints.push_back(palm_ring);

HingeServo* servo_palm_ring =  new HingeServo((HingeJoint*)palm_ring, conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_palm_ring);


switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

//ring_bc = new HingeJoint(ring_b, ring_c,Pos(conf.invert*(0.9+conf.x), (0.3+conf.y), conf.invert == -1 ? (conf.invert*0.3+conf.z) :(1.23+conf.z)), Axis(0, 1, 0));

ring_bc = new HingeJoint(ring_b, ring_c, (ring_b->getPosition()+ring_c->getPosition())/2, Axis(0, 1, 0));


break;

case(Is_Draw_under_180_degree):

//ring_bc = new HingeJoint(ring_b, ring_c,Pos(conf.invert*(1.7+conf.x), (0.3+conf.y), (0.5+conf.z)), Axis(0, 1, 0));

ring_bc = new HingeJoint(ring_b, ring_c, (ring_b->getPosition()+ring_c->getPosition())/2, Axis(0, 1, 0));


break;

default:
break;
}

ring_bc ->init(odeHandle, osgHandle, true, 0.2 );
ring_bc->setParam(dParamHiStop,  M_PI/2);
ring_bc->setParam(dParamLoStop, -M_PI/360);
joints.push_back(ring_bc);

HingeServo* servo_ring_bc =  new HingeServo((HingeJoint*)ring_bc, conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_ring_bc);

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

ring_ct = new HingeJoint(ring_c, ring_t,
 					      Pos(conf.invert*(0.9+conf.x), (0.3+conf.y), conf.invert == -1 ? (conf.invert*1.1+conf.z) :(2.05+conf.z)), Axis(0, 1, 0));
break;

case(Is_Draw_under_180_degree):

//ring_ct = new HingeJoint(ring_c, ring_t, Pos(conf.invert*(2.5+conf.x), (0.3+conf.y), (0.5+conf.z)), Axis(0, 1, 0));

ring_ct = new HingeJoint(ring_c, ring_t, (ring_c->getPosition()+ring_t->getPosition())/2, Axis(0, 1, 0));

break;

default:
break;
}

ring_ct ->init(odeHandle, osgHandle, true, 0.2 );
ring_ct->setParam(dParamHiStop,  M_PI/2);
ring_ct->setParam(dParamLoStop, -M_PI/360);
joints.push_back(ring_ct);

HingeServo* servo_ring_ct =  new HingeServo((HingeJoint*)ring_ct, conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_ring_ct);


//-----------little finger--------------------------------------------------------

Primitive *little_b, *little_c, *little_t;

little_b = new Capsule(0.2,0.6);
little_b -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

little_b ->setPose(osg::Matrix::rotate(2*M_PI, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.6+conf.x), (0.75+conf.y), conf.invert == -1 ? (conf.invert*-0.1+conf.z) : (0.9+conf.z)));

break;

case(Is_Draw_under_180_degree):

little_b->setPose(osg::Matrix::translate(conf.invert*(0), (1.35), (0))*(index_b->getPose()));

//little_b ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.9+conf.x), (0.75+conf.y), (0.5+conf.z)));

break;

default:
break;
}

objects.push_back(little_b);

if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_little_b = new IRSensor();
      ir_sensors.push_back(sensor_little_b);
	irSensorBank.registerSensor(sensor_little_b, objects[14], 
				   osg::Matrix::rotate(conf.invert*-M_PI/2, 0, 1, 0)  * 
				  				  osg::Matrix::translate(conf.invert*(-0.2), 0, 0),   conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}

little_c = new Capsule(0.2,0.6);
little_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

little_c ->setPose(osg::Matrix::rotate(2*M_PI, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.6+conf.x), (0.75+conf.y), conf.invert == -1 ? (conf.invert*0.7+conf.z) :(1.65+conf.z)));

break;

case(Is_Draw_under_180_degree):

little_c->setPose(osg::Matrix::translate(conf.invert*(0), (0), conf.invert*(0.59))*(little_b->getPose()));

//little_c ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(1.5+conf.x), (0.75+conf.y), (0.5+conf.z)));

break;

default:
break;
}

objects.push_back(little_c);

if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_little_c = new IRSensor();
      ir_sensors.push_back(sensor_little_c);
	irSensorBank.registerSensor(sensor_little_c, objects[15], 
				   osg::Matrix::rotate(conf.invert*-M_PI/2, 0, 1, 0)  * 
				  				  osg::Matrix::translate(conf.invert*(-0.2), 0, 0),   conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}

little_t = new Capsule(0.2,0.6);
little_t -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

little_t ->setPose(osg::Matrix::rotate(2*M_PI, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.6+conf.x) , (0.75+conf.y), conf.invert == -1 ? (conf.invert*1.47+conf.z) :(2.45+conf.z)));

break;

case(Is_Draw_under_180_degree):

little_t->setPose(osg::Matrix::translate(conf.invert*(0), (0), conf.invert*(0.59))*(little_c->getPose()));


//little_t ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(2.05+conf.x) , (0.75+conf.y), (0.5+conf.z)));

break;

default:
break;
}

objects.push_back(little_t);

if(conf.ir_sensor_used)
{
irSensorBank.init(odeHandle, osgHandle);
	IRSensor* sensor_little_t = new IRSensor();
      ir_sensors.push_back(sensor_little_t);
	irSensorBank.registerSensor(sensor_little_t, objects[16], 
				   osg::Matrix::rotate(conf.invert*-M_PI/2, 0, 1, 0)  * 
				 				  osg::Matrix::translate(conf.invert*(-0.2), 0, 0),   conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));
}

Primitive* little_nail = new Cylinder(0.19,0.0001);
Primitive* fix_little_nail= new Transform(objects[16],little_nail, osg::Matrix::translate(-0.39, 0,0.205)*osg::Matrix::rotate(conf.invert*M_PI/2,0,1,0));
fix_little_nail -> init (odeHandle , 0 , osgHandle);

//-----------------little finger joints-----------------------------------------

Joint *palm_little, *little_bc, *little_ct;

palm_little = new HingeJoint(palm, little_b,
 					      Pos(conf.invert*(0.6+conf.x), (0.75+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
palm_little ->init(odeHandle, osgHandle, false, 0.2 );
switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
palm_little ->setParam(dParamLoStop,  -M_PI/2);
palm_little ->setParam(dParamHiStop,  M_PI/360);

break;

case(Is_Draw_under_180_degree):
if (conf.finger_winkel==M_PI/2)
{
palm_little ->setParam(dParamLoStop, -M_PI/360);
palm_little ->setParam(dParamHiStop,  M_PI/2);
}
else if (conf.finger_winkel==2*M_PI)
{
palm_little ->setParam(dParamLoStop,  -M_PI/2);
palm_little ->setParam(dParamHiStop,  M_PI/360);
}
else if (conf.finger_winkel==M_PI/4)
{
palm_little ->setParam(dParamLoStop,  -M_PI/4);
palm_little ->setParam(dParamHiStop,  M_PI/4);
}
else if (conf.finger_winkel==M_PI/3)
{
palm_little ->setParam(dParamLoStop,  -M_PI/6);
palm_little ->setParam(dParamHiStop,  M_PI/3);
}
else if (conf.finger_winkel==M_PI/6)
{
palm_little  ->setParam(dParamLoStop,  -M_PI/3);
palm_little  ->setParam(dParamHiStop,  M_PI/6);
}

break;

default:
break;
}


joints.push_back(palm_little);

HingeServo* servo_palm_little =  new HingeServo((HingeJoint*)palm_little, conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_palm_little);

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
little_bc = new HingeJoint(little_b, little_c,
 					      Pos(conf.invert*(0.6+conf.x), (0.75+conf.y), conf.invert == -1 ? (conf.invert*0.3+conf.z) :(1.23+conf.z)), Axis(0, 1, 0));

break;

case(Is_Draw_under_180_degree):

little_bc = new HingeJoint(little_b, little_c, (little_b->getPosition()+little_c->getPosition())/2, Axis(0, 1, 0));

//little_bc = new HingeJoint(little_b, little_c,                                    Pos(conf.invert*(1.198+conf.x), (0.75+conf.y), (0.5+conf.z)), Axis(0, 1, 0));

break;

default:
break;
}

little_bc ->init(odeHandle, osgHandle, true, 0.2 );
little_bc ->setParam(dParamHiStop,  M_PI/2);
little_bc ->setParam(dParamLoStop, -M_PI/360);
joints.push_back(little_bc);

HingeServo* servo_little_bc =  new HingeServo((HingeJoint*)little_bc, conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_little_bc);


switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
little_ct = new HingeJoint(little_c, little_t,
 					      Pos(conf.invert*(0.6+conf.x), (0.75+conf.y), conf.invert == -1 ? (conf.invert*1.1+conf.z) :(2.05+conf.z)), Axis(0, 1, 0));

break;

case(Is_Draw_under_180_degree):

little_ct = new HingeJoint(little_c, little_t, (little_c->getPosition()+little_t->getPosition())/2, Axis(0, 1, 0));

//little_ct = new HingeJoint(little_c, little_t,    						Pos(conf.invert*(1.8+conf.x), (0.75+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
//little_ct = new HingeJoint(little_c, little_t,(little_c->getPosition()+little_t->getPosition())/4, Axis(0, 1, 0));

break;

default:
break;
}

little_ct ->init(odeHandle, osgHandle, true, 0.2 );
little_ct ->setParam(dParamHiStop,  M_PI/2);
little_ct ->setParam(dParamLoStop, -M_PI/360);
joints.push_back(little_ct);

HingeServo* servo_little_ct =  new HingeServo((HingeJoint*)little_ct, 
conf.jointLimit1, conf.jointLimit2, conf.servo_motor_Power);
    servos.push_back(servo_little_ct);


//-joints connecting index, middle, ring and little finger so that they move together
/*
Joint *index_middle, *middle_ring, *ring_little;


index_middle = new HingeJoint(index_b, middle_b,
 					      Pos(1.4+conf.x, -0.375+conf.y, 0.5+conf.z), Axis(1, 0, 0));
index_middle ->init(odeHandle, osgHandle, true, 0.2 );
index_middle ->setParam(dParamLoStop,  M_PI/360);
index_middle ->setParam(dParamHiStop, -M_PI/360);
joints.push_back(index_middle);

middle_ring = new HingeJoint(middle_b, ring_b,
 					      Pos(1.4+conf.x, 0.075+conf.y, 0.5+conf.z), Axis(1, 0, 0));
middle_ring ->init(odeHandle, osgHandle, true, 0.2 );
middle_ring ->setParam(dParamLoStop,  M_PI/360);
middle_ring ->setParam(dParamHiStop, -M_PI/360);
joints.push_back(middle_ring);

ring_little = new HingeJoint(ring_b, little_b,
 					      Pos(1.4+conf.x, 0.525+conf.y, 0.5+conf.z), Axis(1, 0, 0));
ring_little ->init(odeHandle, osgHandle, true, 0.2 );
ring_little ->setParam(dParamLoStop,  M_PI/360);
ring_little ->setParam(dParamHiStop, -M_PI/360);

joints.push_back(ring_little);
*/



//---------joint connecting forearm with simulation environment----------------------

fix_forearm_joint = new HingeJoint(forearm, 0,
 					      Pos(conf.invert*(-6.2+conf.x), (0+conf.y), (0.5+conf.z)), Axis(1, 0, 0));
fix_forearm_joint->init(odeHandle, osgHandle, true, 0.2 );
fix_forearm_joint->setParam(dParamLoStop, -M_PI/360);
fix_forearm_joint->setParam(dParamHiStop,  M_PI/360);
joints.push_back(fix_forearm_joint);

/*
Primitive* palm = new Cylinder(1.0,0.3);
 
  palm-> init (odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);

  palm->setPose(osg::Matrix::translate(0+conf.x,0.05+conf.y,0.5+conf.z));
//  palm->getOSGPrimitive()->setTexture("Images/ground.rgb"); 

objects.push_back(palm);
*/
//---nails for the fingers-----------------------------------------------------------

/*
//=======thumb-finger-nail=======================================================

Primitive* thumb_nail;
Joint* fix_thumb_nail;

thumb_nail = new Cylinder(0.19,0.0001);
thumb_nail -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

thumb_nail ->setPose(osg::Matrix::rotate(conf.invert==-1 ? conf.invert*(-M_PI)+M_PI/58:-M_PI+M_PI/58, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1))*osg::Matrix::translate(conf.invert*(-0.8+conf.x), -0.8+conf.y, conf.invert==-1 ? -1.12+conf.z : 2.1+conf.z));

fix_thumb_nail = new HingeJoint(thumb_t, thumb_nail,
 					      Pos(conf.invert*(-0.7+conf.x), -0.8+conf.y, conf.invert==-1 ? -1.02+conf.z :2.0+conf.z), Axis(0, 0, 1));

break;

case(Is_Draw_under_180_degree):

thumb_nail ->setPose(osg::Matrix::rotate(conf.invert==-1 ? conf.invert*(-M_PI)+M_PI/58:-M_PI+M_PI/58, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1))*osg::Matrix::translate(conf.invert*(-0.81+conf.x), -2.6+conf.y, 0.49+conf.z));

fix_thumb_nail = new HingeJoint(thumb_t, thumb_nail,
 					      Pos(conf.invert*(-0.75+conf.x), -2.4+conf.y, 0.5+conf.z), Axis(0, 1, 0));
break;

default:
break;
}

objects.push_back(thumb_nail);

fix_thumb_nail->init(odeHandle, osgHandle, false, 0.2 );
fix_thumb_nail->setParam(dParamLoStop, -M_PI/360);
fix_thumb_nail->setParam(dParamHiStop,  M_PI/360);
joints.push_back(fix_thumb_nail);


//=======index-finger-nail=======================================================


Primitive* index_nail;
Joint* fix_index_nail;

index_nail = new Cylinder(0.19,0.0001);
index_nail -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

index_nail ->setPose(osg::Matrix::rotate(-M_PI/2, 0, 1, 0)*osg::Matrix::translate(conf.invert*(0.8+conf.x), -0.6+conf.y, conf.invert==-1 ? -1.8+conf.z:2.79+conf.z));

fix_index_nail = new HingeJoint(index_t, index_nail,
 					      Pos(conf.invert*(0.6+conf.x), -0.6+conf.y, conf.invert==-1 ? -1.75+conf.z:3.0+conf.z), Axis(1, 0, 0));

break;

case(Is_Draw_under_180_degree):

index_nail ->setPose(osg::Matrix::translate(conf.invert*(2.85+conf.x), -0.6+conf.y, conf.invert==-1 ? 0.7+conf.z:0.29+conf.z));

fix_index_nail = new HingeJoint(index_t, index_nail,
 					      Pos(conf.invert*(2.65+conf.x), -0.6+conf.y, 0.5+conf.z), Axis(1, 0, 0));

break;

default:
break;
}

objects.push_back(index_nail);

fix_index_nail->init(odeHandle, osgHandle, false, 0.2 );
fix_index_nail->setParam(dParamLoStop, -M_PI/360);
fix_index_nail->setParam(dParamHiStop,  M_PI/360);
joints.push_back(fix_index_nail);

//=======middle-finger-nail=======================================================

Primitive* middle_nail;
Joint* fix_middle_nail;
middle_nail = new Cylinder(0.19,0.0001);
middle_nail -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);

switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

middle_nail ->setPose(osg::Matrix::rotate(-M_PI/2, 0, 1, 0)*osg::Matrix::translate(conf.invert*(1.15+conf.x), -0.15+conf.y, conf.invert==-1 ? -2.05+conf.z:3.0+conf.z));

fix_middle_nail = new HingeJoint(middle_t, middle_nail,
 					      Pos(conf.invert*(1.1+conf.x), -0.15+conf.y, conf.invert==-1 ? -1.75+conf.z:2.8+conf.z), Axis(0, 0, 1));

break;

case(Is_Draw_under_180_degree):

middle_nail ->setPose(osg::Matrix::translate(conf.invert*(3.5+conf.x), -0.15+conf.y, conf.invert==-1 ? 0.7+conf.z:0.29+conf.z));

fix_middle_nail = new HingeJoint(middle_t, middle_nail,
 					      Pos(conf.invert*(3.3+conf.x), -0.15+conf.y, 0.5+conf.z), Axis(1, 0, 0));

break;

default:
break;
}

objects.push_back(middle_nail);

fix_middle_nail->init(odeHandle, osgHandle, false, 0.2 );
fix_middle_nail->setParam(dParamLoStop, -M_PI/360);
fix_middle_nail->setParam(dParamHiStop,  M_PI/360);
joints.push_back(fix_middle_nail);

//=======ring-finger-nail=======================================================

Primitive* ring_nail;
Joint* fix_ring_nail;

ring_nail = new Cylinder(0.19,0.0001);
ring_nail -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):
ring_nail ->setPose(osg::Matrix::rotate(-M_PI/2, 0, 1, 0)*osg::Matrix::translate(conf.invert*(1.1+conf.x), 0.3+conf.y, conf.invert==-1 ? -1.8+conf.z:2.79+conf.z));

fix_ring_nail = new HingeJoint(ring_t, ring_nail,
 					      Pos(conf.invert*(1.0+conf.x), 0.3+conf.y, conf.invert==-1 ? -1.55+conf.z:3.0+conf.z), Axis(1, 0, 0));

break;

case(Is_Draw_under_180_degree):

ring_nail ->setPose(osg::Matrix::translate(conf.invert*(3.2+conf.x), 0.3+conf.y,conf.invert==-1 ? 0.7+conf.z:0.29+conf.z));

fix_ring_nail = new HingeJoint(ring_t, ring_nail,
 					      Pos(conf.invert*(3.0+conf.x), 0.3+conf.y, 0.5+conf.z), Axis(1, 0, 0));

break;

default:
break;
}

objects.push_back(ring_nail);

fix_ring_nail->init(odeHandle, osgHandle, false, 0.2 );
fix_ring_nail->setParam(dParamLoStop, -M_PI/360);
fix_ring_nail->setParam(dParamHiStop,  M_PI/360);
joints.push_back(fix_ring_nail);

//=======little-finger-nail=======================================================
Primitive* little_nail;
Joint* fix_little_nail;


little_nail = new Cylinder(0.19,0.0001);
little_nail -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
switch(conf.hand_is_drawn_under_angel)
{
case(Is_Draw_under_90_degree):

little_nail ->setPose(osg::Matrix::rotate(-M_PI/2, 0, 1, 0)*osg::Matrix::translate(conf.invert*(0.8+conf.x) , 0.75+conf.y, conf.invert==-1 ? -1.8+conf.z:2.79+conf.z));

fix_little_nail = new HingeJoint(little_t, little_nail,
 					      Pos(conf.invert*(0.65+conf.x), 0.75+conf.y, (conf.invert==-1 ? -1.7+conf.z : 2.79+conf.z)), Axis(0, 0, 1));

break;

case(Is_Draw_under_180_degree):

little_nail ->setPose(osg::Matrix::translate(conf.invert*(2.8+conf.x) , 0.75+conf.y, conf.invert==-1 ? 0.7+conf.z:0.29+conf.z));

fix_little_nail = new HingeJoint(little_t, little_nail,
 					      Pos(conf.invert*(2.6+conf.x), 0.75+conf.y, 0.5+conf.z), Axis(1, 0, 0));

break;

default:
break;
}

objects.push_back(little_nail);

fix_little_nail->init(odeHandle, osgHandle, false, 0.2 );
fix_little_nail->setParam(dParamLoStop, -M_PI/360);
fix_little_nail->setParam(dParamHiStop,  M_PI/360);
joints.push_back(fix_little_nail);
*/
///////////////////////////////////////////////////////////////////////////////////
/*
Primitive* palm2 = new Cylinder(1.2,0.3);

//Anstatt von odeHandle hand_space
  palm2-> init (odeHandle , 0.1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);

  palm2->setPose(osg::Matrix::translate(0+conf.x,0.05+conf.y,0.5++conf.z));

Joint* fix_forearm_joint2 = new HingeJoint(palm, palm2, Pos(0+conf.x, 0+conf.y, 0+conf.z), Axis(1, 0, 0));
fix_forearm_joint2->init(odeHandle, osgHandle, true, 0.2 );
fix_forearm_joint2->setParam(dParamLoStop, -M_PI/360);
fix_forearm_joint2->setParam(dParamHiStop,  M_PI/360);
joints.push_back(fix_forearm_joint2);
//  palm->getOSGPrimitive()->setTexture("Images/ground.rgb"); 

objects.push_back(palm2);
*/

/*
draw_contact = new Box(0.02,0.02,0.02);
draw_contact-> init ( odeHandle , 1 , osgHandle , Primitive::Body 	
				   | Primitive::Geom | Primitive::Draw);
draw_contact->setPose(osg::Matrix::translate(0,0.05,0.5));
//draw_contact->setPosition(Pos(contact[i].geom.pos));
objects.push_back(draw_contact);
*/
    created=true;
  }


  /** destroys robot
   */
  void Hand::destroy(){
    if (created){

	for (std::vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++)
	 {
	if(*i) delete *i;
      }
      objects.clear();
    
      for (std::vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
	if(*i) delete *i;
      }
      joints.clear();
  
 for (std::vector<HingeServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
      if(*i) delete *i;
      }
      servos.clear();

      for (std::vector<AngularMotor*>::iterator i = frictionmotors.begin(); i!= 			frictionmotors.end(); i++){
	if(*i) delete *i;
      }
      frictionmotors.clear();

      irSensorBank.clear();

	dJointGroupDestroy (odeHandle.jointGroup);
      dSpaceDestroy(odeHandle.space);
	dWorldDestroy (odeHandle.world);
//	dCloseODE();
    }
    created=false;
  }



  Configurable::paramlist Hand::getParamList() const{
    paramlist list;
    list += std::pair<paramkey, paramval> (std::string("jointLimit1"), conf.jointLimit1);
    list += std::pair<paramkey, paramval> (std::string("frictionjoint"), conf.frictionJoint);
    list += std::pair<paramkey, paramval> (std::string("servo_motor_Power"),   conf.servo_motor_Power);
    list += std::pair<paramkey, paramval> (std::string("factorSensor"), conf.factorSensor);
    list += std::pair<paramkey, paramval> (std::string("irRange"), conf.irRange);
    list += std::pair<paramkey, paramval> (std::string("x"), conf.x);



/*

    list.push_back( std::pair<paramkey, paramval> (std::string("factorForce"), factorForce));
    list.push_back( std::pair<paramkey, paramval> (std::string("factorSensor"), factorSensor));
    list.push_back( std::pair<paramkey, paramval> (std::string("frictionGround"), frictionGround));
    list.push_back( std::pair<paramkey, paramval> (std::string("place"), 0));
*/
    return list;
  }


  Configurable::paramval Hand::getParam(const paramkey& key) const{
    if(key == "factorForce") return factorForce; 
    else if(key == "factorSensor") return conf.factorSensor; 
    else if(key == "jointLimit") return conf.jointLimit1; 
    else if(key == "irRange") return conf.irRange;
    else if(key == "servo_motor_Power") return conf.servo_motor_Power;
    else if(key == "x") return conf.x;
    else  return Configurable::getParam(key) ;
  }


  bool Hand::setParam(const paramkey& key, paramval val){

if(key == "servo_motor_Power") 
{
conf.servo_motor_Power=val;
for (std::vector<HingeServo*>::iterator i = servos.begin(); i!= servos.end(); i++){
    if(*i) (*i)->setPower(conf.servo_motor_Power);
  }
}
    else if(key == "factorForce") factorForce=val;
    else if(key == "factorSensor") conf.factorSensor=val; 
    else if(key == "jointLimit") conf.jointLimit1=val; 
    else if(key == "irRange") 
{
conf.irRange=val; 
for (unsigned int i=2; i< objects.size(); i++){
//     for (std::vector<Primitive*>::iterator i = objects.begin()+2; i!= objects.end(); i++){

	irSensorBank.registerSensor(ir_sensors[i-2], objects[i], 
				   osg::Matrix::rotate(-M_PI/2, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1)) * 
				  osg::Matrix::translate(conf.invert*(0), 0.2, 0), conf.irRange, 
conf.Draw_part_of_ir_sensor==Draw_All ? RaySensor::drawAll : (conf.Draw_part_of_ir_sensor == Draw_just_Sensor ? RaySensor::drawSensor :   (conf.Draw_part_of_ir_sensor == Draw_just_Ray ?  RaySensor::drawRay : RaySensor::drawNothing)));

//objects[i]->update();		
    }

irSensorBank.update();
}  
else if(key == "x") {

	conf.x=val; 
      OdeRobot::place(Pos(conf.x,conf.y,conf.z)); 

}
/*    else if(key == "place") {
      OdeRobot::place(Pos(0,0,3)) ; 
    } */
    else return Configurable::setParam(key, val);
    return true;
  }

} 
  

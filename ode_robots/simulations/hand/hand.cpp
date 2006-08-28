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
 *   Revision 1.1  2006-08-28 14:13:22  martius
 *   added hand simulation
 *
 *         
 *                                                                 *
 ***************************************************************************/

#include "hand.h"

using namespace std;

namespace lpzrobots {

  /**
   * Constructor
   * @param w world in which robot should be created
   * @param s space in which robot should be created
   * @param c contactgroup for collision treatment
   */

  Hand::Hand(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const HandConf& conf, const string& name)
    : OdeRobot(odeHandle, osgHandle, name, "$Id$"), conf(conf), oldp(0,0,0){
    /*  
	Hand::Hand(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const string& name)
	: OdeRobot(odeHandle, osgHandle, name, "$Id$"), oldp(0,0,0){
    */
    factorForce=3.0;
    factorSensor=20.0;
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
  
    for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++){
      if(*i) (*i)->update();
    }

    for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
      if(*i) (*i)->update();
    }
   
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
  }

  // internal collisions
  void Hand::mycallback(void *data, dGeomID o1, dGeomID o2){

    // connected bodies are allowed to intersect
    // if( dAreConnected(dGeomGetBody(o1), dGeomGetBody(o2)) )
    // return;

    Hand* me = (Hand*)data;  


    int i=0;
    int o1_index= -1;
    int o2_index= -1;
    for (vector<Primitive*>::iterator n = me->objects.begin(); n!= me->objects.end(); n++, i++){      
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
	  /*if(true)
	    {
	    Primitive* draw_contact = new Box(0.02,0.02,0.02);
	    //draw_contact-> init ( odeHandle , 1 , osgHandle , Primitive::Body 	
	    //				   | Primitive::Geom | Primitive::Draw);
	    draw_contact->setPosition(osg::Vec3((Pos(contact[i].geom.pos).toPosition().x),(Pos(contact[i].geom.pos).toPosition().y),(Pos(contact[i].geom.pos).toPosition().z)));
	    //objects.push_back(draw_contact);

	    }*/

	}
      }

    }
  }
  bool Hand::collisionCallback(void *data, dGeomID o1, dGeomID o2){
    //checks if one of the collision objects is part of the robot
    if( o1 == (dGeomID)odeHandle.space || o2 == (dGeomID)odeHandle.space){

      // the rest is for collisions of some snake elements with the rest of the world
      /*
      // only things colliding with hand space are considered
      int g1 = (o1 == (dGeomID)odeHandle.space|| o2 == (dGeomID)odeHandle.space);
      if (!g1) return;
      */
      // dSpaceCollide(odeHandle.space, this, mycallback);

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
	  contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	    dContactSoftERP | dContactSoftCFM | dContactApprox1;
	  contact[i].surface.mu = dInfinity;
	  contact[i].surface.slip1 = 0.1;
	  contact[i].surface.slip2 = 0.1;
	  contact[i].surface.soft_erp = 0.5;
	  contact[i].surface.soft_cfm = 0.3;
	  dJointID c = dJointCreateContact( odeHandle.world, odeHandle.jointGroup, &contact[i]);
	  dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;
	}
      }
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
	  Pos s = (p - oldp)*factorSensor;
    
	  sensors[0]=s.x();
	  sensors[1]=s.y();
	  oldp=p; 
	*/
	//!	sensors[0]=((AngularMotor*)palm_motor_joint)->getParam(dParamVel2);
	//!	sensors[1]=((AngularMotor*)palm_motor_joint)->getParam(dParamVel3);
	//	sensors[0]=((AngularMotor1Axis*)palm_motor_joint)->get(0);
	//      sensors[1]=((AngularMotor1Axis*)palm_motor_joint)->get(1);

	//	sensors[0]=((AngularMotor3AxisEuler*)palm_motor_joint)->get(0);
	//      sensors[1]=((AngularMotor3AxisEuler*)palm_motor_joint)->get(1);

      }
      else if(i==1){
	/*    Pos p(objects[2]->getPosition());      //read actual position
	      Pos s = (p - oldp)*factorSensor;
    
	      sensors[2]=s.x();
	      sensors[3]=s.y();
	      sensors[4]=s.z();
	      oldp=p; */
	//!	sensors[2]=((AngularMotor*)thumb_motor_joint)->getParam(dParamVel);
	//!	sensors[3]=((AngularMotor*)thumb_motor_joint)->getParam(dParamVel2);
	//	sensors[4]=((AngularMotor*)thumb_motor_joint)->getParam(dParamVel3);
	//	sensors[2]=((AngularMotor3AxisEuler*)thumb_motor_joint)->get(0);
	//    sensors[3]=((AngularMotor3AxisEuler*)thumb_motor_joint)->get(2);

      }
      else  {
       	sensors[3+i] = factorSensor * ((HingeJoint*)joints[i])->getPosition1Rate();

	//	sensors[2*i+1] = factorSensor * ((HingeJoint*)joints[i])->getPosition1();
      }
    }
    //  sensors[n]   = conf.sensorFactor * ((HingeJoint*)joints[n])->getPosition1Rate();
    return sensor_number+3 ;
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
	((HingeJoint*)joints[i])->setParam ( dParamVel , motors[3+i]* velocity );
	((HingeJoint*)joints[i])->setParam(dParamFMax, power); 

	//	      ((HingeJoint*)joints[i])->addTorque(factorForce * motors[3+i]);
	//((HingeJoint*)joints[i])->addTorque(factorForce * motors[2*i+1]);
      }
      /*for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
	if(*i) (*i)->update();
	}*/
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
    return sensor_number+3;
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
    Primitive* palm = new Cylinder(1.0,0.3);
    // Primitive* palm = new Box(2.0,1.8,0.3);
    //Anstatt von odeHandle hand_space
    palm-> init (odeHandle , 0.1 , osgHandle , Primitive::Body 	
		 | Primitive::Geom | Primitive::Draw);

    palm->setPose(osg::Matrix::translate(conf.invert*(0+conf.x),(0.05+conf.y),(0.5+conf.z)));

    //  palm->getOSGPrimitive()->setTexture("Images/ground.rgb"); 

    objects.push_back(palm);

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
//!     ((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop, 0);
//!     ((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop, 0);
//!     ((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop2, 0);
//!     ((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop2, 0);
//!     ((AngularMotor*)palm_motor_joint)->setParam(dParamLoStop3, 0);
//!     ((AngularMotor*)palm_motor_joint)->setParam(dParamHiStop3,  0);

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
    thumb_b ->setPose(osg::Matrix::rotate(conf.invert*M_PI/2, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1)) * osg::Matrix::translate(conf.invert*(-0.6+conf.x),(-0.8+conf.y), (0.5+conf.z)));
    objects.push_back(thumb_b);

    thumb_c = new Capsule(0.2,0.5);
    thumb_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
		      | Primitive::Geom | Primitive::Draw);
    thumb_c ->setPose(osg::Matrix::rotate(conf.invert*M_PI/6, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1)) * osg::Matrix::translate(conf.invert*(-0.1+conf.x),(-1.2+conf.y), (0.5+conf.z)));
    objects.push_back(thumb_c);

    thumb_t = new Capsule(0.2,0.5);
    thumb_t -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
		      | Primitive::Geom | Primitive::Draw);
    thumb_t ->setPose(osg::Matrix::rotate(conf.invert*M_PI/6, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1))  * osg::Matrix::translate(conf.invert*(0.6+conf.x) ,(-1.6+conf.y), (0.5+conf.z)));

    objects.push_back(thumb_t);

    //-------ball joint between palm and thumb----------------------------------------
    Joint* palm_thumb = new BallJoint(palm, thumb_b,
				      Pos(conf.invert*(-0.4+conf.x),  (-0.75+conf.y), (0.5+conf.z)));//(*) pose wurde an dieser Stelle weggelassen, kann dann aber spaeter dazugenommen werden.
    palm_thumb->init(odeHandle, osgHandle, false, 0.2);//Die groesse muss hier neu eingestellt werden, bei Ode nicht einstellbar!!!
    joints.push_back(palm_thumb);

    //-------hinge joint between thumb_b and thumb_t---------------------------------

    Joint* thumb_ct = new HingeJoint(thumb_c, thumb_t,
				     Pos(conf.invert*(0.25+conf.x), (-1.3+conf.y), (0.5+conf.z)), Axis(0, 0, conf.invert*(-1)));
    thumb_ct->init(odeHandle, osgHandle, true, 0.2 );
    thumb_ct->setParam(dParamLoStop, -M_PI/360);
    thumb_ct->setParam(dParamHiStop,  M_PI/2);
    joints.push_back(thumb_ct);


    Joint* thumb_bc = new HingeJoint(thumb_b, thumb_c,
				     Pos(conf.invert*(-0.55+conf.x), (-0.9+conf.y), (0.5+conf.z)), Axis(0, 0, conf.invert*(-1)));
    thumb_bc->init(odeHandle, osgHandle, true, 0.2 );
    thumb_bc->setParam(dParamLoStop, -M_PI/360);
    thumb_bc->setParam(dParamHiStop,  M_PI/2);
    joints.push_back(thumb_bc);

    //------------AngularMotor for BallJoint ---------------

    thumb_motor_joint = new AngularMotor3AxisEuler(odeHandle, (BallJoint*)palm_thumb, 
						   axis1, axis3, power);
    /*
      thumb_motor_joint->init(hand_space, osgHandle, true, 0.2 );
    */
    if(conf.invert==-1){
      thumb_motor_joint->getJoint()->setParam(dParamHiStop, conf.invert*(-M_PI/360));
      thumb_motor_joint->getJoint()->setParam(dParamLoStop,  conf.invert*M_PI*(3/2));
    }
    else{
      thumb_motor_joint->getJoint()->setParam(dParamLoStop, -M_PI/360);
      thumb_motor_joint->getJoint()->setParam(dParamHiStop,  M_PI*(3/2));
    }
    thumb_motor_joint->getJoint()->setParam(dParamLoStop2, -M_PI/360);
    thumb_motor_joint->getJoint()->setParam(dParamHiStop2,  M_PI/360);
    thumb_motor_joint->getJoint()->setParam(dParamLoStop3, -M_PI/360);
    thumb_motor_joint->getJoint()->setParam(dParamHiStop3,  M_PI/360);


    frictionmotors.push_back(thumb_motor_joint);
    //frictionmotors.push_back(new AngularMotor3AxisEuler(odeHandle, (BallJoint*)palm_thumb, axis1, axis3, conf.power));

    //-----------index finger--------------------------------------------------

    Primitive *index_b, *index_c, *index_t;

    index_b = new Capsule(0.2,0.6);
    index_b -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
		      | Primitive::Geom | Primitive::Draw);
    index_b ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(1.0+conf.x), (-0.6+conf.y), (0.5+conf.z)));
    objects.push_back(index_b);


    index_c = new Capsule(0.2,0.6);
    index_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
		      | Primitive::Geom | Primitive::Draw);
    index_c ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(1.8+conf.x), (-0.6+conf.y), (0.5+conf.z)));
    objects.push_back(index_c);


    index_t = new Capsule(0.2,0.5);
    index_t -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
		      | Primitive::Geom | Primitive::Draw);
    index_t ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(2.45+conf.x),(-0.6+conf.y), (0.5+conf.z)));
    objects.push_back(index_t);

    //----------------index finger joints----------------------------------------------
    Joint *palm_index, *index_bc, *index_ct;

    palm_index = new HingeJoint(palm, index_b,
				Pos(conf.invert*(0.6+conf.x), (-0.6+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
    palm_index ->init(odeHandle, osgHandle, true, 0.2 );
    palm_index ->setParam(dParamHiStop,  M_PI/2);
    palm_index ->setParam(dParamLoStop, -M_PI/360);
    joints.push_back(palm_index);

    index_bc = new HingeJoint(index_b, index_c,
			      Pos(conf.invert*(1.4+conf.x),(-0.6+conf.y),(0.5+conf.z)), Axis(0, 1, 0));
    index_bc ->init(odeHandle, osgHandle, true, 0.2 );
    index_bc ->setParam(dParamHiStop,  M_PI/2);
    index_bc ->setParam(dParamLoStop, -M_PI/360);
    joints.push_back(index_bc);

    index_ct = new HingeJoint(index_c, index_t,
			      Pos(conf.invert*(2.2+conf.x), (-0.6+conf.y),(0.5+conf.z)), Axis(0, 1, 0));
    index_ct ->init(odeHandle, osgHandle, true, 0.2 );
    index_ct ->setParam(dParamHiStop,  M_PI/2);
    index_ct ->setParam(dParamLoStop, -M_PI/360);
    joints.push_back(index_ct);

 

    //---------middle finger----------------------------------------------------------  

    Primitive *middle_b,*middle_c,*middle_t;


    middle_b = new Capsule(0.2,0.7);
    middle_b -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
		       | Primitive::Geom | Primitive::Draw);
    middle_b->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(1.35+conf.x), (-0.15+conf.y), (0.5+conf.z)));
    objects.push_back(middle_b);


    middle_c = new Capsule(0.2,0.7);
    middle_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
		       | Primitive::Geom | Primitive::Draw);
    middle_c ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(2.25+conf.x), (-0.15+conf.y), (0.5+conf.z)));
    objects.push_back(middle_c);



    middle_t = new Capsule(0.2,0.6);
    middle_t -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
		       | Primitive::Geom | Primitive::Draw);
    middle_t ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(3.1+conf.x),(-0.15+conf.y), (0.5+conf.z)));
    objects.push_back(middle_t);

    //-------------------------middle finger joints-------------------------------------
    Joint *palm_middle, *middle_bc, *middle_ct;

    palm_middle = new HingeJoint(palm, middle_b,
				 Pos(conf.invert*(0.9+conf.x), (-0.15+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
    palm_middle ->init(odeHandle, osgHandle, true, 0.2 );
    palm_middle->setParam(dParamHiStop,  M_PI/2);
    palm_middle->setParam(dParamLoStop, -M_PI/360);
    joints.push_back(palm_middle);



    middle_bc = new HingeJoint(middle_b, middle_c,
			       Pos(conf.invert*(1.8+conf.x), (-0.15+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
    middle_bc ->init(odeHandle, osgHandle, true, 0.2 );
    middle_bc->setParam(dParamHiStop,  M_PI/2);
    middle_bc->setParam(dParamLoStop, -M_PI/360);
    joints.push_back(middle_bc);


    middle_ct = new HingeJoint(middle_c, middle_t,
			       Pos(conf.invert*(2.7+conf.x), (-0.15+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
    middle_ct ->init(odeHandle, osgHandle, true, 0.2 );
    middle_ct->setParam(dParamHiStop,  M_PI/2);
    middle_ct->setParam(dParamLoStop, -M_PI/360);
    joints.push_back(middle_ct);


    //---------------------------ring finger------------------------------------------- 
    Primitive *ring_b, *ring_c, *ring_t;

    ring_b = new Capsule(0.2,0.6);
    ring_b-> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
		    | Primitive::Geom | Primitive::Draw);
    ring_b ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(1.3+conf.x), (0.3+conf.y), (0.5+conf.z)));
    objects.push_back(ring_b);

    ring_c = new Capsule(0.2,0.6);
    ring_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
		     | Primitive::Geom | Primitive::Draw);
    ring_c ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(2.1+conf.x), (0.3+conf.y), (0.5+conf.z)));
    objects.push_back(ring_c);


    ring_t = new Capsule(0.2,0.5);
    ring_t -> init ( odeHandle, 0.1 , osgHandle , Primitive::Body 	
		     | Primitive::Geom | Primitive::Draw);
    ring_t ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(2.85+conf.x) , (0.3+conf.y), (0.5+conf.z)));
    objects.push_back(ring_t);




    //-----------ring finger joints----------------------------------------------------
    Joint *palm_ring, *ring_bc, *ring_ct;

    palm_ring = new HingeJoint(palm, ring_b,
			       Pos(conf.invert*(0.9+conf.x), (0.3+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
    palm_ring->init(odeHandle, osgHandle, true, 0.2 );
    palm_ring->setParam(dParamHiStop,  M_PI/2);
    palm_ring->setParam(dParamLoStop, -M_PI/360);
    joints.push_back(palm_ring);




    ring_bc = new HingeJoint(ring_b, ring_c,
			     Pos(conf.invert*(1.7+conf.x), (0.3+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
    ring_bc ->init(odeHandle, osgHandle, true, 0.2 );
    ring_bc->setParam(dParamHiStop,  M_PI/2);
    ring_bc->setParam(dParamLoStop, -M_PI/360);
    joints.push_back(ring_bc);


    ring_ct = new HingeJoint(ring_c, ring_t,
			     Pos(conf.invert*(2.5+conf.x), (0.3+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
    ring_ct ->init(odeHandle, osgHandle, true, 0.2 );
    ring_ct->setParam(dParamHiStop,  M_PI/2);
    ring_ct->setParam(dParamLoStop, -M_PI/360);
    joints.push_back(ring_ct);


    //-----------little finger--------------------------------------------------------

    Primitive *little_b, *little_c, *little_t;

    little_b = new Capsule(0.2,0.6);
    little_b -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
		       | Primitive::Geom | Primitive::Draw);
    little_b ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(0.9+conf.x), (0.75+conf.y), (0.5+conf.z)));
    objects.push_back(little_b);

    little_c = new Capsule(0.2,0.6);
    little_c -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
		       | Primitive::Geom | Primitive::Draw);
    little_c ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(1.7+conf.x), (0.75+conf.y), (0.5+conf.z)));
    objects.push_back(little_c);



    little_t = new Capsule(0.2,0.5);
    little_t -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
		       | Primitive::Geom | Primitive::Draw);

    little_t ->setPose(osg::Matrix::rotate(M_PI/2, 0, 1, 0) * osg::Matrix::translate(conf.invert*(2.45+conf.x) , (0.75+conf.y), (0.5+conf.z)));
    objects.push_back(little_t);



    //-----------------little finger joints-----------------------------------------

    Joint *palm_little, *little_bc, *little_ct;

    palm_little = new HingeJoint(palm, little_b,
				 Pos(conf.invert*(0.6+conf.x), (0.75+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
    palm_little ->init(odeHandle, osgHandle, true, 0.2 );
    palm_little ->setParam(dParamHiStop,  M_PI/2);
    palm_little ->setParam(dParamLoStop, -M_PI/360);
    joints.push_back(palm_little);

    little_bc = new HingeJoint(little_b, little_c,
			       Pos(conf.invert*(1.4+conf.x), (0.75+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
    little_bc ->init(odeHandle, osgHandle, true, 0.2 );
    little_bc ->setParam(dParamHiStop,  M_PI/2);
    little_bc ->setParam(dParamLoStop, -M_PI/360);
    joints.push_back(little_bc);

    little_ct = new HingeJoint(little_c, little_t,
			       Pos(conf.invert*(2.1+conf.x), (0.75+conf.y), (0.5+conf.z)), Axis(0, 1, 0));
    little_ct ->init(odeHandle, osgHandle, true, 0.2 );
    little_ct ->setParam(dParamHiStop,  M_PI/2);
    little_ct ->setParam(dParamLoStop, -M_PI/360);
    joints.push_back(little_ct);



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


    //=======thumb-finger-nail=======================================================

    Primitive* thumb_nail;
    thumb_nail = new Cylinder(0.19,0.0001);
    thumb_nail -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
			 | Primitive::Geom | Primitive::Draw);

    thumb_nail ->setPose(osg::Matrix::rotate(conf.invert==-1 ? conf.invert*(-M_PI/3):-M_PI/3, osg::Vec3(1, 0, 0),M_PI/2, osg::Vec3(0, 1, 0),0.0,osg::Vec3(0, 0, 1))*osg::Matrix::translate(conf.invert*(0.8+conf.x), -1.95+conf.y, 0.49+conf.z));
    objects.push_back(thumb_nail);

    Joint* fix_thumb_nail;
    fix_thumb_nail = new HingeJoint(thumb_t, thumb_nail,
				    Pos(conf.invert*(0.6+conf.x), -1.6+conf.y, 0.5+conf.z), Axis(1, 0, 0));
    fix_thumb_nail->init(odeHandle, osgHandle, false, 0.2 );
    fix_thumb_nail->setParam(dParamLoStop, -M_PI/360);
    fix_thumb_nail->setParam(dParamHiStop,  M_PI/360);
    joints.push_back(fix_thumb_nail);


    //=======index-finger-nail=======================================================


    Primitive* index_nail;
    index_nail = new Cylinder(0.19,0.0001);
    index_nail -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
			 | Primitive::Geom | Primitive::Draw);

    index_nail ->setPose(osg::Matrix::translate(conf.invert*(2.85+conf.x), -0.6+conf.y, conf.invert==-1 ? 0.7+conf.z:0.29+conf.z));
    objects.push_back(index_nail);

    Joint* fix_index_nail;
    fix_index_nail = new HingeJoint(index_t, index_nail,
				    Pos(conf.invert*(2.65+conf.x), -0.6+conf.y, 0.5+conf.z), Axis(1, 0, 0));
    fix_index_nail->init(odeHandle, osgHandle, false, 0.2 );
    fix_index_nail->setParam(dParamLoStop, -M_PI/360);
    fix_index_nail->setParam(dParamHiStop,  M_PI/360);
    joints.push_back(fix_index_nail);

    //=======middle-finger-nail=======================================================

    Primitive* middle_nail;
    middle_nail = new Cylinder(0.19,0.0001);
    middle_nail -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
			  | Primitive::Geom | Primitive::Draw);

    middle_nail ->setPose(osg::Matrix::translate(conf.invert*(3.5+conf.x), -0.15+conf.y, conf.invert==-1 ? 0.7+conf.z:0.29+conf.z));
    objects.push_back(middle_nail);

    Joint* fix_middle_nail;
    fix_middle_nail = new HingeJoint(middle_t, middle_nail,
				     Pos(conf.invert*(3.3+conf.x), -0.15+conf.y, 0.5+conf.z), Axis(1, 0, 0));
    fix_middle_nail->init(odeHandle, osgHandle, false, 0.2 );
    fix_middle_nail->setParam(dParamLoStop, -M_PI/360);
    fix_middle_nail->setParam(dParamHiStop,  M_PI/360);
    joints.push_back(fix_middle_nail);

    //=======ring-finger-nail=======================================================

    Primitive* ring_nail;
    ring_nail = new Cylinder(0.19,0.0001);
    ring_nail -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
			| Primitive::Geom | Primitive::Draw);
    ring_nail ->setPose(osg::Matrix::translate(conf.invert*(3.2+conf.x), 0.3+conf.y,conf.invert==-1 ? 0.7+conf.z:0.29+conf.z));
    objects.push_back(ring_nail);

    Joint* fix_ring_nail;
    fix_ring_nail = new HingeJoint(ring_t, ring_nail,
				   Pos(conf.invert*(3.0+conf.x), 0.3+conf.y, 0.5+conf.z), Axis(1, 0, 0));
    fix_ring_nail->init(odeHandle, osgHandle, false, 0.2 );
    fix_ring_nail->setParam(dParamLoStop, -M_PI/360);
    fix_ring_nail->setParam(dParamHiStop,  M_PI/360);
    joints.push_back(fix_ring_nail);

    //=======little-finger-nail=======================================================
    Primitive* little_nail;
    little_nail = new Cylinder(0.19,0.0001);
    little_nail -> init ( odeHandle , 0.1 , osgHandle , Primitive::Body 	
			  | Primitive::Geom | Primitive::Draw);
    little_nail ->setPose(osg::Matrix::translate(conf.invert*(2.8+conf.x) , 0.75+conf.y, conf.invert==-1 ? 0.7+conf.z:0.29+conf.z));
    objects.push_back(little_nail);

    Joint* fix_little_nail;
    fix_little_nail = new HingeJoint(little_t, little_nail,
				     Pos(conf.invert*(2.6+conf.x), 0.75+conf.y, 0.5+conf.z), Axis(1, 0, 0));
    fix_little_nail->init(odeHandle, osgHandle, false, 0.2 );
    fix_little_nail->setParam(dParamLoStop, -M_PI/360);
    fix_little_nail->setParam(dParamHiStop,  M_PI/360);
    joints.push_back(fix_little_nail);

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

      for (vector<Primitive*>::iterator i = objects.begin(); i!= objects.end(); i++)
	{
	  if(*i) delete *i;
	}
      objects.clear();
    
      for (vector<Joint*>::iterator i = joints.begin(); i!= joints.end(); i++){
	if(*i) delete *i;
      }
      joints.clear();
  
      for (vector<AngularMotor*>::iterator i = frictionmotors.begin(); i!= 			frictionmotors.end(); i++){
	if(*i) delete *i;
      }
      frictionmotors.clear();

      dJointGroupDestroy (odeHandle.jointGroup);
      dSpaceDestroy(odeHandle.space);
      dWorldDestroy (odeHandle.world);
      //	dCloseODE();
    }
    created=false;
  }



  Configurable::paramlist Hand::getParamList() const{
    paramlist list;
    list.push_back( pair<paramkey, paramval> (string("factorForce"), factorForce));
    list.push_back( pair<paramkey, paramval> (string("factorSensor"), factorSensor));
    list.push_back( pair<paramkey, paramval> (string("frictionGround"), frictionGround));
    list.push_back( pair<paramkey, paramval> (string("place"), 0));
    return list;
  }


  Configurable::paramval Hand::getParam(const paramkey& key) const{
    if(key == "factorForce") return factorForce; 
    else if(key == "factorSensor") return factorSensor; 
    else if(key == "frictionGround") return frictionGround; 
    else  return Configurable::getParam(key) ;
  }


  bool Hand::setParam(const paramkey& key, paramval val){
    if(key == "factorForce") factorForce=val;
    else if(key == "factorSensor") factorSensor=val; 
    else if(key == "frictionGround") frictionGround=val; 
    else if(key == "place") {
      OdeRobot::place(Pos(0,0,3)) ; 
    }
    else return Configurable::setParam(key, val);
    return true;
  }

} 
  

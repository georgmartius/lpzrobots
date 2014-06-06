/***************************************************************************
 *   Copyright (C) 2013                                                    *
 *    wbj                                                                  *
 *    Frank Hesse    <fhesse at physik3 dot gwdg dot de>                   *
 *									                                                       *
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
#include <ode-dbl/ode.h>

//used for debugging
#include <iostream>
#include <stdio.h>

// include primitives (box, spheres, cylinders ...)
#include "ode_robots/primitive.h"
#include "ode_robots/osgprimitive.h"

// include joints
#include "ode_robots/joint.h"

//include motors
#include "ode_robots/oneaxisservo.h"

// include header file
#include "kuka.h"

#include <vector>




using namespace osg;


namespace lpzrobots {

  // constructor:
  // - give handle for ODE and OSG stuff
  // - size of arm is adjustable
  // - objectsOf Interest holds all primitives the robot to which the robot will measure its relative position
  Kuka::Kuka(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
           const std::string& name,
           double size,
	std::vector<Primitive*> objectsOfInterest)
    : // calling OdeRobots construtor with name of the actual robot
      OdeRobot(odeHandle, osgHandle, name, "$Id$")
  {
    // robot is not created till now
    created=false;

    // choose color
    this->osgHandle.color = Color(2, 156/255.0, 0, 1.0f);

        manualControl = false;        	  	// robot listens for motorcommands sent by controller
        length=        size*0.21;         	// length of segments 21cm
        width=        length/4.5;      		// diameter of body
	gripper_radius = size*0.015;		//radius of the gripper
	endeffector_radius = size*0.05;		// radius of the endeffector (black ball)
	socketSize = size*0.117	;		//size of the socket/ the segment fixed to the environment
	simSize= size;

	this->objectsOfInterest = objectsOfInterest;
	segmentsno=    8;           		// number of segments of the robot
        cmass=        0.7;			//16/segmentsno-1;      // mass of the segments
	jointsno =     segmentsno-1;         	// number of joints (same as sensors, motors)
	/* number of sensors:
		- 7 joint position,
		- 7 joint speed,
		- 1 for the gripper (>0 if something is grasped)
		- 1 distance to ball	*/
	sensorno=    jointsno*2 + 1 + 3*objectsOfInterest.size();// joints, jointspeed, gripper-state, x,y,z-distances to the object of interest
        motorno=    jointsno; //+1;       // number of motors: 7 hingeServos, 1 for grasping/releasing,

	//chain = KDL::KukaLWR_DHnew();		// a model of the kuka arm used to do inverse kinematics calculations

    // these vectors hold the speed limit and the maximal torques for each of the joints
    speed.resize(jointsno);
    max_force.resize(jointsno);
	//when the manual control mode is active, this vector holds the position of the joints
    manualCommands.resize(jointsno);

    //setting the speed limits for each joint:
    for(int i=0; i<jointsno; i++){
        if (i == 4){ speed[i]= 3.1416;}     // 180°/s equals pi/s approximated as 3.1416
        else { speed[i]= 1.9635;}        // 112.5°/s equals 1.9635 rad/s
    }
    //adjusting the maximum torque
	// to reduce the noise, these values have been set very high
	// the origial values that correspond to the actual force limits of the Kuka arm are in the comment
    for(int i=0; i<jointsno; i++){
        if (i<2)              { max_force[i]= 1500;} 	//200   //SI Unit Nm
        if (i > 1 and i < 5)  { max_force[i]= 1000;}	//100
        else                  { max_force[i] = 1000;}	//30
    }
  };


  /** sets actual motorcommands
      @param motors motors scaled to [-1,1]
      @param motornumber length of the motor array
  */
  void Kuka::setMotorsIntern(const double* motors, int motornumber){
    assert(created); 	// robot must exist
    if (!manualControl){//this is for the regular case when the robot receives motor values
	for (int i=0; i<7; i++){
		//forward and backward rotation have to be switched at those joints to match the real kuka robot arm
		hingeServos[i]->set(motors[i]*((i==0 or i==2 or i==3 or i==4 or i==6)? -1:1));
	}
    }
//manual control:
    else {
        for (int i =0; i < 7; i++){
            hingeServos[i]->set(manualCommands[i]*( (i==0 or i==2 or i==3 or i==4 or i==6)? -1:1)   / ( (i%2==0)? 2.97:2.094) ) ;
        }
    }
  };

  /** returns actual sensorvalues
	7 sensor values for the motor position [-1,1]
	7 sensor values for the joint speeds
	1 sensor whether gripper is active (arm is holding an object)
	3 distance sensor value to the first objectOfInterest
      @param sensors sensors scaled to [-1,1] (more or less)
      @param sensornumber length of the sensor array
      @return number of actually written sensors
  */
  int Kuka::getSensorsIntern(double* sensors, int sensornumber){
    //std::cout << "getSensors wurde begonnen\n";
    assert(created); // robot must exist

	// the first 7 sensorvalues are the joint positions in radians
    	for (int i=0; i<jointsno; i++){
        	sensors[i] = dynamic_cast<HingeJoint*>(joints[i])->getPosition1()*((i==0 or i==2 or i==3 or i==4 or i==6)? -1:1);
		// as the +/- directions of the Kuka arm are different from the +/- directions of the LpzRobots Simulator
		// the values of joints 0,2,3,4 and 6 have to multiplied with -1
	}
	// the second 7 sensorvalues are the jointspeeds
	// angular speed in radians/s
    	for (int i=jointsno; i<jointsno*2; i++){
        	sensors[i]= dynamic_cast<HingeJoint*>(joints[i-7])->getPosition1Rate()*((i==0 or i==2 or i==3 or i==4 or i==6)? -1:1);
        }

	// sensor 14 returns whether the gripper:
		// - is empty and inactive (=0),
		// - holds an object (= 1)
		// - is active but does not hold an object (-1)

	if (grasped)	{sensors[14] = 1;}
	else if (gripper_active) {
		sensors[14]=-1;
		gripper_active = false;	//this resets the gripper making it available in the next step again
	}
	else {sensors[14]=0;}

	int n = objectsOfInterest.size();
	for (int i=0; i<n;i++){
	//3 distances (x,y,z) to the objectOfInterest i
		//sensor distances[3] = {0};
    //relSensors[i]->get(distances,3);
    //sensors[15+i*3] = distances[0];
    //sensors[16+i*3] = distances[1];
    //sensors[17+i*3] = distances[2];
		std::list<sensor> distances;
		distances = relSensors[i]->getList();
		sensors[15+i*3] = distances.front(); distances.pop_front();
		sensors[16+i*3] = distances.front(); distances.pop_front();
		sensors[17+i*3] = distances.front(); distances.pop_front();
	}

    // the number of read sensors is returned
    return sensorno;

  };


  void Kuka::placeIntern(const osg::Matrix& pose){
    osg::Matrix p2;
    p2 = pose * osg::Matrix::translate(Vec3(0, 0, 0));
    create(p2);
  };


  /**
   * updates the osg notes
   */
  void Kuka::update() {
    OdeRobot::update();
    assert(created); // robot must exist
    if (grasped){
      griff->update();
    }
  };

  /** creates arm at desired pose
      @param pose matrix with desired position and orientation
  */
  void Kuka::create( const osg::Matrix& pose ){
    if (created) {  // if robot exists destroy it
      destroy();
    }
    // create arm space (intern collision handling)
    odeHandle.createNewSimpleSpace(parentspace, true);

    objects.resize(segmentsno);
    joints.resize(jointsno);
    hingeServos.resize(jointsno);

	//initialize the manualCommands vector
    	for (int i=0; i<7; i++) {manualCommands[i] = 0;}

	//resize the vector that holds the relativeDistanceSensors
	relSensors.resize(objectsOfInterest.size());

    // create a box as the socket for the Kuka arm
    Box* fixation = new Box(socketSize,socketSize,socketSize);
    fixation->init(odeHandle, cmass*2, osgHandle);
    Vec3 posBox = Vec3(0,0,socketSize*0.5);
        fixation->setPose(osg::Matrix::translate(posBox) * pose);
       objects[0]= fixation;

    //creating the arm segment spheres
    for (int i = 1;i<segmentsno-2; i++ ){
        Capsule* arm = new Capsule(width, length );
            arm->init(odeHandle, cmass, osgHandle);
        Vec3 posArm = Vec3(0,0,socketSize + (i-0.5)*length);
            arm->setPose(osg::Matrix::translate(posArm) * pose);
           objects[i]= arm;
    }

    //create the "endeffector"
    Capsule* ball = new Capsule(endeffector_radius, 0);
    ball->setTexture("Images/eye.rgb");
    ball->init(odeHandle, cmass, osgHandle);
    Vec3 posHand = Vec3(0, 0, socketSize+5*length);
    ball->setPose(osg::Matrix::translate(posHand) * pose);
    objects[segmentsno-2] = ball;
    endeffector = ball;


    //create a grasping device ( e.o.the universal jamming gripper)
    Box* ujgrip = new Box(gripper_radius, gripper_radius, gripper_radius);
    gripper = ujgrip;
    ujgrip->setTexture("Images/really_white.rgb");
    ujgrip->init(odeHandle, 0.01, osgHandle);
    Vec3 posFinger = posHand + Vec3(0,0,endeffector_radius);
    ujgrip->setPose(osg::Matrix::translate(posFinger) * pose);
    objects[segmentsno-1] = ujgrip;
    grasped = false;
    gripper_active = false;

    //creating the joints
    // anchor is set to the middle between the two segments
    for (int i = 0; i<jointsno; i++){
        if (i == 5){ //special case for joint A5
            joints[i] = new HingeJoint( objects[i], objects[i+1],
                    (objects[i+1]->getPosition()),
                      Axis(0,1,0)*pose );
        }
        else {
            joints[i] = new HingeJoint( objects[i], objects[i+1],
                    (objects[i]->getPosition() + objects[i+1]->getPosition())/2,
                      Axis(0,(i%2==0? 0:1),(i%2==0? 1:0))*pose );
            }

        joints[i]->init(odeHandle,osgHandle,true,0.1);
        joints[i]->setParam(dParamHiStop, (i%2==0? 2.97:2.094));    //range of motion either +/- 170° or +/- 120°
        joints[i]->setParam(dParamLoStop, (i%2==0? -2.97:-2.094));    //converted to fractions of 2pi

        //adding servos to the joints
        hingeServos[i] = new OneAxisServoVel(odeHandle, joints[i],(i%2==0? -2.97:-2.094),(i%2==0? 2.97:2.094),
                            getPower(i), 0.2 , 2 , getVelocity(i));

    }

	//configure the relativePositionSensors with:
	// - the maximal distance they can have to the objects of interest
	//	--> this is set to one, therefore the sensors will not work when the distance is greater
	// - the endeffector
	// - and the reference object
	for (unsigned int i=0; i< objectsOfInterest.size(); i++){
    		RelativePositionSensor* sensor = new RelativePositionSensor( 1,1);
		sensor->init(endeffector);
		sensor->setReference(objectsOfInterest[i]);
		relSensors[i]= sensor;
	}

    created=true; // robot is created

 };

	/**enables or disables the manual control mode
	in manual control mode the robot ignores all commands sent by a controller and moves the joints
	to the positions given in the manualCommands vector*/
    void Kuka::toggleManualControlMode(){
        manualControl = !manualControl;
	for (int i=0; i< jointsno; i++){
		manualCommands[i] = dynamic_cast<HingeJoint*>(joints[i])->getPosition1()*((i==0 or i==2 or i==3 or i==4 or i==6)? -1:1);
	}
        if(manualControl) std::cout << "enabled manual control mode\n";
        else std::cout << "disabled manual control mode\n";
    }


    // this method can be called during the simulation to have a better look at one or more joints
    /** blocks one joint
	@param index of the joint to be blocked
    */
    void Kuka::blockJoints(int joint){
        switch (joint)
        {
        case -2: // block no joint / reactivate all blocked joints
            for (int i =0; i<jointsno;i++){
                  joints[i]->setParam(dParamHiStop, (  (i%2==0)? 2.97:2.094) );
                joints[i]->setParam(dParamLoStop, (  (i%2==0)? -2.97:-2.094) );
            }
            break;
        case -1:  //block all joints
            for (int i =0; i<jointsno;i++){
                if (i != joint){
                    joints[i]->setParam(dParamHiStop, 0);
                    joints[i]->setParam(dParamLoStop, 0);
                }
            }
            std::cout << "all joints blocked!\n";
            break;

        default: //block only one joint
            if (joint> -1 and joint < jointsno){
                joints[joint]->setParam(dParamHiStop, 0);
                joints[joint]->setParam(dParamLoStop, 0);
            }
            std::cout << "joint"<<joint<<" blocked!\n";

        }
    };



    /** grasps an object by creating a fixed joint between the endeffector (Not the gripper!)
        and a desired object
        @param the primitive that is to be grasped
    */
	// - right now grasping will be successful, if the x- and y-distance between the center of the object and the center of the
	//   endeffector is <0.052
	// - a minimum for the z-distance can be set but is not active at the moment

    bool Kuka::grasp(Primitive *object){
	double xdis, ydis;//, zdis;
//	sensor distances[3] = {0};
//	relSensors[0]->get(distances,3);
//	xdis = fabs(distances[0]);
//	ydis = fabs(distances[1]);
//	//zdis = distances[2];

  std::list<sensor> distances;
  distances = relSensors[0]->getList();
  xdis = distances.front(); distances.pop_front();
  ydis = distances.front(); distances.pop_front();
  //zdis = distances.front(); distances.pop_front();


	double grasping_distance = 0.052;
        if (xdis <=grasping_distance and ydis<=grasping_distance /*and fabs(zdis) <0.1*/ and !grasped)	{
            	griff = new FixedJoint(endeffector , object,
                    (endeffector->getPosition() + object->getPosition())/2);
            	griff->init(odeHandle,osgHandle,true,0.2);
            	grasped = true;
		return true;
        }
        else {
		if (grasped){
			std::cout << "gripper is already active!"<<std::endl;
		}
		else{
		//std::cout<<"object is too far away."<<std::endl;
		gripper_active= true;
		}
		return false;
        }
    };

//this method sets sensor 23 to the given value as long as the gripper is active
// this is used to return the colo of the ball
 bool Kuka::grasp(Primitive *object, int sensor23){
	bool test = grasp(object);
	if (test == true){
		this->sensor23 = sensor23;
	}
	return test;
}


    /** releases the grasped object
    */
    void Kuka::release(){
	sensor23 = -1;
        if (grasped){
            delete griff;
            griff = 0;
            grasped = false;}
    };

    /** returns the maximal power of a motor
    */
    int Kuka::getPower( int i){
    if (i<2) return 200;
    if (i>1 and i<5) return 100;
    else return 30;
    };

    /** returns the maximal velocity of a motor
    */
    int Kuka::getVelocity( int i){
    return ((i==4)? 3.1416:1.9635);            // 180°/s equals pi/s approximated as 3.1416
                            			// 112.5°/s equals 1.9635 rad/s
    };



  /** destroys vehicle and space
   */
void Kuka::destroy(){
	if (created){
		for (int i=0; i<jointsno; i++){
			if(joints[i]) delete joints[i]; // destroy bodies and geoms
		}
		if (grasped) {delete griff;}
		/*
		for (int i=0; i<segmentsno; i++){
			if(objects[i]) delete objects[i]; // destroy bodies and geoms
		}*/
		cleanup();
		odeHandle.deleteSpace(); // destroy space


	}
	created=false; // robot does not exist (anymore)
};

    /** prints the current joint configuration as a vector of reals in [-1,1] and as a vector of degrees
        only available in manual control mode
    */
    void Kuka::printJointConf(){
        if (manualControl){
            std::cout <<  "current joint configuration: \n{ ";
            for (int i=0; i< motorno;i++){
                std::cout  <<manualCommands[i] << "\t";
            }
            std::cout << "} or\n{ ";
            for (int i=0; i< 7;i++){
                std::cout  << dynamic_cast<HingeJoint*>(joints[i])->getPosition1()*57 *((i==0 or i==2 or i==3 or i==4 or i==6)? -1:1) << "°\t";
            }
            std::cout << "}\n";
        }
        else std::cout << "this method is currently not available outside of manual Control mode!"<<std::endl;
    };

    /**moves a desired joint to a desired position. Only available in manual control mode
     @param jointno the number of the joint to be moved
     @param pos the desired position of the joint in radians. +/- directions of the joint as defined in the Kuka manual
    */
    void Kuka::moveJoint(int jointno, double pos){
        if (abs(pos) < (jointno%2==0? 2.97:2.094)){
            manualCommands[jointno] = pos;
        }
        else std::cout<<"invalid command! requested position is beyond joint Limits!\n";
    };

}

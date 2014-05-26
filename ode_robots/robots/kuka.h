/***************************************************************************
 *   Copyright (C) 2013                                                    *
 *    wbj                                                                  *
 *    Frank Hesse    <fhesse at physik3 dot gwdg dot de>                   *
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
#ifndef __KUKA_H
#define __KUKA_H

#include "ode_robots/oderobot.h"
#include <vector>
#include "ode_robots/oneaxisservo.h"
#include "ode_robots/relativepositionsensor.h"
#include <osg/io_utils>
#include <math.h>

/*
//the inverse kinematics from the fuerte package
#include "models/models.hpp"
#include "src/chain.hpp"
#include <src/frames_io.hpp>
//#include <src/chainiksolverpos_lma.hpp>
#include <src/chainiksolverpos_nr.hpp>
#include <src/chainiksolverpos_nr_jl.hpp>
#include <src/chainfksolver.hpp>
#include <src/chainfksolverpos_recursive.hpp>
#include <src/chainiksolvervel_pinv.hpp>
#include <src/chainiksolvervel_pinv_nso.hpp>
#include <src/chainiksolvervel_pinv_givens.hpp>
#include <src/chainiksolvervel_wdls.hpp>
#include <src/chainiksolverpos_lma.hpp>

//#include <orocos_kinematics_dynamics/orocos_kdl/src/chainiksolverpos_lma.hpp>
*/
namespace lpzrobots {

  class Primitive;
  class Hinge2Joint;

  /**Robot that emulates the Kuka lightweight robot arm,
    consisting of 5 arm segments, one endeffector upon and a small box representing
	the Tool-Center-Point (TCP) of the endeffector where different tools
	can be mounted.
    All segments are linked with hinge joints, whch allow a One-axis-rotation above
	the x- or z- axis.
	Relative position sensors can measure its relative position to given objects.
	The sensors have to be normalized with the maximal distance, they will measure during the simulation (default = 1).
  */
  class Kuka : public OdeRobot{
  public:
	friend class Kuka2;
    /**
     * constructor of  robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param size scaling of the robot (original size = 1)
     * @param objectsOfInterest a list of primitives (objects, obstacles) to which the robot will measure its relative position
     */
    Kuka(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const std::string& name,
      double size=1, std::vector<Primitive*> objectsOfInterest = std::vector<Primitive*>());

    virtual ~Kuka(){
      destroy();
    };

    /**
     * updates the OSG nodes of the vehicle
     */
    virtual void update();


    /** sets the pose of the vehicle
    @param pose desired pose matrix
    */
    virtual void placeIntern(const osg::Matrix& pose);

    /** returns actual sensorvalues see kuka.cpp to see what values are returned
    @param sensors sensors scaled to [-1,1]
    @param sensornumber length of the sensor array
    @return number of actually written sensors
    */
    virtual int getSensorsIntern(sensor* sensors, int sensornumber);

    /** sets actual motorcommands
    @param motors motors scaled to [-1,1]
    @param motornumber length of the motor array
    */
    virtual void setMotorsIntern(const double* motors, int motornumber);

    /** blocks and reactivates joints during the simulations
        the blocked joint will be set to its initial position
        @param jointNo the index of the only joint that is to be blocked
    */
    virtual void blockJoints(int joint);

    /** grasps an object by creating a fixed joint between the gripper (outermost primitive of the arm)
        and a desired object
        @param the primitive that is to be grasped
    */
    virtual bool grasp(Primitive *object);

/* grasps an object by creating a fixed joint between the gripper (outermost primitive of the arm)
        and a desired object and sets the sensor value 23 to the given value
        @param the primitive that is to be grasped
	@param sensor23 value that is to be returned by sensor 23 as long as the object is grasped.
    	*/
	   virtual bool grasp(Primitive *object, int sensor23);

    /** releases the grasped object
    */
    virtual void release();


    /** returns number of sensors
     */
    virtual int getSensorNumberIntern(){
      return sensorno;
    };

    /** returns number of motors
     */
    virtual int getMotorNumberIntern(){
      return motorno;
    };

    /** switches between manual Control (motor commands written into the manualCommands-vector) and Controller commands
    */
    virtual void toggleManualControlMode();


    /** enables the manual command mode an moves the arm to its initial upright position
    */
    virtual void candle(){
        manualControl = true;
        for (int i=0; i<motorno; i++) {manualCommands[i] = 0;}
    };

	/** returns the desired position of a joint during the manua control mode:
	 the current target position the joint is moving to or
	 has already reached.
    	@param number of the joint
    	*/
    	virtual double getJointTarget(int jointno){
        	return manualCommands[jointno];
    	};

    /** prints the current joint configuration as a vector of reals € [-1,1] and as a vector of degrees
        only available in manual control mode
    */
    virtual void printJointConf();

    /**moves a joint to a desired position, only available in manual Control Mode
     @param jointno the number of the joint to be moved
     @param pos the desired position of the joint. Must be in [-1,1]
    */
    virtual void moveJoint(int jointno, double pos);

    /** returns a pointer to the endeffector
    */
    virtual Primitive* getEndeffector(){
        return endeffector;
    }
	  /** returns a pointer to the gripper (the uppermost primitive)
    */
	virtual Primitive* getGripper(){
	        return gripper;
    }
    /** prints the position of the endeffector
    */
    virtual void printEndeffectorPosition(){
        //The LpZ Position
	std::cout<< "Lpz Endeffector Position: \t";
        Pos p = endeffector->getPosition();
	p.print();
	}

 /** prints the pose of the endeffector
    */
	virtual void printEndeffectorPose(){
		std::cout << "Pose des Endeffektors: "<< endeffector->getPose()<< "\n";
	}

/* sets the reference of the first relative-distance-sensor to the given object
		--> afterwards sensors 15-17 will return the relative distances of the endeffector to the object*/
	virtual void setReference(Primitive* target){
		relSensors[0]->setReference(target);
	};

//the following methods are for a specific simulation and have to be used with care
// they require the KDL orocos files


    /* calculate the inverse Kinematics
	@params the desired position in a KDL::Frame
	@params the KDL::JntArray where the joint conf should be written
	@return returns 0 if everything was ok, -3 if max iterations exceeded (desired position could be not reachable)
    */
    //virtual int getIK(KDL::Frame pos_goal, KDL::JntArray* q_target);

	/** will move the endeffector to the desired position
	by transforming the Cartesian Position into a joint configuration
	and writing this jnt conf into the manualCommands array
	@param the desired position and Pose in a 4x4 KDL-Frame
		3x3 as rotation 3x position
		1x4 irrelevant  1
	you can use the KDL::Frame( Vector )-Constructor for zero rotation
	*/
	//virtual void moveEndeffectorTo(KDL::Frame);

	/** moves the arm over an object
	@param the Main primitive of the target object
	*/
	//virtual void moveOverObject(Primitive* targetObj);

	//virtual void moveTowardsObject(Primitive* targetObj);

	/** generate a random move using the IK Solver
	*/
	//virtual void randomMove();

	/* prints the current joint configuration and the cartesian position of
	the endeffector calculated by the orocos-KDL bib
	*/
	//virtual void jntToPos();

	/* moves the endeffector in neg direction along one axis using the IK solver
		 1/-1 move in positve/negative x direction
		 2/-2 move in positve/negative y direction
		 3/-3 move in positve/negative z direction
	*/
	//virtual void moveAlongAxis(int axis);

	/** returns the orientation that minimizes the distance between the TCP and the center of the target object
	@param pos the Position of the endeffector
	@param target the Position of the target
	*/
	//virtual KDL::Rotation getRotation(KDL::Vector pos, KDL::Vector target);

	/** returns the orientation that minimizes the distance between the TCP and the center of the target object
	@param pos the Position of the endeffector
	@param target the Position of the target
	*/
	//virtual KDL::Rotation getRotation(Position pos, Position target);



  protected:
    /** creates vehicle at desired pose
    @param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose);

    /** destroys vehicle and space
     */
    virtual void destroy();


    /** returns the maximal power of a motor
    */
    int getPower( int i);

    /** returns the maximal velocity of a motor
    */
    int getVelocity( int i);


    /** additional things for collision handling can be done here
     */
    static void mycallback(void *data, dGeomID o1, dGeomID o2);

	 bool created;      // true if robot was created
    double length;     //  length of segments
    double width;      //  width of segments
    double cmass;      // mass of segments
	double gripper_radius;	// radius of the gripper
	double socketSize;	//size of the socket -  the lowest segment of the Kuka arm, the segment that is fixed to the environment
	double endeffector_radius;	//the radius of the endeffector


    int sensorno;     	// number of sensors
    int motorno;       	// number of motors
    int segmentsno;    	// number of arm segments
    int jointsno;  	// number of joints
    bool grasped;    	// true if robot holds an object
	bool gripper_active;	// true if gripper tried to grasp something. Will be reset after one simstep.
	double simSize; //will hold the size of the simulation (e.o. Kuka constructor)

	Joint* griff;    	//can be used to grasp objects
    Primitive* endeffector;	//pointer to the endeffector
    Primitive* gripper;		//pointer to the gripper

    bool manualControl;    	//used to block the motorcommands sent by the controller and enable the manual
                		// control mode
    std::vector< double > manualCommands;    //used to store the manually entered motorcommands. the joint destinations have to be given in radians

	 std::vector< OneAxisJoint * > joints;   //list of all joints , specified as OneAxis to ease use of servos
   	 std::vector<double> speed;      		// factor for adjusting speed of robot for each joint individually
         std::vector<double> max_force;  	// maximal force for motors
    	 std::vector <OneAxisServo*> hingeServos;    //Servos for the joints
	 std::vector<Primitive*> objectsOfInterest;		// list of all objects the arm can/must keep track of/ interact/sense distance

	 std::vector< RelativePositionSensor* > relSensors;	//list of all sensors the kuka uses to measure distances

//these

//especially for my bachelor thesis:
	int sensor23;

   /*
	//stuff for the KDL orocos kinematics handling
	KDL::Rotation lastRotation;
	KDL::Vector lastPosition;
	KDL::Chain chain;
	KDL::JntArray lastJointConf;
	std::vector<KDL::Rotation> orientations;	//stores orientations for the e.o. getIK()
*/


  };

}
#endif

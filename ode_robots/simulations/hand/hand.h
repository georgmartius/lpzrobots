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
#ifndef __HAND_H
#define __HAND_H

#include "oderobot.h"
#include <selforg/configurable.h>
#include "primitive.h"
#include "joint.h"
#include "angularmotor.h"

namespace lpzrobots {
  // struct containing geom and body for each beam (= box, (cappped)cylinder, sphere)
  typedef struct {
  public:

    double velocity;
    double power;
    double invert;
    double x;
    double y;
    double z;
    int show_contacts;
  } HandConf;

  struct Beam{
    dBodyID body;
    dGeomID geom;
  };

  // beams used in simulation
  // _b, _c and _t indicate parts of the finger:
  // _b -> bottom, part of the finger connected to the palm
  // _c -> center, center part of the finger
  // _t -> top, top (or tip) of the finger
  enum{ 
    forearm,
      palm,
      thumb_b, thumb_t,
      index_b, index_c, index_t,
      middle_b, middle_c, middle_t,
      ring_b, ring_c, ring_t,
      little_b, little_c, little_t,
      number_beams
      };

  // joints used in simulation
  // _b, _c and _t indicate parts of the finger:
  // _b -> bottom, part of the finger connected to the palm
  // _c -> center, center part of the finger
  // _t -> top, top (or tip) of the finger
  enum{ 
    forearm_palm,
      palm_thumb, thumb_bt,
      palm_index, index_bc, index_ct,
      palm_middle, middle_bc, middle_ct,
      palm_ring, ring_bc, ring_ct,
      palm_little, little_bc, little_ct,
      number_joints
      };

  // joints for fixing the four fingers (index, middle, ring, liddle)
  // relativ to each other (at the bottom finger element)
  // to achieve prosthetic like motion
  enum{
    index_middle,
      middle_ring,
      ring_little,
      number_fix_joints
      };

  enum GripMode{
    lateral,
    precision
  };

  /**
   * Artificial Hand 
   * 
   */
  class Hand : public OdeRobot{
  public:

    Hand(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const HandConf& conf, const std::string& name);
    //  Hand(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const string& name);


    static HandConf getDefaultConf()
    {
      HandConf conf;
      conf.velocity = 0.2;
      conf.power = 0.5;
      conf.invert = 1; 
      conf.x =0;
      conf.y =0;
      conf.z =2;	
      conf.show_contacts=1;
      return conf;
    }

    /**
     * Constructor
     */
  
    /// update the subcomponents
    virtual void update();

    /** sets the pose of the vehicle
	@param pose desired 4x4 pose matrix
    */
    virtual void place(const osg::Matrix& pose);

    /** checks for internal collisions and treats them. 
     *  In case of a treatment return true (collision will be ignored by other objects and the default routine)
     *  else false (collision is passed to other objects and (if not treated) to the default routine).
     */
    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);
    /** this function is called in each timestep. It should perform robot-internal checks, 
	like space-internal collision detection, sensor resets/update etc.
	@param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(const GlobalData& globalData);
  

    /** returns actual sensorvalues
	@param sensors sensors scaled to [-1,1] 
	@param sensornumber length of the sensor array
	@return number of actually written sensors
    */
    virtual int getSensors(sensor* sensors, int sensornumber);

    /** sets actual motorcommands
	@param motors motors scaled to [-1,1] 
	@param motornumber length of the motor array
    */
    virtual void setMotors(const motor* motors, int motornumber);

    /** returns number of sensors
     */
    virtual int getSensorNumber();

    /** returns number of motors
     */
    virtual int getMotorNumber();

    /** returns a vector with the positions of all segments of the robot
	@param poslist vector of positions (of all robot segments) 
	@return length of the list
    */
    //  virtual int getSegmentsPosition(vector<Position> &poslist);
  
    /** The list of all parameters with there value as allocated lists.       
     */
    virtual paramlist getParamList() const;
  
    virtual paramval getParam(const paramkey& key) const;

    virtual bool setParam(const paramkey& key, paramval val);

  protected:
    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const {
      if(!objects.empty()){
	//      int half = objects.size()/2;
	//      return (objects[half]);
	return (objects[1]);
      }else return 0;
    }


  private:

    /** creates vehicle at desired pose
	@param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose); 

    /** destroys robot and space
     */
    virtual void destroy();

    static void mycallback(void *data, dGeomID o1, dGeomID o2);
       
    bool created;      // true if robot was created

  protected:

    HandConf conf;
    std::vector <Primitive*> objects;
    //objects.reserve(number_beams);
    //static Primitive*  draw_contact;
    std::vector <Joint*> joints;

    std::vector <AngularMotor*> frictionmotors;
    Primitive* p;
    Joint* j;

    //Beam beam[number_beams]; // array of elements (rectangle and cylinders)
    dSpaceID hand_space;     // space containing the hand
    //dJointID joint[number_joints];  // array containg "normal" joints for connecting the elementsconf

    //dJointID fix_joint[number_fix_joints]; //joints for keeping index, middle, ring and little finger together to achieve mor prosthetic like motion

    // two motorjoints for actuating the two ball joints (forearm_palm and palm_thumb)
    AngularMotor* palm_motor_joint;
    AngularMotor* thumb_motor_joint;

    Joint* fix_forearm_joint; //joint connecting forearm with simulation environment



    double thumb1, thumb2, thumb3;

    GripMode gripmode; //for handling lateral and precision grip modes




    Position initial_pos;    // initial position of robot

    int NUM;	   /* number of beats */
    double MASS;	   /* mass of a beats */
    double RADIUS;   /* sphere radius */

    // Joint* joint[10];
    // Primitive* object[10];

    Pos oldp;

    int sensorno;
    int motorno;
    int sensor_number;
    paramval factorForce;
    paramval frictionGround;
    paramval factorSensor;
    double velocity;
    double power;    
  public:


    static double palm_torque;
    static double finger_force;

  };

}

#endif
 

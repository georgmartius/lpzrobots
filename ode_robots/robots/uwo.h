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
 *   Revision 1.5  2007-11-07 13:21:16  martius
 *   doInternal stuff changed signature
 *
 *   Revision 1.4  2007/02/13 19:32:13  martius
 *   twoaxisservo
 *
 *   Revision 1.3  2006/12/21 11:43:05  martius
 *   commenting style for doxygen //< -> ///<
 *   new sensors for spherical robots
 *
 *   Revision 1.2  2006/07/14 12:23:42  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.2  2006/06/25 16:57:17  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.1.2.1  2006/06/10 20:13:33  martius
 *   unknown walking object
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __UWO_H
#define __UWO_H

#include "oderobot.h"
#include "twoaxisservo.h"

namespace lpzrobots {

  class Primitive; 
  class Joint;

  typedef struct {
  public:
    double size;       ///< scaling factor for robot (diameter of body)
    double legLength;  ///< length of the legs in units of size 
    int    legNumber;  ///<  number of snake elements
    bool   radialLegs; ///< joint orientation is radial instead of cartesian
    double mass;       ///< chassis mass
    double relLegmass; ///< relative overall leg mass
    double jointLimit; ///< angle range for legs
    double motorPower; ///< maximal force for motors
    double frictionGround; ///< friction with the ground
  } UwoConf;


  /** UWO: Unknown Walk Object :-), looks like a plate with a lot of legs 
   */
  class Uwo : public OdeRobot {
  public:
  
    /**
     * constructor of uwo robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param size scaling of robot
     * @param force maximal used force to realize motorcommand
     * @param radialLegs switches between cartensian and radial leg joints 
     */
    Uwo(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const UwoConf& conf, 
	const std::string& name);

    virtual ~Uwo(){};

    static UwoConf getDefaultConf(){
      UwoConf c;
      c.size       = 1;
      c.legNumber  = 8;
      c.legLength  = 0.3;
      c.mass       = 1;
      c.relLegmass = 1;
      c.motorPower = 0.5;
      c.jointLimit = M_PI/12; // +- 15 degree
      c.radialLegs = true;
      c.frictionGround = 1;
      return c;
    }

    /**
     * updates the OSG nodes of the vehicle
     */
    virtual void update();


    /** sets the pose of the vehicle
	@param pose desired pose matrix
    */
    virtual void place(const osg::Matrix& pose);

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
    virtual int getSensorNumber(){
      return conf.legNumber*2;
    };

    /** returns number of motors
     */
    virtual int getMotorNumber(){
      return conf.legNumber*2;
    };

    /** checks for internal collisions and treats them. 
     *  In case of a treatment return true (collision will be ignored by other objects 
     *  and the default routine)  else false (collision is passed to other objects and 
     *  (if not treated) to the default routine).
     */
    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2);

    /** this function is called in each timestep. It should perform robot-internal checks, 
	like space-internal collision detection, sensor resets/update etc.
	@param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(GlobalData& globalData);

    
    /** The list of all parameters with there value as allocated lists.
     */
    virtual paramlist getParamList() const;
    
    virtual paramval getParam(const paramkey& key) const;;
    
    virtual bool setParam(const paramkey& key, paramval val);


  protected:
    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return objects[0]; }

    /** creates vehicle at desired pose
	@param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose); 

    /** destroys vehicle and space
     */
    virtual void destroy();

    UwoConf conf; 
    double legmass;    // leg mass

    bool created;      // true if robot was created

    std::vector<Primitive*> objects;  // 1 body, legs
    std::vector<Joint*> joints; // joints between cylinder and each legs
    std::vector <UniversalServo*> servos; // motors

  };

}

#endif

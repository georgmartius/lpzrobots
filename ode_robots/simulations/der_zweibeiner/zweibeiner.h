/***************************************************************************
 *   Copyright (C) 2007 by Robot Group Leipzig                             *
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
 *   Revision 1.2  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.1  2007/10/29 12:43:59  robot3
 *   MAIN WITH DIFFERET OBJECTS AND DERCONTROLLER
 *
 *   Revision 1.1  2007/07/17 07:25:27  martius
 *   first attempt to build a two legged robot (humanoid)
 *
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __ZWEIBEINER_H
#define __ZWEIBEINER_H

#include "oderobot.h"

namespace lpzrobots {

  class Primitive;
  class Joint;
  class OneAxisServo;
  class TwoAxisServo;

  typedef struct {
  public:
    double size;       ///< scaling factor for robot (height)
    double bodyMass;   ///< chassis(body) mass
    double relLegmass; ///< relative overall leg mass
    double relArmmass; ///< relative overall arm mass
    double relFeetmass; ///< relative overall feet mass
    double hipPower;   ///< maximal force for at hip joint motors
    double hipDamping; ///< damping of hip joint servos
    double hipJointLimit; ///< angle range for legs
    double kneePower;  ///< spring strength in the knees
    double kneeDamping; ///< damping in the knees
    double kneeJointLimit; ///< angle range for knees
    double anklePower;  ///< spring strength in the ankles
    double ankleDamping; ///< damping in the ankles
    double ankleJointLimit; ///< angle range for ankles
    double armPower;   ///< maximal force for at arm (shoulder) joint motors
    double armDamping; ///< damping of arm ((shoulder)) joint servos
    double armJointLimit; ///< angle range of arm joint
    double hip2Power;   ///< maximal force for at hip2 (sagital joint axis) joint motors
    double hip2Damping; ///< damping of hip2 joint servos
    double hip2JointLimit; ///< angle range for hip joint in lateral direction
    double pelvisPower;   ///< maximal force for at pelvis joint motor
    double pelvisDamping; ///< damping of pelvis joint servo
    double pelvisJointLimit; ///< angle range of pelvis joint

    bool onlyPrimaryFunctions; ///< true: only leg and arm are controlable, false: all joints

  } ZweiBeinerConf;


  /** should look like a dog
   */
  class ZweiBeiner : public OdeRobot {
  public:

    /**
     * constructor of ZweiBeiner robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param conf configuration object
     */
    ZweiBeiner(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const ZweiBeinerConf& conf,
               const std::string& name);

    virtual ~ZweiBeiner(){};

    static ZweiBeinerConf getDefaultConf(){
      ZweiBeinerConf c;
      c.size       = 1;
      c.bodyMass   = 1;
      c.relLegmass = 1;
      c.relFeetmass = 0.1;
      c.relArmmass = 0.3;
      c.hipPower = 3;
      c.hipDamping = 0.1;
      c.kneePower  = 2;
      c.kneeDamping = 0.05;
      c.anklePower  = 0.5;
      c.ankleDamping = 0.02;
      c.hipJointLimit = M_PI/2; // +- 90 degree
      c.kneeJointLimit = M_PI/4; // +- 45 degree
      c.ankleJointLimit = M_PI/4; // +- 45 degree

      c.armPower  = 0.5;
      c.armDamping = 0.02;
      c.armJointLimit = M_PI/4; // +- 45 degree

      c.hip2Power = 3;
      c.hip2Damping = 0.1;
      c.hip2JointLimit = M_PI/20; // +- 9 degree

      c.pelvisPower  = 0.5;
      c.pelvisDamping = 0.02;
      c.pelvisJointLimit = M_PI/20; // +- 9degree

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
    virtual int getSensorNumber();

    /** returns number of motors
     */
    virtual int getMotorNumber();
    /** checks for internal collisions and treats them.
     *  In case of a treatment return true (collision will be ignored by other objects
     *  and the default routine)  else false (collision is passed to other objects and
     *  (if not treated) to the default routine).
     */
    virtual bool collisionCallback(void *data, dGeomID o1, dGeomID o2) {return false;}

    /** this function is called in each timestep. It should perform robot-internal checks,
        like space-internal collision detection, sensor resets/update etc.
        @param globalData structure that contains global data from the simulation environment
    */
    virtual void doInternalStuff(const GlobalData& globalData);


    /** The list of all parameters with there value as allocated lists.
     */
    virtual paramlist getParamList() const;

    virtual paramval getParam(const paramkey& key, bool traverseChildren=true) const;;

    virtual bool setParam(const paramkey& key, paramval val, bool traverseChildren=true);

    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return objects[0]; }
  protected:

    /** creates vehicle at desired pose
        @param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose);

    /** destroys vehicle and space
     */
    virtual void destroy();

    ZweiBeinerConf conf;

    bool created;      // true if robot was created

    // some objects explicitly needed for ignored collision pairs
    Primitive *rightfeed, *leftfeed, *rightlowerleg, *leftlowerleg;
    std::vector<Primitive*> objects;  // all the objects
    std::vector<Joint*> joints; // joints legs
    std::vector <TwoAxisServo*> hipservos; // motors
    std::vector <OneAxisServo*> kneeservos; // motors
    std::vector <OneAxisServo*> ankleservos; // motors
    std::vector <OneAxisServo*> armservos; // motors

    std::vector <OneAxisServo*> pelvisservos; // (is only one)

  };

}

#endif

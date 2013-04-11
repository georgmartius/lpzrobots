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
 *   Revision 1.1  2007/07/17 07:25:26  martius
 *   first attempt to build a two legged robot (humanoid)
 *
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __SKELETON_H
#define __SKELETON_H

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

  } SkeletonConf;


  /** should look like a dog
   */
  class Skeleton : public OdeRobot {
  public:

    /**
     * constructor of Skeleton robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param conf configuration object
     */
    Skeleton(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const SkeletonConf& conf,
               const std::string& name);

    virtual ~Skeleton(){};

    static SkeletonConf getDefaultConf(){
      SkeletonConf c;
      c.size       = 1;
      c.bodyMass   = 1;
      c.relLegmass = 1;
      c.relFeetmass = 0.1;
      c.relArmmass = 0.3;

      c.hipPower=0;//50;
      c.hipDamping=0.4;
      c.hip2Power=0;//50;
      c.hip2Damping=0.4;
      c.hip2JointLimit=0.05;

      c.kneePower=40;
      c.kneeDamping=0.2;

      c.anklePower=10;
      c.ankleDamping=0.05;

      c.armPower=20;
      c.armDamping=0.1;

      c.pelvisPower=200;
      c.pelvisDamping=0.5;

      c.hipJointLimit = M_PI/2; // +- 90 degree
      c.kneeJointLimit = M_PI/4; // +- 45 degree
      c.ankleJointLimit = M_PI/4; // +- 45 degree

      c.armJointLimit = M_PI/4; // +- 45 degree

      c.hip2JointLimit = M_PI/30; // +- 6 degree
      c.pelvisJointLimit = M_PI/30; // +- 6 degree

      c.onlyPrimaryFunctions=false;
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
    virtual Primitive* getMainPrimitive() const { return objects[Trunk_comp]; }
  protected:

    /** creates vehicle at desired pose
        @param pose 4x4 pose matrix
    */
    virtual void create(const osg::Matrix& pose);

    /** destroys vehicle and space
     */
    virtual void destroy();

    SkeletonConf conf;

    bool created;      // true if robot was created

    typedef enum SkelParts { Hip,Trunk_comp,Neck, Head_comp,
                             Left_Shoulder, Left_Forearm, Left_Hand,
                             Right_Shoulder, Right_Forearm, Right_Hand,
                             Left_Thigh, Left_Shin, Left_Foot,
                             Right_Thigh, Right_Shin, Right_Foot,
                             LastPart };
    std::vector<Primitive*>    objects;  // all the objects
    std::vector<Joint*>        joints; // joints legs
    std::vector<TwoAxisServo*> hipservos; // motors
    std::vector<OneAxisServo*> kneeservos; // motors
    std::vector<OneAxisServo*> ankleservos; // motors
    std::vector<TwoAxisServo*> armservos; // motors
    std::vector<OneAxisServo*> headservos; // motors

    TwoAxisServo* pelvisservo; // between Hip and Trunk_comp

  };

}

#endif

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
 *   Revision 1.5  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.4  2010/11/05 13:54:05  martius
 *   store and restore for robots implemented
 *
 *   Revision 1.3  2009/11/26 14:21:54  der
 *   Larger changes
 *   :wq
 *
 *   wq
 *
 *   Revision 1.2  2009/08/12 10:30:25  der
 *   skeleton has belly joint
 *   works fine with centered servos
 *
 *   Revision 1.1  2009/08/10 15:00:46  der
 *   version that Ralf did at home
 *   Skeleton bugfixing, works now fine with ServoVel
 *
 *   Revision 1.12  2009/08/09 20:21:03  der
 *   From PC home
 *
 *   Revision 1.11  2009/05/11 17:01:20  martius
 *   new velocity servos implemented
 *   reorganized parameters, now also neck and elbows are configurable
 *
 *   Revision 1.10  2009/01/20 17:29:52  martius
 *   cvs commit
 *
 *   Revision 1.9  2008/11/14 11:23:05  martius
 *   added centered Servos! This is useful for highly nonequal min max values
 *   skeleton has now also a joint in the back
 *
 *   Revision 1.8  2008/06/20 14:03:01  guettler
 *   reckturner
 *
 *   Revision 1.7  2008/05/27 13:25:12  guettler
 *   powerfactor moved to skeleton
 *
 *   Revision 1.6  2008/05/27 10:35:03  guettler
 *   include corrected
 *
 *   Revision 1.5  2008/04/10 12:27:32  der
 *   just a few changes
 *
 *   Revision 1.4  2008/03/14 08:04:23  der
 *   Some changes in main and skeleton (with new outfit)
 *
 *   Revision 1.3  2008/02/08 13:35:10  der
 *   satelite teaching
 *
 *   Revision 1.2  2008/02/07 14:25:02  der
 *   added setTexture and setColor for skeleton
 *
 *   Revision 1.1  2008/01/29 09:52:16  der
 *   first version
 *
 *   Revision 1.2  2007/11/07 13:27:28  martius
 *   doInternalstuff changed
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

#include <ode_robots/oderobot.h>
#include <ode_robots/raysensorbank.h>

namespace lpzrobots {

  class Primitive;
  class Joint;
  class OneAxisServo;
  class TwoAxisServo;
  class AngularMotor;


  typedef struct {
  public:
    double size;       ///< scaling factor for robot (height)
    double massfactor; ///< mass factor for all parts

    bool   useVelocityServos; ///< if true the more stable velocity controlling servos are used

    double relLegmass; ///< relative overall leg mass
    double relArmmass; ///< relative overall arm mass
    double relFeetmass; ///< relative overall feet mass
    double hipPower;   ///< maximal force for at hip joint motors
    double hipDamping; ///< damping of hip joint servos
    double hipVelocity; ///< velocity of hip joint servos
    double hipJointLimit; ///< angle range for legs
    double hip2Power;   ///< maximal force for at hip2 (sagital joint axis) joint motors
    double hip2Damping; ///< damping of hip2 joint servos
    double hip2JointLimit; ///< angle range for hip joint in lateral direction
    double neckPower;  ///< spring strength in the neck
    double neckDamping; ///< damping in the neck
    double neckVelocity; ///< velocity in the neck
    double neckJointLimit; ///< angle range for neck
    double kneePower;  ///< spring strength in the knees
    double kneeDamping; ///< damping in the knees
    double kneeVelocity; ///< velocity in the knees
    double kneeJointLimit; ///< angle range for knees
    double anklePower;  ///< spring strength in the ankles
    double ankleDamping; ///< damping in the ankles
    double ankleVelocity; ///< velocity in the ankles
    double ankleJointLimit; ///< angle range for ankles
    double armPower;   ///< maximal force for at arm (shoulder) joint motors
    double armDamping; ///< damping of arm ((shoulder)) joint servos
    double armVelocity; ///< velocity of arm ((shoulder)) joint servos
    double armJointLimit; ///< angle range of arm joint
    double elbowPower;   ///< maximal force for at elbow (shoulder) joint motors
    double elbowDamping; ///< damping of elbow ((shoulder)) joint servos
    double elbowVelocity; ///< velocity of elbow ((shoulder)) joint servos
    double elbowJointLimit; ///< angle range of elbow joint
    double pelvisPower;   ///< maximal force for at pelvis joint motor
    double pelvisDamping; ///< damping of pelvis joint servo
    double pelvisVelocity; ///< velocity of pelvis joint servo
    double pelvisJointLimit; ///< angle range of pelvis joint
    double backPower;   ///< maximal force for at back joint motor
    double backDamping; ///< damping of back joint servo
    double backVelocity; ///< velocity of back joint servo
    double backJointLimit; ///< angle range of back joint

    double powerfactor; ///< scale factor for maximal forces of the servos
    double dampingfactor; ///< scale factor for damping of the servos

    double jointLimitFactor; ///< factor between servo range (XXXJointLimit, see above) and physical joint limit


    bool onlyPrimaryFunctions; ///< true: only leg and arm are controlable, false: all joints
    bool handsRotating; ///< hands are attached with a ball joint

    bool movableHead;  ///< if false then no neck movement

    bool useBackJoint; ///< whether to use the joint in the back

    bool irSensors; ///< whether to use the irsensor eyes

    Color headColor;
    Color bodyColor;
    Color trunkColor;
    Color handColor;


    std::string headTexture; // texture of the head
    std::string bodyTexture; // texture of the body
    std::string trunkTexture; // texture of the trunk and thorax

  } SkeletonConf;


  /** should look like a dog
   */
  class Skeleton : public OdeRobot {
  public:

    enum SkelParts {Pole,Pole2, Hip,Trunk_comp, Belly, Thorax, Neck, Head_trans, Head_comp,
                             Left_Shoulder, Left_Forearm, Left_Hand,
                             Right_Shoulder, Right_Forearm, Right_Hand,
                             Left_Thigh, Left_Shin, Left_Foot,
                             Right_Thigh, Right_Shin, Right_Foot,
                             LastPart };


    /**
     * constructor of Skeleton robot
     * @param odeHandle data structure for accessing ODE
     * @param osgHandle ata structure for accessing OSG
     * @param conf configuration object
     */
    Skeleton(const OdeHandle& odeHandle, const OsgHandle& osgHandle, SkeletonConf& conf,
               const std::string& name);

    virtual ~Skeleton(){};

    static SkeletonConf getDefaultConf(){
      SkeletonConf c;
      c.size       = 1;
      c.massfactor = 1;
      c.relLegmass = 1;   // unused
      c.relFeetmass = 1;// .1; unused
      c.relArmmass = 1;// 0.3; unused

      c.useVelocityServos = false;
      c.powerfactor=1.0;
      c.dampingfactor=1.0;
      c.jointLimitFactor=1.0;

      c.hipPower=100;
      c.hipDamping= 0.2;
      c.hipVelocity=20;

      c.hip2Power=100;
      c.hip2Damping=0.2;

      c.neckPower=20;
      c.neckDamping=0.1;
      c.neckVelocity=20;

      c.kneePower=60;
      c.kneeDamping=0.1;
      c.kneeVelocity=20;

      c.anklePower=30;
      c.ankleDamping=0.1;
      c.ankleVelocity=20;

      c.armPower=40;
      c.armDamping=0.1;
      c.armVelocity=20;

      c.elbowPower=30;
      c.elbowDamping=0.1;
      c.elbowVelocity=20;

      c.pelvisPower=200;
      c.pelvisDamping=0.2;
      c.pelvisVelocity=20;

      c.backPower=200;
      c.backDamping=0.1;
      c.backVelocity=20;

      c.hipJointLimit = 2.2;
      c.hip2JointLimit= 1.4;
      c.kneeJointLimit = 2.8; // + 300, -20 degree
      c.ankleJointLimit = M_PI/2; // - 90 + 45 degree

      c.armJointLimit = M_PI/2; // +- 90 degree
      c.elbowJointLimit = 2.4;

      c.hip2JointLimit = M_PI/30; // +- 6 degree
      c.pelvisJointLimit = M_PI/10; // +- 18 degree

      c.neckJointLimit = M_PI/5;
      c.backJointLimit = M_PI/3;//4// // +- 60 degree (half of it to the back)

      c.onlyPrimaryFunctions=false;
      c.handsRotating = false;
      c.movableHead   = false;
      c.useBackJoint  = true;
      c.irSensors  = false;

      //      c.headTexture="Images/really_white.rgb";
      c.headTexture="Images/dusty.rgb";
      c.headColor=Color(255/255.0, 219/255.0, 119/255.0, 1.0f);
      //  c.bodyTexture="Images/whitemetal_farbig_small.rgb";
      c.bodyTexture="Images/dusty.rgb";
      c.bodyColor=Color(207/255.0, 199/255.0, 139/255.0, 1.0f);
      c.trunkTexture="Images/dusty.rgb";//"Images/whitemetal_farbig_small.rgb";
      c.trunkColor=Color(207/255.0, 199/255.0, 139/255.0, 1.0f);
      c.handColor=Color(247.0/255, 182.0/255,52.0/255, 1.0f);
      return c;
    }

    static SkeletonConf getDefaultConfVelServos(){
      SkeletonConf c = getDefaultConf();

      c.useVelocityServos = true;
      c.dampingfactor=0.02;
/*       c.hipDamping= 0.01; */
/*       c.hip2Damping=0.01; */
/*       c.neckDamping=0.01; */
/*       c.kneeDamping=0.01; */
/*       c.ankleDamping=0.01; */
/*       c.armDamping=0.01; */
/*       c.elbowDamping=0.01; */
/*       c.pelvisDamping=0.01; */
/*       c.backDamping=0.01; */
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
    virtual void doInternalStuff(GlobalData& globalData);



    virtual bool setParam(const paramkey& key, paramval val, bool traverseChildren=true);

    /** the main object of the robot, which is used for position and speed tracking */
    virtual Primitive* getMainPrimitive() const { return objects[Trunk_comp]; }

    /** all parts of the robot */
    virtual std::vector<Primitive*>& getPrimitives() { return objects; }

    /** returns the position of the head */
    virtual Position getHeadPosition();

    /** returns the position of the trunk */
    virtual Position getTrunkPosition();

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

    std::vector<Primitive*>    objects;  // all the objects
    std::vector<Joint*>        joints; // joints legs
    std::vector<TwoAxisServo*> hipservos; // motors
    std::vector<OneAxisServo*> kneeservos; // motors
    std::vector<OneAxisServo*> ankleservos; // motors
    std::vector<TwoAxisServo*> armservos; // motors
    std::vector<OneAxisServo*> arm1servos; // motors
/*     std::vector<OneAxisServo*> headservos; // motors */
    std::vector<TwoAxisServo*> headservos; // motors

    OneAxisServo* pelvisservo; // between Hip and Trunk_comp
    std::vector<OneAxisServo*> backservos;   // between Trunk_comp and Thorax
    //TwoAxisServo* backservo;   // between Trunk_comp and Thorax

    std::vector<AngularMotor*> frictionmotors;

    RaySensorBank irSensorBank;

  };

}

#endif

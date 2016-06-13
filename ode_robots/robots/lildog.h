#ifndef __LILDOG_H
#define __LILDOG_H

#include <ode_robots/oderobot.h>
#include <selforg/inspectable.h>
#include <ode_robots/contactsensor.h>
//-------------Add by Ren relativepositionsensor.h-------
#include <ode_robots/relativepositionsensor.h>
#include "lildogsensormotordefinition.h"
#include <ode_robots/axisorientationsensor.h>

/**
 * forward declarations
 */
namespace lpzrobots {
  class HingeJoint;
  class IRSensor;
  class Joint;
  class OneAxisServo;
  class Primitive;
  class RaySensorBank;
  class SliderJoint;
  class SpeedSensor;
  class Spring;
  class TwoAxisServo;
 }

 namespace lpzrobots {
 	struct LilDogConf{
 	  /** fix the shoulder element to the trunk. */
      bool useShoulder;
      /** whether to use joints at the knees or fix them */
      bool useHip;
      /** use spring foot */
      bool useFoot;

      bool rubberFeet;
      /** decide whether you wand to use a local velocity sensors.
       *  If yes it gets velocity vector in local coordinates and pass it as
       *  sensorvalues */
      bool useLocalVelSensor;
      /** Use binary leg contact sensors. If false, a force sensor is used. */
      bool legContactSensorIsBinary;
      /**@}*/

      /** scaling factor for robot (length of body) */
      double size;
      /** trunk width */
      double width;
      /** trunk height */
      double height;
      /** radius of a wheel */
      double wheel_radius;
      /** width of a wheel */
      double wheel_width;
      /** mass of a wheel */
      double wheel_mass;
      /** trunk mass */
      double trunkMass;
      // front leg definition
      double shoulderMass;
      /** mass of the coxa limbs */
      double humerusMass;
      /** mass of the second limbs */
      double forearmMass;
      /** mass of the feet */
      // hind leg definition
      double hipMass;
      /** mass of the coxa limbs */
      double femurMass;
      /** mass of the second limbs */
      double tibiaMass;
      /** mass of the feet */
      double footMass;
      
      /** fix legs to trunk at this distance from bottom of trunk */
      double shoulderHeight;
      
      double hipHeight;

      /** distance between hindlegs and front legs */
      double legdist;
            /** @name Leg extension from trunk
       *
       *  II has a fixed but adjustable joint that decides how the legs
       *  extend from the trunk.here you can adjust these joints, if
       *  shoulder = 0 this still influences how the legs extend (the coxa-trunk
       *  connection in that case)
       */
      /**@{*/
      /** angle in rad around vertical axis at leg-trunk fixation for front
       *  legs*/
      double fLegTrunkAngleV;
      /** angle in rad around horizontal axis at leg-trunk fixation for front
       *  legs */
      double fLegTrunkAngleH;
      /** rotation of front legs around own axis */
      double fLegRotAngle;
      /** angle in rad around vertical axis at leg-trunk fixation for middle
       *  legs */
      double rLegTrunkAngleV;
      /** angle in rad around horizontal axis at leg-trunk fixation for rear
       *  legs */
      double rLegTrunkAngleH;
      /** rotation of rear legs around own axis */
      double rLegRotAngle;
      /**
       * @name leg part dimensions
       *
       * the lengths and radii of the individual leg parts
       */
      /**@{*/
      //-----------FRONT LEGS--------
      //shoulder
      double shoulderLength;
      double shoulderRadius;
      //humerus
      double humerusLength;
      double humerusRadius;
      //forearm
      double forearmLength;
      double forearmRadius;
	 //-----------REAR LEGS--------
      //hip
      double hipLength;
      double hipRadius;
      //femur
      double femurLength;
      double femurRadius;
      //tibia
      double tibiaLength;
      double tibiaRadius;

      double footRange;
      /** radius of the foot capsules, choose different from tebiaRadius */
      double footRadius;
      /**
       * @name Joint Limits
       *
       * set limits for each joint
       */
      /**{*/
     //-----------FRONT LEGS UP--------
      //shoulder 
      double shoulderJoint1LimitU;
      double shoulderJoint2LimitF;
      //elbow
      double elbowJointLimitU;
      //-----------REAR LEGS UP--------
      //hip
      double hipJoint1LimitU;
      double hipJoint2LimitF;
      //knee
      double kneeJointLimitU;
     //-----------FRONT LEGS DOWN--------
      //shoulder 
      double shoulderJoint1LimitD;
      double shoulderJoint2LimitB;
      //elbow
      double elbowJointLimitD;
     //-----------REAR LEGS DOWN--------
      //hip
      double hipJoint1LimitD;
      double hipJoint2LimitB;
      //knee
      double kneeJointLimitD;
       /** preload of the foot spring */
      double footSpringPreload;
      /** upper limit of the foot spring = maximum value
       *  (negative is downwards (spring extends)) */
      double footSpringLimitU;
      /** lower limit of the foot spring = minimum value
       *  (negative is downwards (spring extends)) */
      double footSpringLimitD;

      /** maximal force of the shoulder1 joint servos */
      double shoulder1Power;
      /** maximal force of the shoulder2 joint servos */
      double shoulder2Power;
      /** maximal force of the elbow joint servos */
      double elbowPower;
      /** maximal force of the hip1 joint servos */
      double hip1Power;
      /** maximal force of the hip2 joint servos */
      double hip2Power;
      /** maximal force of the knee joint servos */
      double kneePower;
      /** maximal force of the foot spring servos */
      double footPower;

      /** damping of the shoulder1 joint servos */
      double shoulder1Damping;
      /** damping of the shoulder2 joint servos */
      double shoulder2Damping;
      /** damping of the elbow joint servo */
      double elbowDamping;
      /** damping of the hip1 joint servos */
      double hip1Damping;
      /** damping of the hip2 joint servos */
      double hip2Damping;
      /** damping of the tibia joint servo */
      double tibiaDamping;
      /** damping of the foot spring servo */
      double footDamping;
       double MaxVel;
      double shoulder1MaxVel ;
    double shoulder2MaxVel ;
    double elbowMaxVel ;
    double hip1MaxVel ;
    double hip2MaxVel;
    double kneeMaxVel ;
    double footMaxVel ;

      /** path to texture for legs */
      std::string texture;
      /** path to texture for trunk */
      std::string bodyTexture;

      //-----------Add GoalSensor by Ren------------------------
      std::vector<Primitive*> GoalSensor_references;
      //-----------Add GoalSensor by Ren------------------------


      // Internal variable storing the currently used version
      int lildog_version;

      /***
       * Author: Subhi Shaker Barikhan
       * Date:03.06.2014
       * highFootContactsensoryFeedback=true ==> the threshold  of foot contact sensor signal is 4
       * otherwise is 1
       */
      bool highFootContactsensoryFeedback;
	};

	class LilDog : public OdeRobot, public Inspectable {
    public:
      enum LegPos {
        L0, L1, R0, R1, LEG_POS_MAX
      };
      enum LegPosUsage {
        LEG, WHEEL, UNUSED
      };
      enum LegJointType {
        // shoulder joint1 for forward (+) and backward (-) movements
        SHO1,
        // shoulder joint2 for elevation (+) and depression (-) of
        // the leg
        SHO2,
        // elbow joints for extension (+) and flexion (-) 
        ELB,

        LEG_JOINT_TYPE_MAX
      };
      typedef LilDogMotorNames MotorName;
      typedef LilDogSensorNames SensorName;

      /**
       * Returns the default configuration values
       */
      static LilDogConf getDefaultConf(double _scale = 1.0, bool _useShoulder = 1,bool _useHip = 1, bool _useFoot = 1,bool _highFootContactsensoryFeedback=false);

      static LilDogConf getLilDogConf(double _scale = 1.0, bool _useShoulder = 1,bool _useHip = 1, bool _useFoot = 1,bool _highFootContactsensoryFeedback=false);



      /*
       * Coordinated locomotion between two hexapod robots
       * */

      Primitive* getTibiaPrimitive(LegPos leg);
      Primitive* getShoulderPrimitive(LegPos leg);

      /**
       * constructor
       * @param odeHandle data structure for accessing ODE
       * @param osgHandle ata structure for accessing OSG
       * @param conf configuration object
       * @param name name to display for this robot
       */
      LilDog(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const LilDogConf& conf = getDefaultConf(),
          const std::string& name = "LilDog robot");

      virtual ~LilDog();

      /**
       * updates the OSG nodes of the vehicle
       */
      virtual void update();

      /**
       * sets the pose of the vehicle
       * @param pose desired pose matrix
       */
      virtual void placeIntern(const osg::Matrix& pose);

      /**
       * returns actual sensorvalues
       * @param sensors sensor array with sensors scaled to [-1,1]
       * @param sensornumber length of the sensor array
       * @return number of actually written sensors
       */
      virtual int getSensorsIntern(sensor* sensors, int sensornumber);

      /**
       * sets actual motorcommands
       * @param motors motors scaled to [-1,1]
       * @param motornumber length of the motor array
       */
      virtual void setMotorsIntern(const double* motors, int motornumber);

      /**
       * returns number of sensors
       */
      virtual int getSensorNumberIntern();

      /**
       * returns number of motors
       */
      virtual int getMotorNumberIntern();

      /**
       * this function is called in each timestep. It should perform
       * robot-internal checks,like space-internal collision detection, sensor
       * resets/update etc.
       * @param globalData structure that contains global data from the
       *                   simulation environment
       */
      virtual void doInternalStuff(GlobalData& globalData);

      virtual void sense(GlobalData& globalData) override;

      virtual double getMassOfRobot();

      void setLegPosUsage(LegPos leg, LegPosUsage usage);

      // Configurable Interface
      virtual bool setParam(const paramkey& key, paramval val);

      /**
       * the main object of the robot, which is used for position and speed
       * tracking
       */
      virtual Primitive* getMainPrimitive() const;

      /**
       * returns the MotorName enum value for the given joint at the given
       * leg. If the value for leg or joint are not valid dungBeetle_MOTOR_MAX
       * is returned.
       *
       * @param leg leg position
       * @param joint leg joint type
       * @return the motor name value or dungBeetle_MOTOR_MAX if parameters are
       *         invalid
       */
      static MotorName getMotorName(LegPos leg, LegJointType joint);

      /**
       * Returns the joint type of the given motor. If the given motor name
       * is not associated with a leg joint JOINT_TYPE_MAX is returend and a
       * warning is given out.
       *
       * @param MotorName name of the motor
       * @return joint type controlled by this motor or JOINT_TYPE_MAX if
       *         MotorName is invalid
       */
      static LegJointType getLegJointType(MotorName);

      /**
       * Returns the leg of the given motor. If the given motor name is not
       * associated wit a leg LEG_POS_MAX is returned and a warning is given
       * out.
       *
       * @param MotorName name of the motor
       * @return the leg on which this motor operates or LEG_POS_MAX if
       *         MotorName is invalid
       */
      static LegPos getMotorLegPos(MotorName);

    protected:

      struct Leg {
          Leg();
          HingeJoint * shoulderJoint1;
          HingeJoint * shoulderJoint2;
          HingeJoint * elbowJoint;
          HingeJoint * hipJoint1;
          HingeJoint * hipJoint2;
          HingeJoint * kneeJoint;
        /*Slider*/Joint * footJoint;
          OneAxisServo * shoulder1Servo;
          OneAxisServo * shoulder2Servo;
          OneAxisServo * elbowServo;
          OneAxisServo * hip1Servo;
          OneAxisServo * hip2Servo;
          OneAxisServo * kneeServo;
          Spring * footSpring;
          Primitive * shoulder;
          Primitive * humerus;
          Primitive * forearm;
          Primitive * hip;
          Primitive * femur;
          Primitive * tibia;
          Primitive * foot;
         };

              /**
       * creates vehicle at desired pose
       *
       * @param pose 4x4 pose matrix
       */
      virtual void create(const osg::Matrix& pose);

      /**
       * destroys vehicle and space
       */
      virtual void destroy();

      /**
       * Assign a human readable name to a motor. This name is used for the
       * associated inspectable value as used e.g. in guilogger.
       *
       * @param motorNo index of the motor (for standard motors defined by
       *        the MotorName enum)
       *
       * @param name human readable name for the motor
       */
      void nameMotor(const int motorNo, const char* name);

      /**
       * Assign a human readable name to a sensor. This name is used for the
       * associated inspectable value as used e.g. in guilogger.
       *
       * @param motorNo index of the motor (for standard motors defined by
       *        the SensorName enum)
       * @param name human readable name for the sensor
       */
      void nameSensor(const int sensorNo, const char* name);

    private:

      /** typedefs */
      typedef std::map<LegPos, HingeJoint*> HingeJointMap;
      typedef std::map<LegPos, Leg> LegMap;
      typedef std::map<LegPos, ContactSensor*> LegContactMap;
      typedef std::map<MotorName, OneAxisServo*> MotorMap;
      typedef std::map<LegPos, LegPosUsage> LegPosUsageMap;
      typedef std::vector<Primitive*> PrimitiveList;
      typedef std::vector<Joint*> JointList;
      typedef std::vector<OneAxisServo*> ServoList;

      //-----------Add Orientation Sensor by Ren----------------
      AxisOrientationSensor* OrientationSensor;
      //-----------Add Orientation Sensor by Ren----------------

      LilDogConf conf;
      bool created; // true if robot was created

      /** speed sensor */
      SpeedSensor * speedsensor;


      /**
       * statistics
       * theses values are updated in every timestep and have to be updated
       * to make them available to the lpzrobots::Inspectable interface
       */
      /** position of the robot */
      Position position;

      /**
       * defines for everey leg position the way it is used (e.g  place
       * a leg or a wheel at this position)
       */
      LegPosUsageMap legPosUsage;

      LegContactMap legContactSensors;

      // body in case of no hinge joint being used
      Primitive *trunk;

      // information on all legs
      LegMap legs;


      // passive servos without a Motorname
      ServoList passiveServos;

      // contains all active servos
      MotorMap servos;

      //---------------Add GoalSensor by Ren---------------
      std::vector<RelativePositionSensor> GoalSensor; // Relative position sensors

      bool GoalSensor_active;
      //---------------Add GoalSensor by Ren---------------



      };

 }
#endif
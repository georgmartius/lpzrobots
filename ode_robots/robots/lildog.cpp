
//#define VERBOSE
#include "lildog.h"
#include <cmath>
#include <assert.h>

// #include <ode/ode.h>
#include <ode-dbl/ode.h>

// include primitives (box, spheres, cylinders ...)
#include <ode_robots/primitive.h>

// include sensors
#include <ode_robots/speedsensor.h>

// include joints
#include <ode_robots/joint.h>
#include <ode_robots/oneaxisservo.h>
#include <ode_robots/twoaxisservo.h>
#include <ode_robots/spring.h>

#include <ode_robots/mathutils.h>

// include header file

// rotation and translation matrixes (to make the code shorter)
#define ROTM osg::Matrix::rotate
#define TRANSM osg::Matrix::translate

namespace lpzrobots {

  LilDog::Leg::Leg() {
    shoulderJoint1 = 0;
    shoulderJoint2 = 0;
    elbowJoint = 0;
    hipJoint1 = 0;
    hipJoint2 = 0;
    kneeJoint = 0;
    footJoint = 0;
    shoulder1Servo= 0;
	shoulder2Servo= 0;
	elbowServo= 0;
	hip1Servo= 0;
	hip2Servo= 0;
	kneeServo= 0;
    footSpring = 0;
    shoulder = 0;
  	humerus = 0;
  	forearm = 0;
	femur = 0;
	tibia = 0;
	foot = 0;
	}

 LilDog::LilDog(const OdeHandle& odeHandle, const OsgHandle& osgHandle, const LilDogConf& c, const std::string& name) :
          OdeRobot(odeHandle, osgHandle, name, "LilDog"), conf(c) {
          	    legPosUsage[L0] = LEG;
    			legPosUsage[L1] = LEG;
    			legPosUsage[R0] = LEG;
    			legPosUsage[R1] = LEG;

    // robot is not created till now
    			created = false;
    			speedsensor = 0;

    			addParameter("shouler1Power", &conf.shoulder1Power);
    			addParameter("shouler2Power", &conf.shoulder2Power);
    			addParameter("elbowPower", &conf.elbowPower);
    			addParameter("hip1Power", &conf.hip1Power);
    			addParameter("hip2Power", &conf.hip2Power);
    		
 				addParameter("shouler1Damp", &conf.shoulder1Damping);
    			addParameter("shouler2Damp", &conf.shoulder2Damping);
    			addParameter("elbowDamp", &conf.elbowDamping);
    			addParameter("hip1Damp", &conf.hip1Damping);
    			addParameter("hip2Damp", &conf.hip2Damping);
    			

    			addParameter("shoulderJoint1LimitU", &conf.shoulderJoint1LimitU);
    			addParameter("shoulderJoint2LimitF", &conf.shoulderJoint2LimitF);
    			addParameter("shoulderJoint1LimitD", &conf.shoulderJoint1LimitD);
    			addParameter("shoulderJoint2LimitB", &conf.shoulderJoint2LimitB);
    			addParameter("elbowJointLimitU", &conf.elbowJointLimitU);
    			addParameter("elbowJointLimitD", &conf.elbowJointLimitD);
    			//addParameter("secondJointLimitD", &conf.secondJointLimitD);
    			//addParameter("secondJointLimitU", &conf.secondJointLimitU);
    			addParameter("hipJoint1LimitU", &conf.hipJoint1LimitU);
    			addParameter("hipJoint2LimitF", &conf.hipJoint2LimitF);
    			addParameter("hipJoint1LimitD", &conf.hipJoint1LimitD);
    			addParameter("hipJoint2LimitB", &conf.hipJoint2LimitB);
    			addParameter("kneeJointLimitU", &conf.kneeJointLimitU);
    			addParameter("kneeJointLimitD", &conf.kneeJointLimitD);


    			nameSensor(S1R0_as, "*SH1R0 angle sensor");
    			nameSensor(S2R0_as, "*SH2R0 angle sensor");
    			nameSensor(ELR0_as, "*ELR0 angle sensor");
    			nameSensor(S1L0_as, "*SH1L0 angle sensor");
    			nameSensor(S2L0_as, "*SH2L0 angle sensor");
    			nameSensor(ELL0_as, "*ELL0 angle sensor");
    			nameSensor(S1R1_as, "*H1R1 angle sensor");
    			nameSensor(S2R1_as, "*H2R1 angle sensor");
    			nameSensor(S1L1_as, "*H1L1 angle sensor");
    			nameSensor(S2L1_as, "*H2L1 angle sensor");
    			nameSensor(ELR1_as, "*KNR1 angle sensor");
    			nameSensor(ELL1_as, "*KNL1 angle sensor");

    			nameSensor(R0_fs, "*R0 foot contact sensor");
    			nameSensor(R1_fs, "*R1 foot contact sensor");
 				nameSensor(L0_fs, "*L0 foot contact sensor");
    			nameSensor(L1_fs, "*L1 foot contact sensor"); 

    			// name the motors
    			nameMotor(S1R0_m, "TR0 motor");
    			nameMotor(S2R0_m, "TR1 motor");
    			nameMotor(ELR0_m, "TR2 motor");
    			nameMotor(S1L0_m, "TL0 motor");
    			nameMotor(S2L0_m, "TL1 motor");
    			nameMotor(ELL0_m, "TL2 motor");
    			nameMotor(S1R1_m, "CR0 motor");
    			nameMotor(S2R1_m, "CR1 motor");
    			nameMotor(ELR1_m, "CR2 motor");
    			nameMotor(S1L1_m, "CL0 motor");
    			nameMotor(S2L1_m, "CL1 motor");
    			nameMotor(ELL1_m, "CL2 motor");			 
 
          }

  LilDog::~LilDog() {
    destroy();
  }

 int LilDog::getMotorNumberIntern(){
  	return LILDOG_MOTOR_MAX;
  }
  ;
  /**
   * Assign a human readable name to a sensor. This name is used for the
   * associated inspectable value as used e.g. in guilogger.
   *
   * @param motorNo index of the motor (for standard motors defined by
   *        the SensorName enum)
   * @param name human readable name for the sensor
   */
void LilDog::nameSensor(const int sensorNo, const char* name) {
#ifdef VERBOSE
    std::cerr << "LilDog::nameSensor BEGIN\n";
#endif
    addInspectableDescription("x[" + std::itos(sensorNo) + "]", name);
#ifdef VERBOSE
    std::cerr << "LilDog::nameSensor END\n";
#endif
  }
  

 /**
   * Assign a human readable name to a motor. This name is used for the
   * associated inspectable value as used e.g. in guilogger.
   *
   * @param motorNo index of the motor (for standard motors defined by
   *        the MotorName enum)
   * @param name human readable name for the motor
   */
  void LilDog::nameMotor(const int motorNo, const char* name) {
#ifdef VERBOSE
    std::cerr << "LilDog::nameMotor BEGIN\n";
#endif
    addInspectableDescription("y[" + std::itos(motorNo) + "]", name);
#ifdef VERBOSE
    std::cerr << "LilDog::nameMotor END\n";
#endif
  }

  /* sets actual motorcommands
   @param motors motors scaled to [-1,1]
   @param motornumber length of the motor array
   */
  void LilDog::setMotorsIntern(const double* motors, int motornumber) {
#ifdef VERBOSE
    std::cerr << "LilDog::setMotors BEGIN\n";
#endif
    assert(created);
    // robot must exist
    assert(motornumber >= getMotorNumberIntern());
    for (MotorMap::iterator it = servos.begin(); it != servos.end(); it++) {
      MotorName const name = it->first;
      OneAxisServo * const servo = it->second;
      //We multiple with -1 to map to real hexapod
      if (servo)
        servo->set(-motors[name]);
    }
#ifdef VERBOSE
    std::cerr << "LilDog::setMotors END\n";
#endif
  }
  ;

  int LilDog::getSensorNumberIntern() {
#ifdef VERBOSE
    std::cerr << "LilDog::getSensorNumberIntern BEGIN\n";
#endif
#ifdef VERBOSE
    std::cerr << "LilDog::getSensorNumberIntern END\n";
#endif
    return LILDOG_SENSOR_MAX;
  }
  ;


/* returns actual sensorvalues
   @param sensors sensors scaled to [-1,1] (more or less)
   @param sensornumber length of the sensor array
   @return number of actually written sensors
   */
  int LilDog::getSensorsIntern(double* sensors, int sensornumber) {
#ifdef VERBOSE
    std::cerr << "LilDog::getSensors BEGIN\n";
#endif
    assert(created);
    assert(sensornumber >= getSensorNumberIntern());

    // angle sensors
    //We multiple with -1 to map to real hexapod
    sensors[S1R0_as] = servos[S1R0_m] ? -servos[S1R0_m]->get() : 0;
    sensors[S2R0_as] = servos[S2R0_m] ? -servos[S1R0_m]->get() : 0;
    sensors[ELR0_as] = servos[ELR0_m] ? -servos[ELR0_m]->get() : 0;
    sensors[S1L0_as] = servos[S1L0_m] ? -servos[S1L0_m]->get() : 0;
    sensors[S2L0_as] = servos[S2L0_m] ? -servos[S2L0_m]->get() : 0;
    sensors[ELL0_as] = servos[ELL0_m] ? -servos[ELL0_m]->get() : 0;
    sensors[S1R1_as] = servos[S1R1_m] ? -servos[S1R1_m]->get() : 0;
    sensors[S2R1_as] = servos[S2R1_m] ? -servos[S2R1_m]->get() : 0;
    sensors[ELR1_as] = servos[ELR1_m] ? -servos[ELR1_m]->get() : 0;
    sensors[S1L1_as] = servos[S1L1_m] ? -servos[S1L1_m]->get() : 0;
    sensors[S2L1_as] = servos[S2L1_m] ? -servos[S2L1_m]->get() : 0;
    sensors[ELL1_as] = servos[ELL1_m] ? -servos[ELL1_m]->get() : 0;
 if (conf.legContactSensorIsBinary) { // No scaling since binary signals are already in the range of [0,..,1]
      sensors[R0_fs] = legContactSensors[R0] ? legContactSensors[R0]->get() : 0;
      sensors[R1_fs] = legContactSensors[R1] ? legContactSensors[R1]->get() : 0;
      
      sensors[L0_fs] = legContactSensors[L0] ? legContactSensors[L0]->get() : 0;
      sensors[L1_fs] = legContactSensors[L1] ? legContactSensors[L1]->get() : 0;
      
    } else { // Scaling since analog signals are used then we scale them to the range of [0,..,1]
      // Koh! Georg: What are the different values
      std::vector<double> max, min;

        // Koh Corrected to have all equal max force in all legs
         max.push_back(0.2);
         max.push_back(0.2);
         max.push_back(0.2);
         max.push_back(0.2);
         max.push_back(0.2);
         max.push_back(0.2);
         min.push_back(0.0);
         min.push_back(0.0);
         min.push_back(0.0);
         min.push_back(0.0);
         min.push_back(0.0);
         min.push_back(0.0);

       /* max.push_back(0.16);
        max.push_back(0.20);
        max.push_back(0.14);
        max.push_back(0.24);
        max.push_back(0.20);
        max.push_back(0.14);
        min.push_back(0.0);
        min.push_back(0.0);
        min.push_back(0.0);
        min.push_back(0.0);
        min.push_back(0.0);
        min.push_back(0.0);*/
      // Georg: this normalization does not make sense to me.
      sensors[R0_fs] =
          legContactSensors[R0] ? ((legContactSensors[R0]->get() - min.at(0)) / (max.at(0) - min.at(0))) : 0;
      sensors[R1_fs] =
          legContactSensors[R1] ? ((legContactSensors[R1]->get() - min.at(1)) / (max.at(1) - min.at(1))) : 0;
     
      sensors[L0_fs] =
          legContactSensors[L0] ? ((legContactSensors[L0]->get() - min.at(3)) / (max.at(3) - min.at(3))) : 0;
      sensors[L1_fs] =
          legContactSensors[L1] ? ((legContactSensors[L1]->get() - min.at(4)) / (max.at(4) - min.at(4))) : 0;
      
      // Koh! Georg: overwrite the rescaling
      double footContactFactor = conf.highFootContactsensoryFeedback ? 4.0 : 1.0;

      sensors[R0_fs] = legContactSensors[R0]->get()*footContactFactor;
      sensors[R1_fs] = legContactSensors[R1]->get()*footContactFactor;
     
      sensors[L0_fs] = legContactSensors[L0]->get()*footContactFactor;
      sensors[L1_fs] = legContactSensors[L1]->get()*footContactFactor;
      

      // Koh! Georg: I added this as a factor above
      // if (conf.highFootContactsensoryFeedback)
      // {
      //        for (int i = R0_fs; i <= L2_fs; i++) {
      //      	  if (sensors[i] > 4.0)
      //      		  sensors[i] = 4.0;
      //        }
      // }
      // else
      // {
      //        for (int i = R0_fs; i <= L2_fs; i++) {
      //      	  if (sensors[i] > 1.0)
      //      		  sensors[i] = 1.0;
      //        }
      // }

    }

  	        // Body speed sensors
    sensor speedsens[3] = { 0, 0, 0 };


    #ifdef VERBOSE
    std::cerr << "LilDog::getSensors END\n";
#endif
    return LILDOG_SENSOR_MAX;
  }
  ;

  void LilDog::placeIntern(const osg::Matrix& pose) {
#ifdef VERBOSE
    std::cerr << "LilDog::place BEGIN\n";
#endif
    // the position of the robot is the center of the body
    // to set the vehicle on the ground when the z component of the position
    // is 0
    //Matrix p2 = pose * ROTM(0, 0, conf.legLength + conf.legLength/8);
    osg::Matrix p = pose
        * TRANSM(0, 0, conf.forearmLength - conf.shoulderHeight + 2 * conf.tibiaRadius + conf.footRadius);
    create(p);
#ifdef VERBOSE
    std::cerr << "LilDog::place END\n";
#endif
  }
  ;
  /**
   * updates the osg notes
   */
  void LilDog::update() {
    OdeRobot::update();
#ifdef VERBOSE
    std::cerr << "LilDog::update BEGIN\n";
#endif
    assert(created);
    // robot must exist

    for (int i = 0; i < LEG_POS_MAX; i++) {
      if (legContactSensors[LegPos(i)])
        legContactSensors[LegPos(i)]->update();
    }

#ifdef VERBOSE
    std::cerr << "LilDog::update END\n";
#endif
  }
  ;

  double LilDog::getMassOfRobot() {

    double totalMass = 0.0;

    for (unsigned int i = 0; i < objects.size(); i++) {
      dMass massOfobject;
      dBodyGetMass(objects[i]->getBody(), &massOfobject);
      totalMass += massOfobject.mass;
    }
    return totalMass;
  }

    void LilDog::sense(GlobalData& globalData) {
    OdeRobot::sense(globalData);

    for (int i = 0; i < LEG_POS_MAX; i++) {
      if (legContactSensors[LegPos(i)])
        legContactSensors[LegPos(i)]->sense(globalData);
    }

  }

    /**
   * this function is called in each timestep. It should perform robot-
   * internal checks, like space-internal collision detection, sensor
   * resets/update etc.
   *
   * @param global structure that contains global data from the simulation
   * environment
   */
  void LilDog::doInternalStuff(GlobalData& global) {

#ifdef VERBOSE
    std::cerr << "LilDog::doInternalStuff BEGIN\n";
#endif
    OdeRobot::doInternalStuff(global);
    // update statistics
    position = getPosition();

    // passive servos have to be set to zero in every time step so they work
    // as springs
    for (ServoList::iterator it = passiveServos.begin(); it != passiveServos.end(); it++) {
      (*it)->set(0.0);
    }

#ifdef VERBOSE
    std::cerr << "LilDog::doInternalStuff END\n";
#endif
  }

  Primitive* LilDog::getMainPrimitive() const {
    return trunk;
  }

 /**
   * creates vehicle at desired position
   *
   * @param pos struct Position with desired position
   */
void LilDog::create(const osg::Matrix& pose) {
#ifdef VERBOSE
    std::cerr << "LilDog::create BEGIN\n";
#endif
    assert(!created); // cannot be recreated, use relocation via moveToPose or store and restore

    // we want legs colliding with other legs, so we set internal collision
    // flag to "false".
    odeHandle.createNewSimpleSpace(parentspace, false);

    // color of robot
    osgHandle = osgHandle.changeColor("robot1");

    // color of joint axis
    OsgHandle osgHandleJoint = osgHandle.changeColor("joint");

    // change Material substance
    OdeHandle odeHandleBody = odeHandle;
    odeHandleBody.substance.toMetal(3.0);

    //get a representation of the origin
    const Pos nullpos(0, 0, 0);
    /**********************************************************************/
    /*  create body                                                       */
    /**********************************************************************/

    /** central position of the trunk */
    const osg::Matrix trunkPos = pose;

	  trunk = new Box(conf.size, conf.width, conf.height);
      trunk->setTexture(conf.bodyTexture);
      trunk->init(odeHandleBody, conf.trunkMass, osgHandle.changeColor("robot3"));
      trunk->setPose(trunkPos);
      objects.push_back(trunk);

    

    if (conf.useLocalVelSensor) {
      // create speedsensor
      speedsensor = new SpeedSensor(1.0, SpeedSensor::TranslationalRel, SpeedSensor::XYZ);
      //initialize the speedsensor
      speedsensor->init(trunk);
    }

 /************************************
     * LEGS
     ***********************************/

    const osg::Matrix m0 = pose;

    const double l0 = conf.shoulderLength ;
    const double t0 = conf.shoulderRadius;
    const double l1 = conf.humerusLength;
    const double t1 = conf.humerusRadius;
    const double l2 = conf.forearmLength;
    const double t2 = conf.forearmRadius;
    const double l3 = conf.tibiaLength - 2 * conf.tibiaRadius - conf.footRange;
    const double t3 = conf.tibiaRadius;
    const double l4 = 2 * conf.tibiaRadius + conf.footRange - conf.footRadius;
    const double t4 = conf.footRadius;

    std::map<LegPos, osg::Matrix> legtrunkconnections;
    std::map<LegPos, osg::Matrix> shouldertrunkconnections;

    for (int i = 0; i < LEG_POS_MAX; i++) {
      LegPos leg = LegPos(i);

      // +1 for L1,L2,L3, -1 for R1,R2,R3
      //const double lr = (leg == L0 || leg == L1 || leg == L2) - (leg == R0 || leg == R1 || leg == R2);
      const double lr = (leg == L0 || leg == L1 )- (leg == R0 || leg == R1 );
      // create 3d-coordinates for the leg-trunk connection:

      Pos pos = Pos(
          // from (0,0,0) we go down x-axis, make two legs then up
          // legdist1 and so on
          -conf.size * 10 / 43.0
           + (leg == L1 || leg == R1) * 0 + (leg == L0 || leg == R0) * (conf.legdist),
          // switch left or right side of trunk for each leg
          lr * conf.width / 2, // /2
          // height of leg fixation to trunk (trunk bottom sits at
          // total legLength)
          -conf.height / 2 + conf.shoulderHeight*0.4);//SET HERE TRUNK HEIGHT....GIULIANO 0.2


      // get a coordinate system at the position pos by rotating such that
      // z-axis points toward trunk, pose is where the robot will be
      // placed so we begin there.
      legtrunkconnections[leg] = ROTM(M_PI/2, lr, 0, 0) * TRANSM(pos) * pose;

      // we create a transformation matrix that represents the
      // transformation from the trunk center to the trunk-shoulder
      // connections. we need it because we need the coordinates relative
      // to the trunk to create one body including the shoulders
        shouldertrunkconnections[leg] = ROTM(M_PI/2, lr, 0, 0) * TRANSM(pos);
    }



    // if wanted, leg trunk connections are rotated here:
    legtrunkconnections[R1] = ROTM(conf.rLegRotAngle, 0, 0, 1) * ROTM(conf.rLegTrunkAngleH, 1, 0, 0)
            * ROTM(conf.rLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[R1];
    legtrunkconnections[L1] = ROTM(conf.rLegRotAngle, 0, 0, -1) * ROTM(conf.rLegTrunkAngleH, -1, 0, 0)
            * ROTM(conf.rLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[L1];
    legtrunkconnections[R0] = ROTM(conf.fLegRotAngle, 0, 0, 1) * ROTM(conf.fLegTrunkAngleH, 1, 0, 0)
            * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[R0];
    legtrunkconnections[L0] = ROTM(conf.fLegRotAngle, 0, 0, -1) * ROTM(conf.fLegTrunkAngleH, -1, 0, 0)
            * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[L0];
//    legtrunkconnections[R0] = ROTM(conf.fLegRotAngle, 0, 0, 1) * ROTM(conf.fLegTrunkAngleH, 1, 0, 0)
//            * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[R0];
//    legtrunkconnections[L0] = ROTM(conf.fLegRotAngle, 0, 0, -1) * ROTM(conf.fLegTrunkAngleH, -1, 0, 0)
//            * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * legtrunkconnections[L0];

    // also the relative coordinates for the shoulders
    shouldertrunkconnections[R1] = ROTM(conf.rLegRotAngle, 0, 0, 1) * ROTM(conf.rLegTrunkAngleH, 1, 0, 0)
            * ROTM(conf.rLegTrunkAngleV, 0, 1, 0) * shouldertrunkconnections[R1];
    shouldertrunkconnections[L1] = ROTM(conf.rLegRotAngle, 0, 0, -1) * ROTM(conf.rLegTrunkAngleH, -1, 0, 0)
            * ROTM(conf.rLegTrunkAngleV, 0, 1, 0) * shouldertrunkconnections[L1];
    shouldertrunkconnections[R0] = ROTM(conf.fLegRotAngle, 0, 0, 1) * ROTM(conf.fLegTrunkAngleH, 1, 0, 0)
            * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * shouldertrunkconnections[R0];
    shouldertrunkconnections[L0] = ROTM(conf.fLegRotAngle, 0, 0, -1) * ROTM(conf.fLegTrunkAngleH, -1, 0, 0)
            * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * shouldertrunkconnections[L0];
//    shouldertrunkconnections[R0] = ROTM(conf.fLegRotAngle, 0, 0, 1) * ROTM(conf.fLegTrunkAngleH, 1, 0, 0)
//            * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * shouldertrunkconnections[R0];
//    shouldertrunkconnections[L0] = ROTM(conf.fLegRotAngle, 0, 0, -1) * ROTM(conf.fLegTrunkAngleH, -1, 0, 0)
//            * ROTM(conf.fLegTrunkAngleV, 0, 1, 0) * shouldertrunkconnections[L0];





    // create the legs
    for (int i = 0; i < LEG_POS_MAX; i++) {
      LegPos leg = LegPos(i);
      if (legPosUsage[leg] == LEG) {
        // get a representation of the origin
        const Pos nullpos(0, 0, 0);

        // +1 for R1,R2,R3, -1 for L1,L2,L3
        const double pmrl = (leg == R0 || leg == R1 ) - (leg == L0 || leg == L1);



        osg::Matrix c1;

        // m0 is the position where the center of mass of the zeroth limb
        // capsule is placed
        osg::Matrix m0;

       if (conf.useShoulder) {
          //shift connection of coxa outward
          c1 = TRANSM(0, 0, -l0) * legtrunkconnections[leg];
          //create shoulder
          Primitive * should = new Capsule(t0, l0);
          should->setTexture(conf.texture);
          // add shoulder to trunk body
          // the shoulder's pose has to be given relative to the trunk's pose
          // add the first four shoulders to center the other two to front
          Primitive * trans = new Transform(trunk, should,
              TRANSM(0,0, -l0 / 2) * shouldertrunkconnections[leg]);

          trans->init(odeHandle, conf.shoulderMass, osgHandle);
          legs[leg].shoulder = trans;
          objects.push_back(trans);
        } else {
          //first limb data
          c1 = legtrunkconnections[leg];
        }

     osg::Matrix m1 = TRANSM(0, 0, -l0/3) * c1;

        // calculate anchor of the first joint
        const osg::Vec3 anchor1 = nullpos * c1;
        // and it's axis (multiplication with c1 indicates in which
        // (local) coordinate system it is)
        const Axis axis1 = Axis(pmrl, 0, 0) * c1;

        // proceed along the leg (and the respective z-axis) for second
        // limb
        osg::Matrix c2 = ROTM(M_PI/2, pmrl, 0, 0)*TRANSM(0, 0, -l1 / 3) * m1;
        osg::Matrix m2 = TRANSM(0, 0, -l2 / 2) * c2;
        const osg::Vec3 anchor2 = nullpos * c2;
        const Axis axis2 = Axis(0, 1, 0) * c2;

        //and third
        osg::Matrix c3 = TRANSM(0, 0, -l2 / 2) * m2;
        osg::Matrix m3 = TRANSM(0, 0, -l3 / 2) * c3;
        const osg::Vec3 anchor3 = nullpos * c3;
        const Axis axis3 = Axis(0, 1, 0) * c3;

        // now create first limp
        Primitive* shoulder;
        // create upper limp with radius t1 and length l1 (length refers
        // only to length of the cylinder without the semispheres at
        // both ends)
        shoulder = new Capsule(t0, l0);
        shoulder->setTexture(conf.texture);
        shoulder->init(odeHandle, conf.shoulderMass, osgHandle);
        //put it at m1
        shoulder->setPose(m1);
        legs[leg].shoulder = shoulder;
        objects.push_back(shoulder);
                if (conf.useShoulder) {
          odeHandle.addIgnoredPair(legs[leg].shoulder, shoulder);
        }
        // powered hip joint of trunk to first limb
        HingeJoint* j = new HingeJoint(trunk, shoulder, anchor1, -axis1);
        j->init(odeHandle, osgHandleJoint, true, t0 * 2.1);
        joints.push_back(j);
        // create motor, overwrite the jointLimit argument with 1.0
        // because it is less obscure and setMinMax makes mistakes
        // otherwise. Parameters are set later
        OneAxisServo * servo1 = new OneAxisServoVel(odeHandle, j, -1, 1, 1, 0.01, 0, 1.0);
        //PUSH THIS STUFF BACK INTO STH!!!! not hipservos obviously
        legs[leg].shoulder1Servo = servo1;
        servos[getMotorName(leg, SHO1)] = servo1;

        // second limb
        Primitive* humerus;
        humerus = new Capsule(t2, l2);
        humerus->setTexture(conf.texture);
        humerus->init(odeHandle, conf.humerusMass, osgHandle);
        humerus->setPose(m2);
        legs[leg].humerus = humerus;
        objects.push_back(humerus);

        // create the joint from first to second limb (coxa to second)
        HingeJoint* k = new HingeJoint(shoulder, humerus, anchor2, -axis2);
        k->init(odeHandle, osgHandleJoint, true, t1 * 2.1);
        legs[leg].shoulderJoint2 = k;

        joints.push_back(k);
        /** parameters are set later */
        OneAxisServo * servo2 = new OneAxisServoVel(odeHandle, k, -1, 1, 1, 0.01, 0, 1.0);
        //PUSH THIS STUFF BACK INTO STH!!!! not hipservos obviously
        legs[leg].shoulder2Servo = servo2;
        servos[getMotorName(leg, SHO2)] = servo2;

        // third limb
        Primitive* forearm;
        forearm = new Capsule(t3, l3);
        forearm->setTexture(conf.texture);
        forearm->init(odeHandle, conf.forearmMass, osgHandle);
        forearm->setPose(m3);
        //        tebiaPos.push_back(tebia->getPosition());
        legs[leg].forearm = forearm;
        objects.push_back(forearm);

        // springy knee joint
        HingeJoint* l = new HingeJoint(humerus, forearm, anchor3, -axis3);
        l->init(odeHandle, osgHandleJoint, true, t3 * 2.1);
        legs[leg].elbowJoint = l;
        joints.push_back(l);
        // servo used as a spring
        /** parameters are set later */
        OneAxisServo * servo3 = new OneAxisServoVel(odeHandle, l, -1, 1, 1, 0.01, 0, 1.0);
        legs[leg].elbowServo = servo3;
        servos[getMotorName(leg, ELB)] = servo3;

        //spring foot at the end

          OdeHandle my_odeHandle = odeHandle;
		//spring foot at the end
        if (conf.useFoot) {
          osg::Matrix c4 = TRANSM(0, 0, -l3 / 2 - 2 * conf.tibiaRadius - conf.footRange + conf.footRadius) * m3;
          osg::Matrix m4 = TRANSM(0, 0, -conf.footSpringPreload) * c4;

          const osg::Vec3 anchor4 = nullpos * m4;
          const Axis axis4 = Axis(0, 0, -1) * c4;

          OdeHandle my_odeHandle = odeHandle;
          if (conf.rubberFeet) {
            const Substance FootSubstance(3.0, 0.0, 500.0, 0.1);
            my_odeHandle.substance = FootSubstance;
          }

          Primitive* foot;
          foot = new Capsule(t4, l4);
          foot->setTexture(conf.texture);
          foot->init(my_odeHandle, conf.footMass, osgHandle);
          foot->setPose(m4);
          //            footPos.push_back(foot->getPosition());
          legs[leg].foot = foot;
          objects.push_back(foot);

          SliderJoint* m = new SliderJoint(forearm, foot, anchor4, axis4);
          m->init(odeHandle, osgHandleJoint, true, t3, true);
          legs[leg].footJoint = m;
          joints.push_back(m);

          /** parameters are set later */
          Spring* spring = new Spring(m, -1, 1, 1);
          legs[leg].footSpring = spring;
          passiveServos.push_back(spring);
          odeHandle.addIgnoredPair(humerus, foot);

          legContactSensors[LegPos(i)] = new ContactSensor(conf.legContactSensorIsBinary, 50, 1.01 * t4, true);
          legContactSensors[LegPos(i)]->setInitData(odeHandle, osgHandle, TRANSM(0, 0, -0.5 * l4));
          legContactSensors[LegPos(i)]->init(foot);
          odeHandle.addIgnoredPair(forearm, legContactSensors[LegPos(i)]->getTransformObject());
      }
      else if (legPosUsage[leg] == WHEEL) 
      {
        //Sphere* sph = new Sphere(radius);
        Cylinder* wheel = new Cylinder(conf.wheel_radius, conf.wheel_width);
        wheel->setTexture(conf.texture);
        OsgHandle bosghandle = osgHandle;
        wheel->init(odeHandle, conf.wheel_mass, // mass
            bosghandle.changeColor("robot2"));
        const double pmlr = (leg == L0 || leg == L1) - (leg == R0 || leg == R1);
        Pos pos = Pos(
            // from (0,0,0) we go down x-axis, make two legs then up
            // legdist1 and so on
            -conf.size * 16.5 / 43.0 + (leg == L1 || leg == R1) * conf.legdist
            + (leg == L0 || leg == R0) * (conf.legdist),
            // switch left or right side of trunk for each leg
            pmlr * conf.width,
            // height of wheel fixation to trunk
            -0.7 * conf.height + conf.wheel_radius);

        wheel->setPose(ROTM(0.5 * M_PI, 1, 0, 0) * TRANSM(pos) * trunkPos);
        objects.push_back(wheel);
        // generate  joints to connect the wheels to the body
        Pos anchor(dBodyGetPosition(wheel->getBody()));
        anchor -= Pos(0, 0, 0);
        HingeJoint * wheeljoint = new HingeJoint(objects[0], wheel, anchor, Axis(0, 1, 0) * trunkPos);
        wheeljoint->init(odeHandle, osgHandleJoint, true, 1.1 * conf.wheel_width);
        joints.push_back(wheeljoint);
      }
     }
    }


    //-----------------add GoalSensor by Ren------------------------
    if (conf.GoalSensor_references.size()>0)
    {
            // Relative position sensor
            for (std::vector<Primitive*>::iterator it = conf.GoalSensor_references.begin(); it<conf.GoalSensor_references.end();it++)
            {
              // Maxlen 1 and exponent 1 gives exact distances as linear sensor
                    RelativePositionSensor GoalSensor_tmp(1, 1,Sensor::XYZ, true);
                    //max distance for normalization
                    //exponent for sensor characteristic
                    //dimensions to sense
                    //use Z as x-coordinate ( robot was created with vertical capsule or something like that)
                    //local_coordinates
                    GoalSensor_tmp.setReference(*it);
                    GoalSensor.push_back(GoalSensor_tmp);
                    //sensorno += rpos_sens_tmp.getSensorNumber(); // increase sensornumber of robot, have been declared in sensormotordefinition
            }
            GoalSensor_active = true;
    }
    else
    {
            GoalSensor_active =false;
    }
    //----------------------Goal Sensor by Ren-----------------------

    // --------------Add Goal Sensor by Ren -------------------
    // Relative position sensor
    if (GoalSensor_active) {
      for (std::vector<RelativePositionSensor>::iterator it = GoalSensor.begin(); it < GoalSensor.end(); it++) {
        it->init(trunk); // connect sensor to main body
      }
    }
    // --------------Add Goal Sensor by Ren -------------------

    //-----------Add Orientation Sensor by Ren----------------
    OrientationSensor = new AxisOrientationSensor(AxisOrientationSensor::Axis,Sensor::X |Sensor::Y | Sensor::Z);
    OrientationSensor->init(trunk);
    //-----------Add Orientation Sensor by Ren----------------


    setParam("dummy", 0); // apply all parameters.

    created = true;
#ifdef VERBOSE
    std::cerr << "LilDog::create END\n";
#endif
  }
  ;
  /** destroys vehicle and space
   */
  

void LilDog::destroy() {
    if (created) {
#ifdef VERBOSE
      std::cerr << "begin LilDog::destroy\n";
#endif
      // delete contact sensors
      for (int i = 0; i < LEG_POS_MAX; i++) {
        if (legContactSensors[LegPos(i)])
          delete legContactSensors[LegPos(i)];
      }
      legContactSensors.clear();

      // remove all ignored pairs (brute force method)
      for (PrimitiveList::iterator i = objects.begin(); i != objects.end(); i++) {
        for (PrimitiveList::iterator j = objects.begin(); j != objects.end(); j++) {
          odeHandle.removeIgnoredPair(*i,*j);
        }

      }


      if (speedsensor) {
        delete speedsensor;
        speedsensor = 0;
      }

      for (MotorMap::iterator it = servos.begin(); it != servos.end(); it++) {
        if (it->second)
          delete (it->second);
      }
      servos.clear();

      for (ServoList::iterator it = passiveServos.begin(); it != passiveServos.end(); it++) {
        if (*it)
          delete (*it);
      }
      passiveServos.clear();

      cleanup();

      //should all be empty as objects were cleared:
      legs.clear();

      //------------------ delete GoalSensor here by Ren--------------------

      // deleting pointers to GoalSensor_references
      // Georg: you do not want to delete the Goal object from here
      // for (std::vector<Primitive*>::iterator i = conf.GoalSensor_references.begin(); i != conf.GoalSensor_references.end(); i++) {
      //   if (*i)
      //     delete *i;
      // }

      GoalSensor.clear();
      //------------------ delete GoalSensor here by Ren--------------------

      // Added sound sensor (5)// delete sensor
      odeHandle.deleteSpace();
#ifdef VERBOSE
      std::cerr << "end LilDog::destroy\n";
#endif
    }

    created = false;
  }

 bool LilDog::setParam(const paramkey& key, paramval val) {
#ifdef VERBOSE
    std::cerr << "LilDog::setParam BEGIN\n";
#endif
    // the parameters are assigned here
    bool rv = Configurable::setParam(key, val);

    // we simply set all parameters here
    for (LegMap::iterator it = legs.begin(); it != legs.end(); it++) {
      Spring * const footspring = it->second.footSpring;
      if (footspring) {
        footspring->setPower(conf.footPower);
        footspring->setDamping(conf.footDamping);
        footspring->setMaxVel(conf.footMaxVel);
        //yes, min is up, up is negative
        footspring->setMinMax(conf.footSpringLimitD, conf.footSpringLimitU);
      }

      OneAxisServo * shoulder1Servo = it->second.shoulder1Servo;
      if (shoulder1Servo) {
        shoulder1Servo->setPower(conf.shoulder1Power);
        shoulder1Servo->setDamping(conf.shoulder1Damping);
        shoulder1Servo->setMaxVel(conf.shoulder1MaxVel);
        shoulder1Servo->setMinMax(conf.shoulderJoint1LimitU, conf.shoulderJoint1LimitD);
      }

      OneAxisServo * shoulder2Servo = it->second.shoulder2Servo;
      if (shoulder2Servo) {
        shoulder2Servo->setPower(conf.shoulder2Power);
        shoulder2Servo->setDamping(conf.shoulder2Damping);
        shoulder2Servo->setMaxVel(conf.shoulder2MaxVel);
        //yes, min is up, up is negative
        shoulder2Servo->setMinMax(conf.shoulderJoint2LimitF, conf.shoulderJoint2LimitB);
      }

      OneAxisServo * elbowServo = it->second.elbowServo;
      if (elbowServo) {
        elbowServo->setPower(conf.elbowPower);
        elbowServo->setDamping(conf.elbowDamping);
        elbowServo->setMaxVel(conf.elbowMaxVel);
        //yes, min is up, up is negative
        elbowServo->setMinMax(conf.elbowJointLimitU, conf.elbowJointLimitD);
      }
    }

#ifdef VERBOSE
    std::cerr << "LilDog::setParam END\n";
#endif
    return rv;
  }

  /**
   * returns the MotorName enum value for the given joint at the given
   * leg. If the value for leg or joint are not valid AMOSII_MOTOR_MAX
   * is returned.
   *
   * @param leg leg position
   * @param joint leg joint type
   * @return the motor name value or AMOSII_MOTOR_MAX if parameters are
   *         invalid
   */
  LilDog::MotorName LilDog::getMotorName(LegPos leg, LegJointType joint) {
    if (leg == L0 && joint == SHO1)
      return S1L0_m;
    if (leg == L0 && joint == SHO2)
      return S2L0_m;
    if (leg == L0 && joint == ELB)
      return ELL0_m;
    if (leg == L1 && joint == SHO1)
      return S1L1_m;
    if (leg == L1 && joint == SHO2)
      return S2L1_m;
    if (leg == L1 && joint == ELB)
      return ELL1_m;
    if (leg == R0 && joint == SHO1)
      return S1R0_m;
    if (leg == R0 && joint == SHO2)
      return S2R0_m;
    if (leg == R0 && joint == ELB)
      return ELR0_m;
    if (leg == R1 && joint == SHO1)
      return S1R1_m;
    if (leg == R1 && joint == SHO2)
      return S2R1_m;
    if (leg == R1 && joint == ELB)
      return ELR1_m;
   
    return LILDOG_MOTOR_MAX;
  }

  /**
   * returns the joint type of the given motor. If the given motor name
   * is not associated with a leg joint JOINT_TYPE_MAX is returend and a
   * warning is given out.
   *
   * @param MotorName name of the motor
   * @return joint type controlled by this motor or JOINT_TYPE_MAX if
   *         MotorName is invalid
   */
  LilDog::LegJointType LilDog::getLegJointType(MotorName name) {
    assert(name!=LILDOG_MOTOR_MAX);
    switch (name) {
      case S1R0_m:
      case S1L0_m:
      case S1R1_m:
	  case S1L1_m:
     	return SHO1;
      case S2R0_m:
      case S2L0_m:
      case S2R1_m:
	  case S2L1_m:
      	return SHO2;


      case ELR0_m:
      case ELL0_m:
      case ELL1_m:
      case ELR1_m:
  		return ELB;

      default:
        std::cerr << "WARNING: point in LilDog::getMotorJointType reached " << "that should not" << std::endl;
        return LEG_JOINT_TYPE_MAX;
    }
  }

  /**
   * Returns the leg of the given motor. If the given motor name is not
   * associated wit a leg LEG_POS_MAX is returned and a warning is given
   * out
   *
   * @param MotorName name of the motor
   * @return the leg on which this motor operates or LEG_POS_MAX if
   *         MotorName is invalid
   */
  LilDog::LegPos LilDog::getMotorLegPos(MotorName name) {
    assert(name!=LILDOG_MOTOR_MAX);
    switch (name) {
	  case S1R0_m:
	  case S2R0_m:
	  case ELR0_m:
        return R0;
	  case S1R1_m:
	  case S2R1_m:
	  case ELR1_m:
        return R1;
	  case S1L0_m:
	  case S2L0_m:
	  case ELL0_m:
        return L0;
	  case S1L1_m:
	  case S2L1_m:
	  case ELL1_m:
        return L1;
      default:
        std::cerr << "WARNING: point in LilDog::getMotorLegPos reached " << "that should not" << std::endl;
        return LEG_POS_MAX;
    }
  }

  void LilDog::setLegPosUsage(LegPos leg, LegPosUsage usage) {
    legPosUsage[leg] = usage;
  }

  /*
   * Author:Subhi Shaker Barikhan
   * Date: 03.06.2014
   * Coordinated locomotion (two connected robots)
   * **/


  Primitive* LilDog::getShoulderPrimitive(LegPos leg)
  {
	  assert(created);
	  if(leg < LEG_POS_MAX){
		  return legs[leg].shoulder;
	  }else{
		  return 0;
	  }

  }
  /*
   * Author:Subhi Shaker Barikhan
   * Date: 03.06.2014
   * Coordinated locomotion (two connected robots)
   * **/
  Primitive* LilDog::getTibiaPrimitive(LegPos leg)
  {
	  assert(created);
	  if(leg < LEG_POS_MAX){
		  return legs[leg].forearm;
	  }else{
		  return 0;
	  }

  }


 LilDogConf LilDog::getDefaultConf(double _scale, bool _useShoulder, bool _useFoot, bool _useBack,bool _highFootContactsensoryFeedback) {
	  return getLilDogConf(_scale, _useShoulder, _useFoot, _useBack,_highFootContactsensoryFeedback);
  }

  LilDogConf LilDog::getLilDogConf(double _scale, bool _useShoulder, bool _useFoot, bool _useBack, bool _highFootContactsensoryFeedback) {

    LilDogConf c;

    // "Internal" variable storing the currently used version
    c.lildog_version = 2;
    // use shoulder (fixed joint between legs and trunk)
    c.useShoulder = _useShoulder;
    //create springs at the end of the legs
    c.useFoot = _useFoot;
    c.highFootContactsensoryFeedback=_highFootContactsensoryFeedback; // if highFootContactsensoryFeedback is true, then amplitude of foot sensory signal is higher
    c.rubberFeet = false;
    c.useLocalVelSensor = 0;
    c.legContactSensorIsBinary = true;

    // the trunk length. this scales the whole robot! all parts' sizes,
    // masses, and forces will be adapted!!
    c.size = 0.34 * _scale;
    //trunk width
    c.width = 7.0/ 43.0 * c.size;
    //trunk height
    c.height = 6.5 / 43.0 * c.size;
    // we use as density the original trunk weight divided by the original
    // volume

    //Change mass by KOH to 3.0
    const double density = 3.0 / (0.43 * 0.07 * 0.065); //2.2 / (0.43 * 0.07 * 0.065);

    c.trunkMass = density * c.size * c.width * c.height;
    // use the original trunk to total mass ratio
    const double mass = 5.758 / 2.2 * c.trunkMass;
    // distribute the rest of the weight like this for now */
    c.shoulderMass = (mass - c.trunkMass) / (6 * (3.0 + c.useShoulder)) * (20.0 - c.useFoot) / 20.0;
    c.humerusMass = c.shoulderMass;
    c.forearmMass = c.shoulderMass;
   
    c.hipMass = c.shoulderMass;
    c.femurMass = c.shoulderMass;
    c.tibiaMass = c.shoulderMass;
    // foot gets 3 or 4 times 1/20 of shoulderMass (divide and multiply by
    // 3 or 4)
    c.footMass = (mass - c.trunkMass) / 6 * c.useFoot / 20.0;

    //As real robot!!
    const double shoulderHeight_cm = 2.1;
    //shoulder height "4.5 wrong" --> correct=6.5 cm from rotating point
    c.shoulderHeight = shoulderHeight_cm / 2.1 * c.height;

    // distance between hindlegs and middle legs
    c.legdist = 20.0 / 43.0 * c.size;

    // configure the wheels (if used). They don't have any counterpart in
    // reality, so the chosen values are arbitrary
    c.wheel_radius = 0.10 * c.size;
    c.wheel_width = 0.04 * c.size;
    c.wheel_mass = (mass - c.trunkMass) / 6.0;

    // -----------------------
    // 1) Biomechanics
    // Manual setting adjustable joint positions at the body
    // -----------------------

    // amosII has a fixed but adjustable joint that decides how the legs
    // extend from the trunk. Here you can adjust these joints

    // ------------- Front legs -------------
    // angle (in rad) around vertical axis at leg-trunk fixation 0:
    // perpendicular
    // => forward/backward
    c.fLegTrunkAngleV = 0.0;
    // angle around horizontal axis at leg-trunk fixation 0: perpendicular
    // => upward/downward
    c.fLegTrunkAngleH = 0.0;
    // rotation of leg around own axis 0: first joint axis is vertical
    // => till
    c.fLegRotAngle = 0.0;
    // ------------- Rear legs ------------------
    // => forward/backward
    c.rLegTrunkAngleV = 0.0;
    // => upward/downward
    c.rLegTrunkAngleH = 0.0;
    // => till
    c.rLegRotAngle = 0.0;

    // be careful changing the following dimension, they may break the
    // simulation!! (they shouldn't but they do)
    const double shoulderLength_cm = 2.5;
    c.shoulderLength = shoulderLength_cm / 43.0 * c.size;
    c.shoulderRadius = .03 * c.size;

    const double humerusLength_cm = 10.1;
    c.humerusLength = humerusLength_cm / 43.0 * c.size;
    c.humerusRadius = .04 * c.size;

    const double forearmLength_cm = 10.1;
    c.forearmLength = forearmLength_cm / 43.0 * c.size;
    c.forearmRadius = .03 * c.size;

    const double hipLength_cm = 4.5;
    c.hipLength = hipLength_cm / 43.0 * c.size;
    c.hipRadius = .03 * c.size;

    const double femurLength_cm = 3.5;
    c.femurLength = femurLength_cm / 43.0 * c.size;
    c.femurRadius = .04 * c.size;

    const double tibiaLength_cm = 11.5; // 3)
    c.tibiaLength = tibiaLength_cm / 43.0 * c.size;
    c.tibiaRadius = 1.3 / 43.0 * c.size;
    // this determines the limit of the footspring
    c.footRange = .2 / 43.0 * c.size;
    c.footRadius = 1.5 / 43.0 * c.size;

    // -----------------------
    // 2) Joint Limits
    // Setting Max, Min of each joint with respect to real
    // -----------------------

    // 70 deg; forward (-) MAX --> normal walking range 60 deg MAX
    c.shoulderJoint1LimitU = -M_PI / 180.0 *60.0;
    //-70 deg; backward (+) MIN --> normal walking range -10 deg MIN
    c.shoulderJoint1LimitD = M_PI / 180.0 * 60.0;

    //60 deg; forward (-) MAX --> normal walking range 30 deg MAX
    c.shoulderJoint2LimitF = -M_PI / 180.0 * 60.0;
    //60 deg; backward (+) MIN --> normal walking range -40 deg MIN
    c.shoulderJoint2LimitB = M_PI / 180 * 60.0;

    //70 deg; forward (-) MAX --> normal walking range 60 deg MAX
    c.elbowJointLimitU = -M_PI / 180.0 * 70.0;
    //70 deg; backward (+) MIN --> normal walking range -10 deg MIN
    c.elbowJointLimitD = M_PI / 180.0 * 70.0;

    // -----------------------
    // 3) Motors
    // Motor power and joint stiffness
    // -----------------------

    c.footSpringPreload = 8.0 / 43.0 * c.size;
    // negative is downwards (spring extends)
    c.footSpringLimitD = c.footSpringPreload;
    c.footSpringLimitU = c.footSpringPreload + c.footRange;

    const double shoulderPower_scale = 10.0;
    const double springstiffness = 350.0;

    // use an original radius and mass and scale original torque by their
    // new values to keep acceleration constant
    // torque in Nm
    c.shoulder1Power = shoulderPower_scale * (1.962 / (0.035 * 2.2)) * c.shoulderLength * c.trunkMass;
    c.shoulder2Power = c.shoulder1Power;
    c.elbowPower = c.shoulder1Power;

    c.hip1Power = shoulderPower_scale * (1.962 / (0.035 * 2.2)) * c.hipLength * c.trunkMass;
    c.hip2Power = c.shoulder1Power;
    c.kneePower = c.elbowPower;
    // this is the spring constant. To keep  acceleration for the body
    // constant, we use the above unscaled preload of 0.08 and original
    // trunkMass to and then multiply by the new ones
    c.footPower = (springstiffness * 0.08 / 2.2) * c.trunkMass / c.footSpringPreload;

    c.shoulder1Damping = 0.0;
    c.shoulder2Damping = 0.0;
    c.elbowDamping = 0.01;
    c.hip1Damping = 0.0;
    c.hip2Damping = 0.0;
    c.footDamping = 0.05; // a spring has no damping??

    c.MaxVel = 1.7 * 1.961 * M_PI;
    c.shoulder1MaxVel=1.7 * 1.961 * M_PI;
    c.shoulder2MaxVel = 1.7 * 1.961 * M_PI;
    c.elbowMaxVel = 1.7 * 1.961 * M_PI;
    c.hip1MaxVel = 1.7 * 1.961 * M_PI;
    c.hip2MaxVel = 1.7 * 1.961 * M_PI;
    c.kneeMaxVel = 1.7 * 1.961 * M_PI;
    c.footMaxVel = 1.7 * 1.961 * M_PI;

    c.texture = "Images/whiteground.rgb";
    c.bodyTexture = "Images/stripes.rgb";

    //----------------Add GoalSensor by Ren------------------
    c.GoalSensor_references.clear(); //enforce empty vector -> no relative position sensing
    //----------------Add GoalSensor by Ren------------------

    return c;
  }

}
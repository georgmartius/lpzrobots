// Header guard
#ifndef __RUNBOT_H
#define __RUNBOT_H

#include <ode_robots/oderobot.h>
#include <ode_robots/primitive.h>
#include <ode_robots/joint.h>
#include <ode_robots/angularmotor.h>
#include <ode_robots/irsensor.h>
#include <ode_robots/contactsensor.h>
#include <ode_robots/oneaxisservo.h>
#include <ode_robots/oneaxisservoveldisconnected.h>
#include <ode_robots/spring.h>

namespace lpzrobots{
  /**
   * Runbot: Robot with two legs and a movable weight, leaning backwards
   * and forward.
   * Fixated to the side.
   */

  //structure to hold configuration of the robot
  typedef struct{
    public:
      double bodyDepth; //depth of the body
      double bodyWidth; //width of the block on top  of the legs
      double bodyHeight; //height of the block on top of the legs
      double bodyMass; //Mass of the body

      double weightWidth; //width of the block on top  of the legs
      double weightHeight; //height of the block on top of the legs
      double weightMass; //Mass of the body
      double weightDepth; //depth of the weight

      double upperLegLength; //length of the upper leg
      double upperLegThickness; //size of the upper leg in width and depth
      double upperLegMass; //mass of the upper leg

      double lowerLegLength; //length of the lower leg
      double lowerLegThickness; //size of the upper leg in width and depth
      double lowerLegMass; //mass of the upper leg

      double footLength; //length of the feet
      double footWidth; //width of the feet
      double footMass; //mass of the feet
      double footHeight; //height of the feet

      /**
       * joint distances from bottom to top / rear to  front
       */
      double footAnklePos; //position of the ankle on foot
      double legAnklePos; //position of the ankle on lower leg
      double lowerKneePos; //position of the knee on the lower leg
      double upperKneePos; //position of the knee on the upper leg
      double legDistFromCenter; //distance of the legs from the center of the body
      double legHipPos; //height of the hip on the upper leg
      double bodyHipHeight; //height of hip on the body
      double bodyHipDepth; //depth position of the hip join on the body
      double massjointBodyHeight; //height of the joint for the movable weight on the body
      double massjointBodyDepth; //depthposition of the joint for the movable weight on the body
      double massjointMassHeight; //height of the joint on the weight
      double massjointMassDepth; //depth of the joint on the weight

      double armLength; // distance to the stabilizing mass (runbot running around a center)
      double armOffset; // the the position in depth of the 'shoulder'

      double scale; //overall scale of the robot

      //---------------Motor parameters------------------//
      double kneeMotorPower;
      double kneeMotorMin;
      double kneeMotorMax;
      double kneeMotorMaxVel;

      double hipMotorPower;
      double hipMotorMin;
      double hipMotorMax;
      double hipMotorMaxVel;

      double massMotorPower;
      double massMotorMin;
      double massMotorMax;
      double massMotorMaxVel;

      //passive Motors
      double ankleMotorPower;
      double ankleMotorMin;
      double ankleMotorMax;
      double ankleMotorDamping;

      bool invertMotors;
  }RunbotConf;

  class Runbot: public OdeRobot{
    public:
      /**
       * Contructor
       */
      Runbot(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
          const RunbotConf &conf = getDefaultConf(),
          const std::string& name = "Runbot");
      /**
       * Default configuration of the robot
       */
      static RunbotConf getDefaultConf(){
        RunbotConf conf;

        //measured values (lengths) in cm
        //masses in Kg
        conf.bodyWidth = 8.5;
        conf.bodyHeight = 6.5;
        conf.bodyDepth = 8;
        conf.bodyMass = 0.45;

        conf.weightHeight = 2.5;
        conf.weightMass = 0.50;
        conf.weightWidth =  5.0;
        conf.weightDepth = 7.0;

        conf.upperLegLength = 14.2;
        conf.upperLegMass = 0.2;
        conf.upperLegThickness = 1.5;

        conf.lowerLegLength = 12.5;
        conf.lowerLegMass = 0.08 ;
        conf.lowerLegThickness = 1.5;

        conf.footLength = 5.0;
        conf.footWidth = 1.5;
        conf.footMass = 0.01;
        conf.footHeight = 0.8; //0.3, changed for numerical instability in collision detection

        conf.legDistFromCenter = 1.5;

        conf.footAnklePos = 1.5;
        conf.legAnklePos = 0;
        conf.lowerKneePos = 12;
        conf.upperKneePos = 2;
        conf.legHipPos = 13.2;
        conf.bodyHipHeight = 1;
        conf.bodyHipDepth = 2.5;
        conf.massjointBodyDepth = 7;
        conf.massjointBodyHeight = 5.5;
        conf.massjointMassDepth = 6.5;
        conf.massjointMassHeight = 0.7;

        conf.armLength = 60.0;
        conf.armOffset = 2.5;

        conf.scale = 0.01; //m->cm



        double speedfactor = 1;
        double torquefactor = 0.5; //0.8
        //---------------Motor parameters------------------//
        //Todo: need good, realistic values!
        conf.kneeMotorPower = 30*torquefactor;
        conf.kneeMotorMin = -M_PI*0.5;
        conf.kneeMotorMax = 0.0;
        conf.kneeMotorMaxVel = 8.4*speedfactor; //rad / sec

        conf.hipMotorPower = 55*torquefactor;
        conf.hipMotorMin = -0.16*M_PI;//0.17
        conf.hipMotorMax = +0.16*M_PI;//0.17
        conf.hipMotorMaxVel = 5.6*speedfactor; //rad / sec

        conf.massMotorPower = 30*torquefactor;
        conf.massMotorMin = -0.0;
        conf.massMotorMax = +M_PI;
        conf.massMotorMaxVel = 10*speedfactor;

        //passive Motors
        conf.ankleMotorPower =1.3*torquefactor;
        conf.ankleMotorMin = -0.5;
        conf.ankleMotorMax = 0.5;
        conf.ankleMotorDamping = 0.05;

        conf.invertMotors = false;

        return conf;


      }

      /**
       * Destructor
       */
      virtual ~Runbot();

      /**
       * Place the robot in the desired pose
       * @param pose desired 4x4 pose matrix
       */
      virtual void placeIntern(const osg::Matrix& pose);
      
      /**
       * Create the robot in the desired pose
       * @param pose desired 4x4 pose matrix
       */
      virtual void create(const osg::Matrix& pose);

      /**
       * Return actual sensor values
       * @param sensors pointer to array
       * @param sensorNumber size of the array
       */
      virtual int getSensorsIntern(sensor* sensors, int sensorNumber);

      /**
       * Set the actual motor commands
       * @param motors pointer to array
       * @param motorNumber size of the array
       */
      virtual void setMotorsIntern(const motor* motors, int motorNumber);

      /**
       * Return the number of sensors
       */
      virtual int getSensorNumberIntern();
      
      /**
       * Return the number of motors
       */
      virtual int getMotorNumberIntern();

      /** This function is called in each timestep. It should perform robot-internal checks,
       * like space-internal collision detection, sensor resets/update etc.
       * @param globalData structure that contains global data from the simulation environment
       */
      virtual void doInternalStuff(GlobalData& globalData);


      /* This function returns the primitive of the base, runbot is connected to
       * (should be fixated in simulation)
       */
      virtual Primitive* getCenterAnchor();

    protected:
      RunbotConf conf;

    private:
      double sign(double);
      std::vector<OneAxisServo*> motors;
      std::vector<OneAxisServo*> passiveMotors;
      std::vector<ContactSensor*> contactsensors;
      Primitive* centerAnchor;
      Hinge2Joint* centerJoint;
  };

} //end of the namespace lpzrobots

#endif

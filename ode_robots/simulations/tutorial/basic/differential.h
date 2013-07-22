// Header guard
#ifndef __DIFFERENTIAL_H
#define __DIFFERENTIAL_H

// Include ODE Robot class to inherit from it
#include <ode_robots/oderobot.h>

// ODE primitives
#include <ode_robots/primitive.h>

// ODE joints for objects
#include <ode_robots/joint.h>

// ODE angular motors
#include <ode_robots/angularmotor.h>

// ODE infrared distance sensors
#include <ode_robots/raysensorbank.h>
#include <ode_robots/irsensor.h>

// Using name space lpzrobots
namespace lpzrobots{

  // structure to hold configuration of the robot
  typedef struct{
    double bodyRadio; // Radio of the cylinder defining the body
    double bodyHeight; // Height of the cylinder defining the body
    double bodyMass; // Mass of the body
    double wheelRadio; // Radio of the cylinder defining the wheel
    double wheelHeight; // Height of the cylinder defining the wheel
    double wheelMass; // Mass of the wheel
    double wheelMotorPower; // Maximum power allowed to the motor to reach MaxSpeed
    double wheelMotorMaxSpeed; // Maximum speed of the wheel
    double irRange; // Range (max distance) of the infra-red sensors
  } DifferentialConf; 

  /**
   * Differential robot: two separated wheel on each side of the body
   * Inherit from OdeRobot
   */
  class Differential: public OdeRobot{
    public:
      // Structure to hold the configuration of the robot
      DifferentialConf conf;

      /**
       * Contrustructor
       */
      Differential(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
          const DifferentialConf &conf, const std::string& name);

      /**
       * Default configuration of the robot
       */
      static DifferentialConf getDefaultConf(){
        DifferentialConf conf;
        conf.bodyRadio          = 1.0;
        conf.bodyHeight         = .3;
        conf.bodyMass           = 1.0;
        conf.wheelRadio         = .3;
        conf.wheelHeight        = .1;
        conf.wheelMass          = 5.0;
        conf.wheelMotorPower    = 5.0;
        conf.wheelMotorMaxSpeed = 5.0;
        conf.irRange            = 2.0;
        return conf;
      }

      /**
       * Destructor
       */
      virtual ~Differential();

      /**
       * Place the robot in the desired pose
       * @param pose desired 4x4 pose matrix
       */
      virtual void place(const osg::Matrix& pose);

      /**
       * Create the robot in the desired pose
       * @param pose desired 4x4 pose matrix
       */
      virtual void create(const osg::Matrix& pose);

      /**
       * Return actual sensor values
       * @param sensors pointer to array
       * @param sensornumber size of the array
       */
      virtual int getSensors(sensor* sensors, int sensorNumber);

      /**
       * Set actual motor commands
       * @param motors pointer to array
       * @param motornumber size of the array
       */
      virtual void setMotors(const motor* motors, int motorNumber);

      /**
       * Return the number of sensors
       */
      virtual int getSensorNumber();

      /**
       * Return the number of motors
       */
      virtual int getMotorNumber();

      /**
       * OSG nodes are updated
       */
      virtual void update();

      /* This function is called in each timestep. It should perform robot-internal checks,
       * like space-internal collision detection, sensor resets/update etc.
       * @param globalData structure that contains global data from the simulation environment
       * */
      virtual void doInternalStuff(GlobalData& globalData);

    private:
      std::vector <AngularMotor1Axis*> wheelMotors; // vector to hold wheel motors
      RaySensorBank irSensorBank; // a collection of ir sensors
  };


} // end namespace lpzrobots


// End of header guard
#endif

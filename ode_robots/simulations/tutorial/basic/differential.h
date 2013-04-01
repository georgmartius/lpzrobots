// Header guard
#ifndef __DIFFERENTIAL_H
#define __DIFFERENTIAL_H

// Include ODE Robot class to inherit from it
#include <ode_robots/oderobot.h>

// ODE primitives
#include <ode_robots/primitive.h>


// Using name space lpzrobots
namespace lpzrobots{

  // structure to hold configuration of the robot
  typedef struct{
    double bodyRadio;
    double bodyHeight;
    double bodyMass;
    double wheelRadio;
    double wheelHeight;
    double wheelMass;
  } DifferentialConf; 

  /**
   * Differential robot: two separated wheel on each side of the body
   * Inherit from OdeRobot
   */
  class Differential: public OdeRobot{
    public:
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
        conf.bodyRadio   = 1.0;
        conf.bodyHeight  = 1.0;
        conf.bodyMass    = 1.0;
        conf.wheelRadio  = .5;
        conf.wheelHeight = .5;
        conf.wheelMass   = 1.0;
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
    
      /// update the OSG notes here
      virtual void update();

    private:
      Primitive* body;
  };

} // end namespace lpzrobots


  // End of header guard
#endif

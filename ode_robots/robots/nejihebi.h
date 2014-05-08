/***************************************************************************
 *   Copyright (C) 2012 by1                                                *
 *    Timo Nachstedt <nachstedt@physik3.gwdg.de>                           *
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
 ***************************************************************************/

#ifndef LPZROBOTS_ODE_ROBOTS_ROBOTS_NEJIHEBI_H_
#define LPZROBOTS_ODE_ROBOTS_ROBOTS_NEJIHEBI_H_

#include <ode_robots/oderobot.h>

// forward declarations
namespace lpzrobots {
  class Box;
  class Color;
  class Cylinder;
  class HingeJoint;
  class TwoAxisServoVel;
}


namespace lpzrobots{
  /**
   * Nejihebi Robot
   *
   * This is a snake-like robot with screw drive mechanism as developed at
   * the Matsuno Lab in Kyoto, Japan. For details on the real robot see:
   *
   * Fukushima et al: Modeling and Control of a Snake-Like Robot Using the
   * Screw-Drive Mechanism. IEEE Transactions on Robotics 28(3), 2012.
   *
   */
  class Nejihebi : public OdeRobot, public Inspectable {
    public:

      /**
       * Struct containing all the configuration options for the nejihebi robot
       */
      struct Conf {
        /** number of screw elements */
        int    numberOfScrews;
        /** configurations for the two ball bearings at the robot head */
        struct {
            /** color of ball bearing */
            Color   color;
            /** mass of each ball bearing */
            double mass;
            /** radius of each ball bearing */
            double radius;
            /** color of the stick holding the ball bearing */
            Color  stickColor;
            /** length of the stick holding the ball bearing */
            double stickLength;
            /** mass of the stick holding the ball bearing */
            double stickMass;
            /** width of the stick holding the ball bearing */
            double stickWidth;
        } ballBearing;
        /** configurations for the blades holding the passive wheels */
        struct {
            /** angle of the blades versus the screw axis */
            double angle;
            /** color of the blades */
            Color   color;
            /** height of the blades */
            double height;
            /** length of the blades */
            double length;
            /** mass of each blade */
            double mass;
            /** number of wheels per blade */
            int    wheels;
            /** width of each blade */
            double width;
        } blade;
        /** configurations for the robot head */
        struct {
            /** color of the robot head */
            Color   color;
            /** height of the robot head */
            double height;
            /** length of the robot head */
            double length;
            /** mass of the robot head */
            double mass;
            /** width of the robot head */
            double width;
        } head;
        /** configurations for the inner non-rotating part of the screw
         * elements */
        struct {
            /** length of inner body behind the screw center */
            double backLength;
            /** color of the inner body */
            Color   color;
            /** length of inner body before the screw center */
            double frontLength;
            /** height of the inner body */
            double height;
            /** mass of the inner body */
            double mass;
            /** width of the inner body */
            double width;
        } innerBody;
        /** configurations for the joints between the screw elements */
        struct {
            /** damping of the joint servo motors */
            double damping;
            /** maximum velocity of the joint servo motors */
            double maxVel;
            /** configurations for the pitch (up-down) servo motors */
            struct {
                /** minimum angle [rad] */
                double minAngle;
                /** maximum angle [rad] */
                double maxAngle;
                /** maximum power */
                double power;
            } pitch;
            /** configurations for the yaw (left-right) servo motors */
            struct {
                /** minimum angle [rad] */
                double minAngle;
                /** maximum angle [rad] */
                double maxAngle;
                /** maximum power */
                double power;
            } yaw;
        } jointUnit;
        /** configurations for the basic ring of the screw elements */
        struct {
            /** number of blades per screw element */
            int    blades;
            /** color of screw ring */
            Color   color;
            /** mass of screw ring */
            double mass;
            /** maximum torque for screw ring */
            double maxForce;
            /** maximum speed for screw ring */
            double maxSpeed;
            /** radius of screw ring */
            double radius;
            /** width of screw ring */
            double width;
        } screwbase;
        /** configurations for the passive wheels */
        struct {
            /** decides if the wheel axis will be visible */
            bool   axisVisible;
            /** size of the wheel axis if visible */
            double axisSize;
            /** color of the passive wheels */
            Color   color;
            /** mass of each passive wheel */
            double mass;
            /** radius of the circle on which the wheels are placed */
            double posradius;
            /** offset of the center of the position circle and the screw
             * center */
            double posshift;
            /** angle size of the circle segment on which the wheels are
             * placed */
            double possegment;
            /** radius of the passive wheels */
            double radius;
            /** width of the passive wheels */
            double width;
        } wheel;
      };

      /**
       * Constructor
       *
       * @param odeHandle data structure for accessing ODE
       * @param osgHandle data structure for accessing OSG
       * @param conf configuration object
       * @param name name to display for this robot
       */
      Nejihebi(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
          const Conf& conf = getDefaultConf(),
          const std::string& name = "ScrewDriveSnake");

      /**
       * Destructor
       */
      virtual ~Nejihebi();

      /**
       * Returns the current configuration of the robot
       * @return current configuration
       */
      const Conf& getConf() const;

      /**
       * Returns default configuration.
       *
       * This method returns a configuration object with the default
       * configuration for the robot which is supposed to be as close as
       * possible to the real robot.
       *
       * @return default configuration
       */
      static Conf getDefaultConf();

      Inspectable::iparamkeylist getInternalParamNames() const;

      Inspectable::iparamvallist getInternalParams() const;

      /**
       * Returns main primitive of the robot.
       *
       * This method returns the primitive of the robot that is used for
       * tracking and camera following. For the Nejihebi robot that is the
       * robot head object
       *
       * @return pointer to the robot head primitive
       */
      virtual Primitive* getMainPrimitive() const;

      /**
       * Returns number of motors.
       *
       * @return number of motors
       */
      virtual int getMotorNumberIntern();

      /**
       * Returns number of sensors.
       *
       * @return number of sensors
       */
      virtual int getSensorNumberIntern();

      /**
       * Returns current sensor values
       *
       * This method returns all the sensor values of the robot. They are scaled
       * into the interval -1 to 1. Let N be the number of screws for the
       * current robot object. Then the sensor values are as follows:
       *
       *   0    : rotation angle of first screw (foremost screw)   [-pi, pi]
       *   ...
       *   N-1  : rotation angle of N-th screw  (backmost screw)   [-pi, pi]
       *   N    : position of first yaw yoint   (foremost joint)   [-pi, pi]
       *   N+1  : angle velocity of first yaw joint        [-maxVel, maxVel]
       *   N+2  : position of first pitch joint                    [-pi, pi]
       *   N+3  : angle velocity of first pitch joint      [-maxVel, maxVel]
       *   N+4  : position of second yaw joint
       *   ...
       *   5N-5 : angle velocity of last pitch joint
       *
       * @param sensors sensor array to which the sensors scaled to [-1,1] will
       *                be written
       * @param sensornumber length of the sensor array
       * @return number of actually written sensors
       */
      virtual int getSensorsIntern(sensor* sensors, int sensornumber);

      /**
       * Assigns a name to a motor.
       *
       * This method assigns a human readable name to a motor. This name is used
       * for the associated inspectable value of the wiring as used e.g. in
       * guilogger.
       *
       * @param motorNo index of the motor
       * @param name human readable name for the motor
       */
      void nameMotor(const int motorNo, const std::string& name);

      /**
       * Assigns a name to sensor.
       *
       * This method assigns a human readable name to a sensor. This name is
       * used for the associated inspectable value of the wiring as used e.g.
       * in guilogger.
       *
       * @param sensorNo index of the sensor
       * @param name human readable name for the sensor
       */
      void nameSensor(const int sensorNo, const std::string& name);

      /**
       * Sets the pose of the vehicle
       *
       * @param pose desired pose matrix
       */
      virtual void placeIntern(const osg::Matrix& pose);

      /**
       * Sets current motorcommands
       *
       * This method is used to deliver motor commands to the robots. If N is
       * the number of screw elements of the robot then the meaning of the motor
       * commands is as follows:
       *
       *  0    : goal speed for first screw (foremost screw) [-maxVel, maxVel]
       *  N-1  : goal speed for N-th screw (backmost screw)  [-maxVel, maxVel]
       *  N    : goal position for first yaw joint           [-pi, pi]
       *  N+1  : goal position for first pitch joint         [-pi, pi]
       *  N+2  : goal position for second yaw joint          [-pi, pi]
       *  ...
       *  3N-3 : goal position for last pitch joint          [-pi, pi]
       *  3N-2 : torque limit for first yaw joint            [0:1]
       *  3N-1 : torque limit for first pitch joint          [0:1]
       *  3N   : torque limit for second yaw joint           [0:1]
       *  ...
       *  5N-5 : torque limit for last pitch joint           [0:1]
       *
       * @param motors motors scaled to [-1,1]
       * @param motornumber length of the motor array
       */
      virtual void setMotorsIntern(const double* motors, int motornumber);

    private:

      /** Struct to hold all relevant parts of a screw element */
      struct Screw {
        Cylinder * screwBase;
        Box * innerPart;
        HingeJoint * joint;
      };

      /**
       * Creates vehicle at desired position
       *
       * @param pose 4x4 pose matrix defining desired position
       */
      void create(const osg::Matrix& pose);

      /**
       * Creates a screw element at the desired position.
       *
       * This method creates a single screw element at the desired position.
       * With the parameter inverted you can switch between left-turning
       * and right-turning screw elements.
       *
       * @param pose 4x3 pose matrix defining desired position
       */
      Screw createScrew(const osg::Matrix& pose, const bool inverted);

      /** pointer to the head box of the robot*/
      Box* head;
      /** vector holding all structs for all screw elements */
      std::vector<Screw> screws;
      /** vector holding all joint servo motors */
      std::vector<TwoAxisServoVel*> servos;
      /** configuration object of the robot */
      Conf conf;
  };
}


#endif /* LPZROBOTS_ODE_ROBOTS_ROBOTS_SCREWDRIVESNAKE_H_ */

/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
#ifndef __ABSTRACTWIRING_H
#define __ABSTRACTWIRING_H

//#include "abstractrobot.h"
#include "matrix.h"
#include "noisegenerator.h"
#include "inspectable.h"
#include "randomgenerator.h"
#include "sensormotorinfo.h"


/** Abstract wiring-object between controller and robot.
 *  Implements wiring of robot sensors to inputs of the controller and
 *  controller outputs to robot motors.
 */
class AbstractWiring : public Inspectable {
public:
  typedef double sensor;
  typedef double motor;

  enum PlotTypes {Nothing=0, Robot=1, Controller=4, Noise=8};


  /** constructor
   *  @param noise NoiseGenerator that is used for adding noise to sensor values
   */
  AbstractWiring(NoiseGenerator* noise, int plotMode=Controller, const std::string& name = "AbstractWiring")
    : Inspectable(name), plotMode(plotMode) {
    rsensornumber = 0;
    rmotornumber  = 0;
    csensornumber = 0;
    cmotornumber  = 0;
    noiseGenerator = noise;
    noisevals=0;
    initialised = false;
  }

  /** destructor
   */
  virtual ~AbstractWiring(){
    if(noiseGenerator) delete noiseGenerator;
  }

  /** Initializes the  number of sensors and motors from robot
   *  (to be precise the internal parameters rsensornumber and rmotornumber!),
   *  calculates the number of sensors and motors on controller side.
   *  The internal version initIntern() is called from here and
   *   be overloaded to calculate and provide the appropriate numbers
   *  controllersensornumber (csensornumber), controllermotornumber (cmotornumber)
   *  The number of noise channels (noisenumber) can also be changed.
   *  @param randGen pointer to random generator, if not given then a new one is created
   *  @return returns false on error, otherwise true
   */
  virtual bool init(int robotsensornumber, int robotmotornumber, RandGen* randGen=0);

  /** Realizes wiring from robot sensors to controller sensors.
   *   The internal version wireSensorsIntern() is called from here and
   *    must be overloaded in order to implement the appropriate mapping.
   *   Noise values of the right size are then accessible via the noisevals array.
   *   @param rsensors pointer to array of sensorvalues from robot
   *   @param rsensornumber number of sensors from robot
   *   @param csensors pointer to array of sensorvalues for controller
   *   @param csensornumber number of sensors to controller
   *   @param noiseStrength size of the noise added to the sensors
   *   @return returns false on error, otherwise true
   */
  virtual bool wireSensors(const sensor* rsensors, int rsensornumber,
                           sensor* csensors, int csensornumber,
                           double noiseStrength);

  /** Realizes wiring from controller motor outputs to robot motors.
   *   The internal version wireMotorsIntern() is called from here and
   *    must be overloaded in order to implement the appropriate mapping.
   *   @param rmotors pointer to array of motorvalues for robot
   *   @param rmotornumber number of robot motors
   *   @param cmotors pointer to array of motorvalues from controller
   *   @param cmotornumber number of motorvalues from controller
   *   @return returns false if error, else true
   */
  virtual bool wireMotors(motor* rmotors, int rmotornumber,
                          const motor* cmotors, int cmotornumber);

  /** Returns the number of sensors on robot side.
   */
  virtual int getRobotSensornumber(){return rsensornumber;}

  /** Returns the number of motors on robot side.
   */
  virtual int getRobotMotornumber() {return rmotornumber;}

  /** Returns the number of sensors on controller side.
   */
  virtual int getControllerSensornumber(){return csensornumber;}

  /** Returns the number of motors on controller side.
   */
  virtual int getControllerMotornumber() {return cmotornumber;}

  /** routes the infos of the motors from robot to controller */
  virtual std::list<SensorMotorInfo> wireSensorInfos(const std::list<SensorMotorInfo>& robotSensorInfos);

  /** routes the infos of the motors from robot to controller */
  virtual std::list<SensorMotorInfo> wireMotorInfos(const std::list<SensorMotorInfo>& robotMotorInfos);

  /// reset internal state
  virtual void reset() {}

  /// used by WiredController to pass infos to inspectable
  void addSensorMotorInfosToInspectable(const std::list<SensorMotorInfo>& robotSensorInfos,
                                        const std::list<SensorMotorInfo>& robotMotorInfos,
                                        const std::list<SensorMotorInfo>& controllerSensorInfos,
                                        const std::list<SensorMotorInfo>& controllerMotorInfos);


protected:
    // static std::list<std::string> infosToNames(std::list<SensorMotorInfo> infos) {
    //   std::list<std::string> names;
    //   std::transform(infos.begin(), infos.end(), names.begin(), [](const SensorMotorInfo& i){return i.name;});
    //   return names;
    // }

  /** to be overloaded by subclasses
      The rsensornumber and rmotornumber are already stored
      in the member variables. The random values are to be accessed
      via the noiseGenerator.
      @see init()
   */
  virtual bool initIntern() = 0;

  /** to be overloaded by subclasses
      @see wireSensors()
   */
  virtual bool wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                                 sensor* csensors, int csensornumber,
                                 double noiseStrength) = 0;

  /** to be overloaded by subclasses
      @see wireMotors()
   */
  virtual bool wireMotorsIntern(motor* rmotors, int rmotornumber,
                                const motor* cmotors, int cmotornumber)  = 0;


  /// using plotTypes this variables defines what is plotted
  int plotMode;

  /// for storing the noise values
  matrix::Matrix mNoise;
  sensor* noisevals; // pointer to the noisevalues stored in the matrix
  // size of the noise vector
  int noisenumber;

  /// number of sensors at robot side
  int rsensornumber;
  /// copy of the last robot sensors
  matrix::Matrix mRsensors;

  /// number of motors at robot side
  int rmotornumber;
  /// copy of the last robot motors
  matrix::Matrix mRmotors;

  /// number of sensors at controller side
  int csensornumber;
  /// copy of the last controller sensors
  matrix::Matrix mCsensors;

  /// number of motors at controller side
  int cmotornumber;
  /// copy of the last controller motors
  matrix::Matrix mCmotors;

  /// noise generator
  NoiseGenerator* noiseGenerator;

  /// random generator used in NoiseGenerator (in case it is needed by subclasses)
  RandGen* randGen;

  bool initialised;
};

#endif

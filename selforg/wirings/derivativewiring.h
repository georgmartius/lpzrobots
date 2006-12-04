/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
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
 *   Revision 1.3  2006-12-04 16:05:10  der
 *   under construction
 *
 *   Revision 1.2  2006/07/14 12:24:02  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2005/11/16 11:24:28  martius
 *   moved to selforg
 *
 *   Revision 1.6  2005/10/28 12:05:27  martius
 *   adapted time horizont for derivative
 *    to quater of the time horizont of averaging
 *
 *   Revision 1.5  2005/10/24 13:32:07  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.4  2005/10/24 11:06:33  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.3  2005/07/21 15:09:00  martius
 *   blind motors
 *
 *   Revision 1.2  2005/07/21 11:30:59  fhesse
 *   started with blind motors
 *
 *   Revision 1.1  2005/07/18 14:44:55  martius
 *   wiring that supports derivatives
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __DERIVATIVEWIRING_H
#define __DERIVATIVEWIRING_H

#include "abstractwiring.h"

/**  Configuration Object for DerivativeWiring.
     If all boolean parametes are false, id is set to true (equivalent to One2OneWiring)
*/
typedef struct __DerivativeWiringConf {
  bool useId;        //< include zeroth derivative
  bool useFirstD;   //< include first derivative
  bool useSecondD;  //< second include second derivative
  double eps;       //< update rate for floating average (0 -> no sensor variation, 1 -> no smoothing)
  double derivativeScale;   //< factor for the derivatives
  unsigned int blindMotorSets;   //< number of complete motor sets that are blind (not given to robot)
} DerivativeWiringConf;


/** Implements a wiring (between controller and robot) 
    which includes the first and second derivative 
    of the original robot sensor values
*/
class DerivativeWiring : public AbstractWiring{
public:
  /** constructor
      @param conf  for giving the wished configuration of DerivativeWiring 
      via \ref __DerivativeWiringConf "DerivativeWiringConf" 
      @param noise NoiseGenerator that is used for adding noise to sensor values  
  */
  DerivativeWiring(const DerivativeWiringConf& conf, 
		   NoiseGenerator* noise);

  /** destructor
   */
  virtual ~DerivativeWiring();

  /** initializes the internal numbers of sensors and motors on robot side, calculate
      number of sensors and motors on controller side
  */
  virtual bool init(int robotsensornumber, int robotmotornumber);

  /** Realizes derivative wiring from robot sensors to controller sensors. 
      @param rsensors pointer to array of sensorvalues from robot 
      @param rsensornumber number of sensors from robot
      @param csensors pointer to array of sensorvalues for controller  
      @param csensornumber number of sensors to controller
      @param noise size of the noise added to the sensors
  */
  virtual bool wireSensors(const sensor* rsensors, int rsensornumber, 
			   sensor* csensors, int csensornumber,
			   double noise);

  /** Realizes wiring from controller motor outputs to robot motors. 
      @param rmotors pointer to array of motorvalues for robot 
      @param rmotornumber number of robot motors 
      @param cmotors pointer to array of motorvalues from controller  
      @param cmotornumber number of motorvalues from controller
  */
  virtual bool wireMotors(motor* rmotors, int rmotornumber,
			  const motor* cmotors, int cmotornumber);

  /** Providing default configuration for DerivativeWiring as static method
   */
  static DerivativeWiringConf getDefaultConf(){
    DerivativeWiringConf c;
    c.useId = true;        // use id
    c.useFirstD = false;   // do not use first derivative
    c.useSecondD = false;  // do not use secound derivative
    c.eps = 0.05;        
    c.derivativeScale=3;
    c.blindMotorSets=0;    // no blind motors used
    return c;
  };

protected:
  /** Calculate the first derivative of the sensorvalues given by the robot
   *  f'(x) = (f(x+1) - f(x-1)) / 2
   *  since we do not have f(x+1) we go one timestep in the past
   */
  void calcFirstDerivative();

  /** Calculate the secound derivative of the sensorvalues given by the robot
   *  f'(x) = f(x) - 2f(x-1) + f(x-2)
   */
  void calcSecondDerivative();

  /// used configuration
  DerivativeWiringConf conf;
  static const int buffersize=40;
  int time;
  /// number timesteps the sensor values are delayed for calculation of the derivative
  //  int delay;
  

  /// current and old smoothed sensor values of robot
  sensor* sensorbuffer[buffersize]; 

  /// current sensors (with noise)
  sensor* id;

  /// current first derivative
  sensor* first;               

  /// current second derivative
  sensor* second;            

  /// array that stored the values of the blind motors     
  motor *blindMotors;        

  /// number of blind motors used
  unsigned int blindMotorNumber;
};

#endif

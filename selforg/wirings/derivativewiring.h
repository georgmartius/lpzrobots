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
 *   Revision 1.12  2011-03-21 17:49:39  guettler
 *   - adapted to enhance Inspectable interface (has now a name shown also in GuiLogger)
 *
 *   Revision 1.11  2010/07/02 15:57:25  martius
 *   wirings have new initIntern signature -> less errors can be made
 *   abstractwiring generates the noise of given length
 *
 *   Revision 1.10  2009/08/05 22:32:21  martius
 *   big change:
 *       abstractwiring is responsable for providing sensors and motors
 *        and noise to the inspectable interface.
 *       external interface: unchanged except plotMode in constructor
 *       internal interface: all subclasses have to overload
 *         initIntern, wireSensorsIntern, wireMotorsIntern
 *       All existing implementation are changed
 *
 *   Revision 1.9  2008/04/17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.8  2007/12/12 10:26:14  der
 *   MI: calculation of H(X) and H(Y|X) added
 *
 *   Revision 1.7  2006/12/21 11:44:17  martius
 *   commenting style for doxygen //< -> ///<
 *   FOREACH and FOREACHC are macros for collection iteration
 *
 *   Revision 1.6  2006/12/12 09:43:40  martius
 *   getDefaultConf1()
 *
 *   Revision 1.5  2006/12/11 18:23:11  martius
 *   changed order again: first all sensors, then all derivatives ...
 *   noise is only added to first sensor set
 *   now 2 functions for default configs
 *   blind motors not as sets, but as direct number given
 *
 *   Revision 1.4  2006/12/04 17:44:28  martius
 *   unclear
 *
 *   Revision 1.3  2006/12/04 16:05:10  der
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

/**  Configuration object for DerivativeWiring.
     If all boolean parametes are false, id is set to true (equivalent to One2OneWiring)
*/
typedef struct __DerivativeWiringConf {
  bool useId;        //< include zeroth derivative
  bool useFirstD;   ///< include first derivative
  bool useSecondD;  ///< second include second derivative
  double eps;       ///< update rate for floating average (0 -> no sensor variation, 1 -> no smoothing)
  double derivativeScale;   ///< factor for the derivatives
  unsigned int blindMotors;   ///< number of motors that are blind (not given to robot)
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
		   NoiseGenerator* noise, const std::string& name = "DerivativeWiring");

  /** destructor
   */
  virtual ~DerivativeWiring();

  /** Providing default configuration for DerivativeWiring with first derivative.
      No smoothing and no scaling. ( as static method )
   */
  static DerivativeWiringConf getDefaultConf(){
    DerivativeWiringConf c;
    c.useId = true;        // use id
    c.useFirstD = false;    // use first derivative
    c.useSecondD = false;  // do not use secound derivative
    c.eps = 1;             // no smoothing
    c.derivativeScale=1;   // no scaleing
    c.blindMotors=0;       // no blind motors used
    return c;
  };

  /** Providing default configuration for DerivativeWiring for only first derivative. 
      smoothing over 4 steps and scale of 5. Use smaller noise!
      ( as static method )
   */
  static DerivativeWiringConf getDefaultConf1(){
    DerivativeWiringConf c;
    c.useId = false;       // do not use id
    c.useFirstD = true;    // use first derivative
    c.useSecondD = false;  // do not use secound derivative
    c.eps = 0.5;          // smoothing over 2 steps
    c.derivativeScale=5;   // scaling with 5
    c.blindMotors=0;       // no blind motors used
    return c;
  };

protected:

  virtual bool initIntern();

  virtual bool wireSensorsIntern(const sensor* rsensors, int rsensornumber, 
				 sensor* csensors, int csensornumber,
				 double noise);

  virtual bool wireMotorsIntern(motor* rmotors, int rmotornumber,
				const motor* cmotors, int cmotornumber);

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
  // sensor* id;

  /// current first derivative
  sensor* first;               

  /// current second derivative
  sensor* second;            

  /// array that stored the values of the blind motors     
  motor *blindMotors;        

};

#endif

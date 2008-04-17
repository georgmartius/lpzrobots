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
 *   Revision 1.5  2008-04-17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.4  2007/11/29 19:18:02  martius
 *   blind channels
 *
 *   Revision 1.3  2006/07/20 17:14:36  martius
 *   removed std namespace from matrix.h
 *   storable interface
 *   abstract model and invertablemodel as superclasses for networks
 *
 *   Revision 1.2  2006/07/14 12:24:02  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.1  2005/11/16 11:24:28  martius
 *   moved to selforg
 *
 *   Revision 1.9  2005/10/28 12:05:54  martius
 *   new inspectable interface
 *
 *   Revision 1.8  2005/10/24 13:32:07  fhesse
 *   comments adjusted and in doxygen style
 *
 *   Revision 1.7  2005/10/06 17:11:37  martius
 *   switched to stl lists
 *
 *   Revision 1.6  2005/08/31 11:10:52  martius
 *   noise -> noisevals
 *
 *   Revision 1.5  2005/08/03 20:34:58  martius
 *   use if Inspectable interface
 *
 *   Revision 1.4  2005/07/21 15:14:47  martius
 *   wireSensors and wireMotors get constant fields
 *
 *   Revision 1.3  2005/07/18 14:44:27  martius
 *   noise moved into wiring
 *
 *   Revision 1.2  2005/07/18 10:15:03  martius
 *   noise is added here
 *
 *   Revision 1.1  2005/07/14 15:57:54  fhesse
 *   now agent contains controller, robot and wiring, plotting ability included, therefore plotagent can be removed; ono2onewiring replaces one2oneagent
 *                                            *
 *                                                                         *
 ***************************************************************************/
#ifndef __ONE2ONEWIRING_H
#define __ONE2ONEWIRING_H

#include "abstractwiring.h"

/** Implements one to one wireing of robot sensors to inputs of the controller 
    and controller outputs to robot motors. 
 */
class One2OneWiring :public AbstractWiring{
public:
  /** constructor
      @param noise NoiseGenerator that is used for adding noise to sensor values  
      @param plotNoise for plotting the noise values (to observe it from outside
      via getInternalParams() and guilogger) set it TRUE, for not plotting the noise set 
      it to FALSE.
      @param blind number of blind channels
        (additional sensors and motors coupled directly)
   */
  One2OneWiring(NoiseGenerator* noise, bool plotNoise=false, int blind=0);

  /** destructor
   */
  virtual ~One2OneWiring();

  /** initializes the number of sensors and motors on robot side, calculate
      number of sensors and motors on controller side
   */
  virtual bool init(int robotsensornumber, int robotmotornumber, RandGen* randGen=0);

  /** Realizes one to one wiring from robot sensors to controller sensors. 
      @param rsensors pointer to array of sensorvalues from robot 
      @param rsensornumber number of sensors from robot
      @param csensors pointer to array of sensorvalues for controller  
      @param csensornumber number of sensors to controller
      @param noise size of the noise added to the sensors
  */
  virtual bool wireSensors(const sensor* rsensors, int rsensornumber, 
			   sensor* csensors, int csensornumber,
			   double noise);

  /** Realizes one to one wiring from controller motor outputs to robot motors. 
      @param rmotors pointer to array of motorvalues for robot 
      @param rmotornumber number of robot motors 
      @param cmotors pointer to array of motorvalues from controller  
      @param cmotornumber number of motorvalues from controller
  */
  virtual bool wireMotors(motor* rmotors, int rmotornumber,
			  const motor* cmotors, int cmotornumber);

  /** Returns the list of the names of all internal parameters.
  */
  virtual std::list<iparamkey> getInternalParamNames() const;

  /** The list of the values of all internal parameters given by getInternalParams().
      (in the order given by getInternalParamNames())
   */
  virtual std::list<iparamval> getInternalParams() const;

protected:
  /// TRUE for plotting noise values, FALSE for not plotting
  bool plotNoise; 
  /// for storing the noise values
  sensor* noisevals;

  int blind; /// number of blind channels
  /// blind motor values
  motor* blindmotors;

};

#endif

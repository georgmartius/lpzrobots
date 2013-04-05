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
 *   Revision 1.2  2011-03-21 17:39:55  guettler
 *   - adapted to enhance Inspectable interface (has now a name shown also in GuiLogger)
 *
 *   Revision 1.1  2007/09/17 19:27:55  fhesse
 *   initial version of wiring for hand with inversion of IR sensors
 *
 *                                            *
 *                                                                         *
 ***************************************************************************/
#ifndef __IRINVERTWIRING_H
#define __IRINVERTWIRING_H

#include <selforg/one2onewiring.h>

/** Implements one to one wireing of robot sensors to inputs of the controller
    and controller outputs to robot motors.
 */
class IRInvertWiring :public One2OneWiring{
public:
  /** constructor
      @param noise NoiseGenerator that is used for adding noise to sensor values
      @param plotNoise for plotting the noise values (to observe it from outside
      via getInternalParams() and guilogger) set it TRUE, for not plotting the noise set
      it to FALSE.
   */
  IRInvertWiring(NoiseGenerator* noise, bool plotNoise=false, const std::string& name = "IRInvertWiring");


  /** destructor
   */
  //  virtual ~IRInvertWiring();


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

};

#endif

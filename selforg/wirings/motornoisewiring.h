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
 *   Revision 1.1  2009-03-31 07:36:18  martius
 *   wiring to add noise on motor channels
 *

 *                                                                         *
 ***************************************************************************/
#ifndef __MOTORNOISEWIRING_H
#define __MOTORNOISEWIRING_H

#include "one2onewiring.h"


/** 
 *   Implements a one to one wiring that adds noise to the motor signals
 *   (the sensors will get no noise)
 */
class MotorNoiseWiring : public One2OneWiring {
public:
  /** constructor
      @param noise NoiseGenerator that is used for adding noise to motor values  
  */
  MotorNoiseWiring(NoiseGenerator* noise, double noiseStrength)
    : One2OneWiring(0), noiseStrength(noiseStrength) {} // no noise at sensors
  virtual ~MotorNoiseWiring(){}

  virtual bool init(int robotsensornumber, int robotmotornumber, RandGen* randGen=0){
    One2OneWiring::init(robotsensornumber, robotmotornumber, randGen);
    if(noiseGenerator)
      noiseGenerator->init(rmotornumber, randGen);
    return true;
  }
  
  virtual bool wireMotors(motor* rmotors, int rmotornumber,
			  const motor* cmotors, int cmotornumber){
    One2OneWiring::wireMotors(rmotors, rmotornumber, cmotors, cmotornumber);
    if(noiseGenerator)
      noiseGenerator->add(rmotors, noiseStrength);  
    return true; 
  }

protected:
  double noiseStrength;

};

#endif

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
 *   Revision 1.5  2010-09-30 17:09:33  martius
 *   improved makefile again
 *   examples made them compile
 *   motornoisewiring improved (was not working anymore)
 *
 *   Revision 1.4  2010/09/27 14:52:58  martius
 *   made it compile again
 *
 *   Revision 1.3  2010/03/30 08:48:01  martius
 *   intern function are called now
 *
 *   Revision 1.2  2009/03/31 15:47:11  martius
 *   works now
 *
 *   Revision 1.1  2009/03/31 07:36:18  martius
 *   wiring to add noise on motor channels
 *

 *                                                                         *
 ***************************************************************************/
#ifndef __MOTORNOISEWIRING_H
#define __MOTORNOISEWIRING_H

#include <selforg/one2onewiring.h>
#include <selforg/configurable.h>

/** 
 *   Implements a one to one wiring that adds noise to the motor signals
 *   (the sensors will get no noise)
 */
class MotorNoiseWiring : public One2OneWiring, public Configurable {
public:
  /** constructor
      @param noise NoiseGenerator that is used for adding noise to motor values  
  */
  MotorNoiseWiring(NoiseGenerator* noise, double noiseStrength)
    : One2OneWiring(0, Controller),    // no noise at sensors, show Controller x,y and noise
      Configurable("MotorNoiseWiring", "$Id$"),
      mNoiseGen(noise), noiseStrength(noiseStrength) {
    addParameter("strength",&noiseStrength);

  }
  virtual ~MotorNoiseWiring(){}

  double getNoiseStrength(){ return noiseStrength; }
  void setNoiseStrength(double _noiseStrength) { 
    if(_noiseStrength>=0) noiseStrength=_noiseStrength;
  }


  virtual bool initIntern(){
    One2OneWiring::initIntern();
    if(mNoiseGen)
      mNoiseGen->init(rmotornumber, randGen);
    return true;
  }
  
  virtual bool wireMotorsIntern(motor* rmotors, int rmotornumber,
                                const motor* cmotors, int cmotornumber){
    One2OneWiring::wireMotorsIntern(rmotors, rmotornumber, cmotors, cmotornumber);
    if(mNoiseGen)
      mNoiseGen->add(rmotors, noiseStrength);  
    return true; 
  }

protected:
  NoiseGenerator* mNoiseGen;
  double noiseStrength;

};

#endif

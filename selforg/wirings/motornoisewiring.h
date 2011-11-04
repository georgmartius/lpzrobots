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
      motNoiseGen(noise), motNoiseStrength(noiseStrength) {
  }
  virtual ~MotorNoiseWiring(){}

  double getNoiseStrength(){ return motNoiseStrength; }
  void setNoiseStrength(double _motNoiseStrength) { 
    if(_motNoiseStrength>=0) motNoiseStrength=_motNoiseStrength;
  }


  virtual bool initIntern(){
    One2OneWiring::initIntern();
    mMotNoise.set(rmotornumber,1);
    addParameter("strength", &this->motNoiseStrength,0, 2, "strength of motor value noise (additive)");

    addInspectableMatrix("n", &mMotNoise, false, "motor noise");
    if(motNoiseGen)
      motNoiseGen->init(rmotornumber, randGen);
    return true;
  }
  
  virtual bool wireMotorsIntern(motor* rmotors, int rmotornumber,
                                const motor* cmotors, int cmotornumber){
    One2OneWiring::wireMotorsIntern(rmotors, rmotornumber, cmotors, cmotornumber);
    if(motNoiseGen){
      double* nv = (double*) mMotNoise.unsafeGetData();
      memset(nv, 0 , sizeof(double) * rmotornumber);    
      motNoiseGen->add(nv, motNoiseStrength);
      for(int i=0; i<rmotornumber; i++){
        rmotors[i]+=nv[i];
      }
    }
    return true; 
  }
  

protected:
  NoiseGenerator* motNoiseGen;
  double motNoiseStrength;
  matrix::Matrix mMotNoise;
};

#endif

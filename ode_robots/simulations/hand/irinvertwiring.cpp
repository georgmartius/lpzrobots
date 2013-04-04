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
 *   Revision 1.3  2011-03-21 17:39:55  guettler
 *   - adapted to enhance Inspectable interface (has now a name shown also in GuiLogger)
 *
 *   Revision 1.2  2008/08/20 08:57:40  fhesse
 *   adapted noise generator->add call to new version
 *
 *   Revision 1.1  2007/09/17 19:27:55  fhesse
 *   initial version of wiring for hand with inversion of IR sensors
 *
 *                                            *
 *                                                                         *
 ***************************************************************************/

#include "irinvertwiring.h"


/// constructor
IRInvertWiring::IRInvertWiring(NoiseGenerator* noise, bool plotNoise, const std::string& name)
  : One2OneWiring(noise, plotNoise, 0, name)
{

}
/*
IRInvertWiring::~IRInvertWiring(){
  One2OneWiring::~One2OneWiring();
}
*/

/// Realizes one to one wiring from robot sensors to controller sensors.
//  sensorvalues with sensornumber>motornumber are
//  recalculated as (1-sensorvalue)
//   @param rsensors pointer to array of sensorvalues from robot
//   @param rsensornumber number of sensors from robot
//   @param csensors pointer to array of sensorvalues for controller
//   @param csensornumber number of sensors to controller
//   @param noise size of the noise added to the sensors
bool IRInvertWiring::wireSensors(const sensor* rsensors, int rsensornumber,
                                sensor* csensors, int csensornumber,
                                double noiseStrength){
  if (rsensornumber!=csensornumber)
    return false;
  else{
    memset(noisevals, 0 , sizeof(sensor) * this->rsensornumber);
    //noiseGenerator->add(noisevals, -noiseStrength, noiseStrength);
    noiseGenerator->add(noisevals, noiseStrength);
    for(int i=0; i< rsensornumber; i++){
      csensors[i] = rsensors[i] + noisevals[i];
    }
//  sensorvalues with sensornumber>motornumber are
//  recalculated as (1-sensorvalue)
    for(int i=6; i< rsensornumber; i++){
      csensors[i] = 1.0 - csensors[i];
    }
    return true;
  }
}



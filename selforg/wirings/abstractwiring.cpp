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
 *   Revision 1.5  2010-09-27 14:53:29  martius
 *   store randGen for further use
 *
 *   Revision 1.4  2010/07/02 15:57:25  martius
 *   wirings have new initIntern signature -> less errors can be made
 *   abstractwiring generates the noise of given length
 *
 *   Revision 1.3  2009/10/23 12:38:30  martius
 *   noise is stored in a matrix internally such that it can be inspected easily
 *
 *   Revision 1.2  2009/10/14 09:59:46  martius
 *   added description of vectors
 *
 *   Revision 1.1  2009/08/05 22:32:21  martius
 *   big change:
 *       abstractwiring is responsable for providing sensors and motors
 *        and noise to the inspectable interface.
 *       external interface: unchanged except plotMode in constructor
 *       internal interface: all subclasses have to overload
 *         initIntern, wireSensorsIntern, wireMotorsIntern
 *       All existing implementation are changed
 *
 *                                            *
 *                                                                         *
 ***************************************************************************/

#include "abstractwiring.h"



bool AbstractWiring::init(int robotsensornumber, int robotmotornumber, RandGen* _randGen){    
  rsensornumber = robotsensornumber;
  rmotornumber  = robotmotornumber;
  noisenumber   = rsensornumber;
  randGen       = _randGen;
  bool rv= initIntern();    
  assert(noisenumber>=rsensornumber);
  
  mNoise.set(noisenumber,1);
  noisevals = (double*) mNoise.unsafeGetData(); // hack! we let the noiseval pointer point to the internal memory of the noisematrix.

  if(noiseGenerator)
    noiseGenerator->init(noisenumber, randGen);

  mRsensors.set(rsensornumber,1);
  mRmotors.set(rmotornumber,1);
  mCsensors.set(csensornumber,1);
  mCmotors.set(cmotornumber,1);
  if(plotMode & Controller) {
    addInspectableMatrix("x", &mCsensors, false, "sensor values (after wiring)");
    addInspectableMatrix("y", &mCmotors,  false, "motor values (before wiring)");
  }
  if(plotMode & Robot) {
    addInspectableMatrix("x_R", &mRsensors, false, "sensor values (before wiring)");
    addInspectableMatrix("y_R", &mRmotors,  false, "motor values (after wiring)");
  }
  if(plotMode & Noise) {    
    addInspectableMatrix("n", &mNoise, false, "sensor noise");
  }

  initialised = true;
  return rv;
}

bool AbstractWiring::wireSensors(const sensor* rsensors, int rsensornumber,
				 sensor* csensors, int csensornumber,
				 double noiseStrength){
  assert(initialised);
  if(noiseGenerator) {
    memset(noisevals, 0 , sizeof(sensor) * noisenumber);    
    noiseGenerator->add(noisevals, noiseStrength);  
  } 
  bool rv = wireSensorsIntern(rsensors, rsensornumber, csensors, csensornumber, noiseStrength);
  mRsensors.set(rsensors);
  mCsensors.set(csensors);
  return rv;
}

bool AbstractWiring::wireMotors(motor* rmotors, int rmotornumber,
				const motor* cmotors, int cmotornumber){
  assert(initialised);
  bool rv = wireMotorsIntern(rmotors, rmotornumber, cmotors, cmotornumber);
  mRmotors.set(rmotors);
  mCmotors.set(cmotors);
  return rv;
}


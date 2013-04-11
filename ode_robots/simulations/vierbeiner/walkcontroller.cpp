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
 *   Revision 1.2  2008-05-01 22:03:56  martius
 *   build system expanded to allow system wide installation
 *   that implies  <ode_robots/> for headers in simulations
 *
 *   Revision 1.1  2007/12/13 17:00:02  martius
 *   walk controller
 *
 *                                                                         *
 ***************************************************************************/

#include <stdio.h>
#include <math.h>
#include "walkcontroller.h"

using namespace std;

WalkController::WalkController()
  : AbstractController("walkcontroller", "$Id$"){
  t=0;
  addParameterDef("speed",&speed, 12.0);
  addParameterDef("phase",&phase, 0.0);
  addParameterDef("hipampl",&hipamplitude, 0.8);
  addParameterDef("kneeampl",&kneeamplitude, 0.8);

  number_sensors=0;
  number_motors=0;
};

void WalkController::init(int sensornumber, int motornumber, RandGen* randGen){
  number_sensors=sensornumber;
  number_motors=motornumber;
  if(motornumber < 12) {
    cerr << "Walkcontroller needs 12 motors!" << endl;
    exit(1);
  }
};

void WalkController::step(const sensor* sensors, int sensornumber,
                          motor* motors, int motornumber) {
  stepNoLearning(sensors, sensornumber, motors, motornumber);
};

void WalkController::stepNoLearning(const sensor* sensors, int number_sensors,
                                    motor* motors, int number_motors) {
  double w = t/speed;
  // Horse Walk from wikipedia
  /* The walk is a four-beat gait that averages about 4 mph.
     When walking, a horse's legs follow this sequence:
     left hind leg, left front leg, right hind leg, right front leg,
     in a regular 1-2-3-4 beat. .... */
  double phases[4]= { w + 2*(M_PI/2),
                      w + 0*(M_PI/2),
                      w + 3*(M_PI/2),
                      w + 1*(M_PI/2) };

  motors[0] = sin(phases[0]+2)*0.8;
  motors[1] = 0;

  // hips
  for(int i=0; i<4; i++)
    motors[i+2]=sin(phases[i])*hipamplitude;

  // knees
  for(int i=0; i<2; i++){
    motors[i+6]=sin(phases[i]+1.05)*kneeamplitude;
  }
  for(int i=2; i<4; i++){
    motors[i+6]=sin(phases[i]+1.8)*kneeamplitude;
  }
  // ankles
  for(int i=0; i<2; i++){
    motors[i+10]=sin(phases[i]+1.05)*0.8;
  }

  //rest sine wave
  for(int i=12; i<number_motors; i++){
    motors[i]=sin(phases[i%4])*0.77;
  }
  t++;
};





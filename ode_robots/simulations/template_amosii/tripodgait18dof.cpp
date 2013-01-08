/***************************************************************************
 *   Copyright (C) 2010 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *   $Log: tripodgait18dof.cpp,v $                                         *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
#include <ode_robots/amosiisensormotordefinition.h>
#include "tripodgait18dof.h"
using namespace matrix;
using namespace std;

TripodGait18DOF::TripodGait18DOF(const TripodGait18DOFConf& _conf)
: AbstractController("TripodGait18DOF", "$Id: tripodgait18dof.cpp,v 0.1 $"),
  conf(_conf) {
  t = 0;
  
  outputH1 = 0.001;
  outputH2 = 0.001;
}
;

TripodGait18DOF::~TripodGait18DOF() {
}

void TripodGait18DOF::init(int sensornumber, int motornumber, RandGen* randGen)
{
  //Tripodgait for 18 DOF Hexapod
  assert(motornumber>=18);
}

/// performs one step (includes learning). Calulates motor commands from sensor
/// inputs.
void TripodGait18DOF::step(const sensor* x_, int number_sensors,
    motor* y_, int number_motors) {
  stepNoLearning(x_, number_sensors, y_, number_motors);
}
;

/// performs one step without learning. Calulates motor commands from sensor
/// inputs.
void TripodGait18DOF::stepNoLearning(const sensor* x_, int number_sensors,
    motor* y_, int number_motors) {
  //Tripodgait for 18 DOF Hexapod

  assert(number_sensors >= 18);
  assert(number_motors >= 18);

  double activityH1 = conf.WeightH1_H1 * outputH1 + conf.WeightH1_H2 * outputH2
      + 0.01;
  double activityH2 = conf.WeightH2_H2 * outputH2 + conf.WeightH2_H1 * outputH1
      + 0.01;

  outputH1 = tanh(activityH1);
  outputH2 = tanh(activityH2);
  
  // generate motor commands      
  // right rear coxa (knee) forward-backward joint (back is positive)
  y_[TR2_m] = outputH2 * conf.fact + conf.bias;
  y_[CR2_m] = outputH1 * conf.fact * conf.direction;
  y_[FR2_m] = -y_[1];
  //left rear coxa (knee) forward-backward joint
  y_[TL2_m] = -outputH2 * conf.fact + conf.bias;
  y_[CL2_m] = -outputH1 * conf.fact * conf.direction;
  y_[FL2_m] = -y_[4];
  //right middle coxa (knee) forward-backward joint
  y_[TR1_m] = -outputH2 * conf.fact + conf.bias;
  y_[CR1_m] = -outputH1 * conf.fact * conf.direction;
  y_[FR1_m] = -y_[7];
  //left middle coxa (knee) forward-backward joint
  y_[TL1_m] = outputH2 * conf.fact + conf.bias;
  y_[CL1_m] = outputH1 * conf.fact * conf.direction;
  y_[FL1_m] = -y_[10];
  //right front coxa (knee) forward-backward joint
  y_[TR0_m] = outputH2 * conf.fact + conf.bias;
  y_[CR0_m] = outputH1 * conf.fact * conf.direction;
  y_[FR0_m] = -y_[13];
  //left front coxa (knee) forward-backward joint
  y_[TL0_m] = -outputH2 * conf.fact + conf.bias;
  y_[CL0_m] = -outputH1 * conf.fact * conf.direction;
  y_[FL0_m] = -y_[16];

  // update step counter
  t++;
}

/** stores the controller values to a given file. */
bool TripodGait18DOF::store(FILE* f) const {
  return true;
}

/** loads the controller values from a given file. */
bool TripodGait18DOF::restore(FILE* f) {
  return true;
}

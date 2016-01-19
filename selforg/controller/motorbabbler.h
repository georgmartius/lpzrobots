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
#ifndef __MOTORBABBLER_H
#define __MOTORBABBLER_H

#include <stdio.h>
#include "abstractcontroller.h"
#include "matrix.h"

/**
 * class for robot control that does motor babbling, e.g. sine waves
 * with different frequencies and phaseshift
 */
class MotorBabbler : public AbstractController {
public:
  enum function {Sine, SawTooth};

  /**
     @param controlmask bitmask to select channels to control (default all)
     @param function controller function to use
   */
  MotorBabbler(function func = Sine );

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);
  virtual int getSensorNumber() const {return number_sensors;}
  virtual int getMotorNumber() const {return number_motors;}
  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber){
    stepNoLearning(sensors, sensornumber, motors, motornumber);
  }
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  // samples a new set of frequencies
  void sampleFrequencies();

  /********* STORABLE INTERFACE ******/
  /// @see Storable
  virtual bool store(FILE* f) const {
    Configurable::print(f,"");
    return true;
  }

  /// @see Storable
  virtual bool restore(FILE* f) {
    Configurable::parse(f);
    return true;
  }

  /// sine
  static double sine(double x);
  /// saw tooth shape oscillator
  static double sawtooth(double x);

protected:
  int number_sensors;
  int number_motors;

  paramval minPeriod;
  paramval maxPeriod;
  paramval amplitude;
  paramint resampling;
  paramint mask;
  matrix::Matrix phases;
  matrix::Matrix frequencies;

  RandGen* randGen;

  double (*osci) (double x); // oscillator function
  int t;
};

#endif

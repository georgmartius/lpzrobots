/***************************************************************************
 *   Copyright (C) 2005-2013 LpzRobots development team                    *
 *   Authors:                                                              *
 *    Simón Smith <artificialsimon at ed dot ac dot uk>                    *
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

// Header guard
#ifndef __BASIC_CONTROLLER_H
#define __BASIC_CONTROOLER_H

#include <selforg/abstractcontroller.h>

class BasicController : public AbstractController{
  public:

    BasicController(const std::string& name);

    /** initialisation of the controller with the given sensor/ motornumber
      Must be called before use. The random generator is optional.
      */
    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0)  override;

    /** @return Number of sensors the controller
      was initialised with or 0 if not initialised */
    virtual int getSensorNumber() const override;

    /** @return Number of motors the controller
      was initialised with or 0 if not initialised */
    virtual int getMotorNumber() const override;

    /** performs one step.
      Calculates motor commands from sensor inputs.
      @param sensors sensors inputs scaled to [-1,1]
      @param sensornumber length of the sensor array
      @param motors motors outputs. MUST have enough space for motor values!
      @param motornumber length of the provided motor array
      */
    virtual void step(const sensor* sensors, int sensornumber,
                      motor* motors, int motornumber) override;


    /** performs one step without learning.
      @see step
      */
    virtual void stepNoLearning(const sensor* , int number_sensors,
                                motor* , int number_motors) override;

    /** stores the object to the given file stream (binary).
    */
    virtual bool store(FILE* f) const override;

    /** loads the object from the given file stream (binary).
    */
    virtual bool restore(FILE* f) override;

  protected:
  double nSensors;
  double nMotors;
  bool initialised;
  double threshold;

};

#endif // Header guard

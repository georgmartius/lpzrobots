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
#ifndef __DISCRETESIZABLE_H
#define __DISCRETESIZABLE_H

//#include <iostream>
//#include <list>
//#include <utility>
//#include <string>
//#include <map>
//#include "stl_adds.h"



/**
 * Abstact class for discretesizable controllers. Implements the methods
 * step(...) and stepNoLearning(...) and defines the methods dStep(...)
 * respectively dStepNoLearning(...), which are called from the implemented
 * one.
 *
 * Hint: Do not mistake dStep(...) for a method from the ODE.
 *
 * Additionally the controller implementing this interface must give the range
 * and intervalCount in the constructor of this interface for configuring the
 * discretization correctly.
 *
 * Note: This interface does only discretesize the sensor values.
 */
class Discretesizable : public DiscreteControllerAdapter {
public:

        /** Initializes the discretization.
         * @param intervalCount sets the number of intervals
         * @param mapToInteger if true, all intervals are mapped to 0..(intervalCount-1)
         * @param minSensorValue is neccessary if the sensor range is not in [-1,1]
         * @param maxSensorValue is neccessary if the sensor range is not in [-1,1]
         */
        Discretesizable(int intervalCount, boolean mapToInteger=true, double minSensorValue=-1.0, double maxSensorValue=1.0);

        virtual        ~Discretesizable() {};

        /** performs one step (includes learning).
         * Calculates motor commands from sensor inputs.
         * @note This method cannot be overwritten, use @see dStep instead.
         * @param sensors sensors inputs scaled to [-1,1]
         * @param sensornumber length of the sensor array
         * @param motors motors outputs. MUST have enough space for motor values!
         * @param motornumber length of the provided motor array
         */
        void step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
                DiscreteControllerAdapter::step( sensors,sensornumber, motors, motornumber);
        }

        /** performs one step without learning.
         * @see step
         * @note This method cannot be overwritten, use @see dStepNoLearning
         * instead.
         */
        void stepNoLearning(const sensor* sensors , int sensornumber,
                            motor* motors, int motornumber) {
                                    DiscreteControllerAdapter::step( sensors,sensornumber, motors, motornumber);
                            }

protected:
        ;

private:

};


#endif

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
#ifndef __ONEACTIVEMULTIPASSIVECONTROLLER_H
#define __ONEACTIVEMULTIPASSIVECONTROLLER_H

#include "abstractmulticontroller.h"

/**
 * class (interface) for using multiple controller, the first one is
 * the active one, which generates motor values. The other controllers
 * are passive and cannot affect the motor values.
 *
 */
class OneActiveMultiPassiveController : public AbstractMultiController {
public:

  /// contructor (hint: use $ID$ for revision)
        OneActiveMultiPassiveController(AbstractController* controller, const std::string& name = "1ActXPassController", const std::string& revision = "$ID$");

        virtual ~OneActiveMultiPassiveController();

/****************************************************************************/
/*        AbstractMultiController should implement the following classes:                */
/*        AbstractController, Configurable, Inspectable, Storeable                    */
/****************************************************************************/


/****************************************************************************/
/*        BEGIN methods of AbstractController                                         */
/****************************************************************************/

  /** initialisation of the controller with the given sensor/ motornumber
   * Must NORMALLY be called before use. For all multicontroller
   * call first AbstractMultiController::init(sensornumber,motornumber)
   * if you overwrite this method
   */
        virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);


          /** performs one step (includes learning).
      Calculates motor commands from sensor inputs.
        @param sensors sensors inputs scaled to [-1,1]
        @param sensornumber length of the sensor array
        @param motors motors outputs. MUST have enough space for motor values!
        @param motornumber length of the provided motor array
  */
        virtual void step(const sensor* sensors, int sensornumber,
                          motor* motors, int motornumber);

  /** performs one step without learning.
        @see step
  */
        virtual void stepNoLearning(const sensor* sensors , int sensornumber,
                                    motor* motors, int motornumber);


/****************************************************************************/
/*        END methods of AbstractController                                             */
/****************************************************************************/



/****************************************************************************/
/*        BEGIN methods of Configurable                                               */
/****************************************************************************/

/****************************************************************************/
/*        END methods of Configurable                                                  */
/****************************************************************************/



/****************************************************************************/
/*        BEGIN methods of Inspectable                                                  */
/****************************************************************************/

/****************************************************************************/
/*        END methods of Inspectable                                                   */
/****************************************************************************/



/****************************************************************************/
/*        BEGIN methods of Storeable                                                   */
/****************************************************************************/

/****************************************************************************/
/*        END methods of Storeable                                                      */
/****************************************************************************/


protected:
        motor* passiveMotors;
};

#endif

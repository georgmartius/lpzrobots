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
#ifndef __ABSTRACTMULTICONTROLLER_H
#define __ABSTRACTMULTICONTROLLER_H

#include "abstractcontrolleradapter.h"

/**
 * Abstract class (interface) for using multiple controller.
 * A controller gets a number of input sensor values each timestep
 *  and has to generate a number of output motor values
 *
 *
 * This is an abstract class, it's useful for implementing multicontrollers such as the
 * OneActiveMultiPassiveController, which can be used with all Controllers.
 *
 * Any MulitController implementing this class should overwrite the methods step(...) and
 * stepNoLearning(...).
 */
class AbstractMultiController : public AbstractControllerAdapter {
public:

  /// contructor (hint: use $ID$ for revision)
        AbstractMultiController(AbstractController* controller, const std::string& name, const std::string& revision);

        virtual ~AbstractMultiController();

        /**
         * Adds a passive controller to this MultiController. If the Agent calls step(..)
         * or stepNoLearning(..), the MultiController calls not only the active controllers
         * step(...) but also the step(...) of all the passive controllers. (same for
         * stepNoLearning(..) ).
         *
         * Note: The initialisation of the MultiController with init(sensornumber, motornumber)
         * must be called after all passive controllers are added, otherwise you must
         * init the passive controller yourself (not recommended and can generate problems)
         */
        virtual void addPassiveController(AbstractController* passiveController);

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
                          motor* motors, int motornumber)=0;

  /** performs one step without learning.
        @see step
  */
        virtual void stepNoLearning(const sensor* sensors , int sensornumber,
                                    motor* motors, int motornumber)=0;


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

// nothing to overwrite


/****************************************************************************/
/*        END methods of Inspectable                                                   */
/****************************************************************************/

/****************************************************************************/
/*        BEGIN methods of Storeable                                                   */
/****************************************************************************/

  /** stores the object to the given file stream (binary).
  */
        virtual bool store(FILE* f) const;

  /** loads the object from the given file stream (binary).
  */
        virtual bool restore(FILE* f);


/****************************************************************************/
/*        END methods of Storeable                                                      */
/****************************************************************************/

protected:
        // The AbstractController* controller is defined in AbstractControllerAdapter!
        std::list<AbstractController*> controllerList; // stores the other controllers
        std::list<std::string> controllerNameList; // stores the names of the controllers

};

#endif

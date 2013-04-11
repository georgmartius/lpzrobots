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
#ifndef __ABSTRACTCONTROLLERADAPTER_H
#define __ABSTRACTCONTROLLERADAPTER_H

#include "abstractcontroller.h"

#include <selforg/stl_adds.h>

/**
 * Abstract adapter class (interface) for robot controller.
 * The controller gets a number of input sensor values each timestep
 *  and has to generate a number of output motor values.
 *
 * Interface assumes the following usage:
 *  - init() is called first to initialise the dimension of sensor- and motor space
 *  - each time step
 *     either step() or stepNoLearning() is called to ask the controller for motor values.
 *
 * With the adapter class you can easily overwrite e.g. init(), step().
 *
 * This is an abstract adapter class, it's useful for implementing adapters such as the
 * DescreteController, which can be used with all Controllers.
 *
 * Note that the configureable and inspectable classes are registered at this
 *  adapter class.
 *
 *  The store and restore-functionality is lead through, thus only store() and restore()
 *  from
 */
class AbstractControllerAdapter : public AbstractController {
public:

  AbstractControllerAdapter(AbstractController* controller, const std::string& name, const std::string& revision)
    : AbstractController(name, revision), controller(controller)
  {
    //  register the inspectable and configureable controller
    addConfigurable(controller);
    addInspectable(controller);
  }

  virtual ~AbstractControllerAdapter() {}

  /****************************************************************************/
  /*        AbstractControllerAdapter must implement the following classes:                */
  /*        AbstractController, Configurable, Inspectable, Storeable                    */
  /****************************************************************************/


  /****************************************************************************/
  /*        BEGIN methods of AbstractController                                         */
  /****************************************************************************/

  /** initialisation of the controller with the given sensor/ motornumber
   * Must NORMALLY be called before use. For all ControllerAdapters
   * call first AbstractControllerAdapter::init(sensornumber,motornumber)
   * if you overwrite this method
   */
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
    controller->init(sensornumber,motornumber);
  }

  /** @return Number of sensors the controller
      was initialised with or 0 if not initialised */
  virtual int getSensorNumber() const { return controller->getSensorNumber();}

  /** @return Number of motors the controller
      was initialised with or 0 if not initialised */
  virtual int getMotorNumber() const {  return controller->getMotorNumber();}

  /** performs one step (includes learning).
      Calculates motor commands from sensor inputs.
      @param sensors sensors inputs scaled to [-1,1]
      @param sensornumber length of the sensor array
      @param motors motors outputs. MUST have enough space for motor values!
      @param motornumber length of the provided motor array
  */
  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber) {
    controller->step(sensors, sensornumber, motors,  motornumber);
  }

  /** performs one step without learning.
      @see step
  */
  virtual void stepNoLearning(const sensor* sensors , int sensornumber,
                              motor* motors, int motornumber) {
    controller->stepNoLearning(sensors,sensornumber,motors,motornumber);
  }

  /****************************************************************************/
  /*        END methods of AbstractController                                             */
  /****************************************************************************/

  /****************************************************************************/
  /*        BEGIN methods of Configurable                                               */
  /****************************************************************************/

  // nothing needed to be overwrited

  /****************************************************************************/
  /*        END methods of Configurable                                                  */
  /****************************************************************************/

  /****************************************************************************/
  /*        BEGIN methods of Inspectable                                                  */
  /****************************************************************************/

  // nothing needed to be overwrited

  /****************************************************************************/
  /*        END methods of Inspectable                                                   */
  /****************************************************************************/

  /****************************************************************************/
  /*        BEGIN methods of Storable                                               */
  /****************************************************************************/

  /********* STORABLE INTERFACE ******/
  /// @see Storable
  virtual bool store(FILE* f) const {
    return controller->store(f);
  }

  /// @see Storable
  virtual bool restore(FILE* f) {
    return controller->restore(f);
  }


  /****************************************************************************/
  /*        END methods of Storable                                                    */
  /****************************************************************************/


protected:
  AbstractController* controller; // the controller for the adapter to handle

};

#endif

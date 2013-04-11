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
#include "abstractmulticontroller.h"

using namespace std;

AbstractMultiController::AbstractMultiController(AbstractController* controller, const std::string& name, const std::string& revision) :
  AbstractControllerAdapter(controller, name, revision) {
  this->controllerNameList.push_back(controller->getName());
}

AbstractMultiController::~AbstractMultiController() {
}

void AbstractMultiController::addPassiveController(AbstractController* passiveController) {
  if (passiveController != 0) {
    controllerList.push_back(passiveController);
    addConfigurable(passiveController);
    addInspectable(passiveController);
    controllerNameList.push_back(passiveController->getName());
  } // otherwise return
}

void AbstractMultiController::init(int sensornumber, int motornumber, RandGen* randGen /* = 0 */) {
  AbstractControllerAdapter::init(sensornumber, motornumber);
  // init the other controllers
  for (std::list<AbstractController*>::iterator i = controllerList.begin(); i != controllerList.end(); i++) {
    if (*i)
      (*i)->init(sensornumber, motornumber);
  }
}

/****************************************************************************/
/*        BEGIN methods of Inspectable                                                  */
/****************************************************************************/

/****************************************************************************/
/*        END methods of Inspectable                                                   */
/****************************************************************************/

/****************************************************************************/
/*        BEGIN methods of Storeable                                                   */
/****************************************************************************/

/** stores the object to the given file stream (binary).
 */
bool AbstractMultiController::store(FILE* f) const {
  return controller->store(f);
  // TODO: store the other controllers
  // and store values from multicontroller itself
}

/** loads the object from the given file stream (binary).
 */
bool AbstractMultiController::restore(FILE* f) {
  return controller->restore(f);
  // TODO: restore the other controllers
  // and restore values from multicontroller itself
}

/****************************************************************************/
/*        END methods of Storeable                                                      */
/****************************************************************************/

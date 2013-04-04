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
#include "oneactivemultipassivecontroller.h"

#include <assert.h>


OneActiveMultiPassiveController::OneActiveMultiPassiveController(AbstractController* controller, const std::string& name, const std::string& revision)
                : AbstractMultiController(controller, name, revision) {}

OneActiveMultiPassiveController::~OneActiveMultiPassiveController() {}

/****************************************************************************/
/*        AbstractMultiController should implement the following classes:                */
/*        AbstractController, Configurable, Inspectable, Storeable                    */
/****************************************************************************/


/****************************************************************************/
/*        BEGIN methods of AbstractController                                         */
/****************************************************************************/

void OneActiveMultiPassiveController::init(const int sensornumber, const int motornumber,
                                           RandGen* randGen) {
        // call the same method of super class
        AbstractMultiController::init( sensornumber,motornumber, randGen);
        // allocate memory for passiveMotors
        this->passiveMotors = (motor*) malloc(sizeof(motor) * motornumber);
}

void OneActiveMultiPassiveController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
        assert(controller);
        // make normal step of the active controller
        // then make step of all passive controllers
        controller->step(sensors,sensornumber,motors,motornumber);

        for(std::list<AbstractController*>::iterator i=controllerList.begin(); i != controllerList.end(); i++){
                (*i)->step(sensors,sensornumber,passiveMotors,motornumber);
        }
}

void OneActiveMultiPassiveController::stepNoLearning(const sensor* sensors , int sensornumber, motor* motors, int motornumber){
        assert(controller);
        // make normal step of the active controller
        // then make step of all passive controllers
        controller->stepNoLearning(sensors,sensornumber,motors,motornumber);
        for(std::list<AbstractController*>::iterator i=controllerList.begin(); i != controllerList.end(); i++){
                (*i)->stepNoLearning(sensors,sensornumber,passiveMotors,motornumber);
        }
}

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


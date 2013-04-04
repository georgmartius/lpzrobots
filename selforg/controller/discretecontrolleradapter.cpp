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

/**
Adapter class for robot controller.
The controller gets a number of input sensor values each timestep and has to
generate a number of output motor values.

These sensor and motor values are discretesized with this adapter.

Use this adapter between an agent and a controller to get discrete
sensor and motor values.

The sensor values are hand over to the controller from the agent,
in the other direction the motor values of the controller are hand over
to the agent.

For more details about controllers,
see AbstractController and all implementing classes.

        @author Frank Güttler <frankguettler@gmx.de>
*/
#include "discretecontrolleradapter.h"
#include <controller_misc.h>

using namespace std;

DiscreteControllerAdapter::DiscreteControllerAdapter(AbstractController* controller, const std::string& name, const std::string& revision)
 : AbstractControllerAdapter(controller, name, revision)
{
        // avoid division by zero
        this->sensorIntervalCount=1;
        this->motorIntervalCount=1;

        this->firstStep=true;

        // There are three possibilities:
        // 1. automaticRange=true
        // --> mapping is always disabled
        // 2. automaticRange=false, mapToInterval=true
        // --> real (found) range is mapped to specified range
        // 3. automaticRange=false, mapToInterval=false
        // --> no mapping, no range adjustment, values outside the specified
        //        range are set to minRange respectively maxRange
        this->automaticMotorRange=true;
        this->automaticSensorRange=true;
        this->mapToSensorInterval=false;
        this->mapToSensorInterval=false;
}


DiscreteControllerAdapter::~DiscreteControllerAdapter(){} // nothing to do


void DiscreteControllerAdapter::setIntervalCount(int intervalCount){
        this->sensorIntervalCount=intervalCount;
        this->motorIntervalCount=intervalCount;
}


void DiscreteControllerAdapter::setSensorIntervalCount(int sensorIntervalCount) {
        this->sensorIntervalCount=sensorIntervalCount;
}


void DiscreteControllerAdapter::setMotorIntervalCount(int motorIntervalCount) {
        this->motorIntervalCount=motorIntervalCount;
}

void DiscreteControllerAdapter::setIntervalRange(double minRange, double maxRange, bool mapToInterval) {
        this->setMotorIntervalRange(minRange,maxRange,mapToInterval);
        this->setSensorIntervalRange(minRange,maxRange,mapToInterval);
}

void DiscreteControllerAdapter::setMotorIntervalRange(double minMotorRange, double maxMotorRange, bool mapToMotorInterval) {
        this->automaticMotorRange=false;
        this->mapToMotorInterval=mapToMotorInterval;
        this->minMotorRange=minMotorRange;
        this->maxMotorRange=maxMotorRange;
}

void DiscreteControllerAdapter::setSensorIntervalRange(double minSensorRange, double maxSensorRange, bool mapToSensorInterval) {
        this->automaticSensorRange=false;
        this->mapToSensorInterval=mapToSensorInterval;
        this->minSensorRange=minSensorRange;
        this->maxSensorRange=maxSensorRange;
}


/***************************************************************************/
/* BEGIN: forwarding methods of AbstractController                         */
/***************************************************************************/

void DiscreteControllerAdapter::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
        this->doDiscretisizeSensorValues( sensors,sensornumber);
        controller->step(discreteSensors,sensornumber,motors,motornumber);
        this->doDiscretisizeMotorValues( motors,motornumber);
}

void DiscreteControllerAdapter::stepNoLearning(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
        this->doDiscretisizeSensorValues( sensors,sensornumber);
        this->controller->stepNoLearning(discreteSensors,sensornumber,motors,motornumber);
        this->doDiscretisizeMotorValues( motors,motornumber);
}

void DiscreteControllerAdapter::init(int sensornumber, int motornumber, RandGen* randGen) {
  // allocate memory for discreteSensors
  this->discreteSensors = (sensor*) malloc(sizeof(sensor) * sensornumber);
  // register sensors as inspectable values
  for (int i=0; i<sensornumber; i++)
    addInspectableValue(string("x_dis[").append(itos(i)).append("]"), &discreteSensors[i], "discretisized sensor value");
        AbstractControllerAdapter::init(sensornumber, motornumber, randGen);
}


/***************************************************************************/
/* END: forwarding methods of AbstractController                           */
/***************************************************************************/

void DiscreteControllerAdapter::doDiscretisizeSensorValues(const sensor* sensors, int sensornumber) {
        // first find lowest and highest sensor range
        this->findMinAndMaxSensorRange( sensors,sensornumber);
        // second find lowest and highest sensor values
        this->findMinAndMaxSensorValues( sensors,sensornumber);

        for (int i=0;i<sensornumber;i++) {
                discreteSensors[i]=this->discretisizeValue( sensors[i],automaticSensorRange,minSensorRange,maxSensorRange, minSensorValue, maxSensorValue,sensorIntervalCount,mapToSensorInterval);
        }
}

void DiscreteControllerAdapter::doDiscretisizeMotorValues(motor* motors, int motornumber) {
        // first find lowest and highest motor range
        this->findMinAndMaxMotorRange( motors, motornumber);
        // second find lowest and highest motor values
        this->findMinAndMaxMotorValues( motors, motornumber);

        for (int i=0;i<motornumber;i++) {
                motors[i]=this->discretisizeValue( motors[i],automaticMotorRange,minMotorRange,maxMotorRange, minMotorValue, maxMotorValue,motorIntervalCount,mapToMotorInterval);
        }

}


void DiscreteControllerAdapter::findMinAndMaxSensorRange(const sensor* sensors, int sensornumber) {
        if (this->firstStep) {
                this->minSensorRange=sensors[0];
                this->maxSensorRange=sensors[0];
        }
        if (this->automaticSensorRange) {
                for (int i=0;i<sensornumber;i++) {
                        if (sensors[i]<this->minSensorRange)
                                this->minSensorRange=sensors[i];
                        if (sensors[i]>this->maxSensorRange)
                                this->maxSensorRange=sensors[i];
                }
        }
}


void DiscreteControllerAdapter::findMinAndMaxMotorRange(motor* motors, int motornumber) {
        if (this->firstStep) {
                this->minMotorRange=motors[0];
                this->maxMotorRange=motors[0];
        }
        if (this->automaticMotorRange) {
                for (int i=0;i<motornumber;i++) {
                        if (motors[i]<this->minMotorRange)
                                this->minMotorRange=motors[i];
                        if (motors[i]>this->maxMotorRange)
                                this->maxMotorRange=motors[i];
                }
        }
}



void DiscreteControllerAdapter::findMinAndMaxSensorValues(const sensor* sensors, int sensornumber) {
        if (this->firstStep) {
                this->minSensorValue=sensors[0];
                this->maxSensorValue=sensors[0];
        }
        if (this->mapToSensorInterval) {
                for (int i=0;i<sensornumber;i++) {
                        if (sensors[i]<this->minSensorValue)
                                this->minSensorValue=sensors[i];
                        if (sensors[i]>maxSensorValue)
                                maxSensorValue=sensors[i];
                }
        }
}

void DiscreteControllerAdapter::findMinAndMaxMotorValues(motor* motors, int motornumber) {
        if (this->firstStep) {
                this->minMotorValue=motors[0];
                this->maxMotorValue=motors[0];
                this->firstStep=false;
        }
        if (this->mapToMotorInterval) {
                for (int i=0;i<motornumber;i++) {
                        if (motors[i]<minMotorValue)
                                this->minMotorValue=motors[i];
                        if (motors[i]>this->maxMotorValue)
                                this->maxMotorValue=motors[i];
                }
        }
}


double DiscreteControllerAdapter::discretisizeValue(double valueToDiscretisize, bool automaticRange, double minRange, double maxRange, double minValue, double maxValue, int intervalCount, bool mapToInterval) {
        // if automatic range is enabled, then the new range is already set.
        // This means, nothing more has to be considered.
        if (!automaticRange) {
                if (mapToInterval) {
                        // then the values will be mapped:
                        // minValue eqv.to minRange,
                        // maxValue eqv.to maxRange
                        valueToDiscretisize=(valueToDiscretisize-minValue)/(maxValue-minValue)
                                                                *(maxRange-minRange)+minRange;
                } else { // no mapping, ensure that value is in range
                        if (valueToDiscretisize<minRange)
                                valueToDiscretisize=minRange;
                        if (valueToDiscretisize>maxRange)
                                valueToDiscretisize=maxRange;
                }
        }
        // value is now in range
        valueToDiscretisize=roundValue((maxRange-valueToDiscretisize)*intervalCount/(maxRange-minRange));
        // we know now the interval
        valueToDiscretisize=valueToDiscretisize/(intervalCount)*(maxRange-minRange)+minRange;
        return -valueToDiscretisize;
}

double DiscreteControllerAdapter::roundValue(double valueToRound) {
        return (int)(valueToRound<0?valueToRound-.5:valueToRound+.5);
}

/***************************************************************************/
/* BEGIN: forwarding methods of Inspectable                                */
/***************************************************************************/

/***************************************************************************/
/* END: forwarding methods of Inspectable                                  */
/***************************************************************************/


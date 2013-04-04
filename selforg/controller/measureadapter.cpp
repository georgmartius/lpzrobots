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
#include "measureadapter.h"
#include <sstream>

MeasureAdapter::MeasureAdapter(AbstractController* controller, const std::string& name, const std::string& revision ) :
  AbstractControllerAdapter(controller, name, revision) {
  st = new StatisticTools("MeasureAdapter's ST");
  addCallbackable(st);
  addInspectable(st);
}

MeasureAdapter::~MeasureAdapter() {
  if (st)
    free(st);
}

std::list<ComplexMeasure*> MeasureAdapter::addSensorComplexMeasure(char* measureName, ComplexMeasureMode mode,
    int numberBins, int stepSize) {
  std::list<ComplexMeasure*> cmlist;
  if (initialized) {
    for (int i = 0; i < controller->getSensorNumber(); i++) {
      std::stringstream name;
      name << measureName << " [" << i << "]";
      ComplexMeasure* cm = new ComplexMeasure(name.str().c_str(), mode, numberBins);
      switch (mode) {
        case ENT:
          cm->addObservable(sensorValues[i], -1.0, 1.0);
          cm->setStepSize(stepSize);
          break;
        case ENTSLOW:
          cm->addObservable(sensorValues[i], -1.0, 1.0);
          break;
        default:
          break;
      }
      st->addMeasure(cm);
      cmlist.push_back(cm);
    }
  } else {
    std::cerr << "ERROR: The method" << std::endl
        << "       addSensorComplexMeasure(char* measureName, ComplexMeasureMode mode,int numberBins, int stepSize)"
        << std::endl << "must be called before the initialization of the Agent!" << std::endl;
    exit(1);
  }
  return cmlist;
}

/****************************************************************************/
/*        BEGIN methods of AbstractController                                 */
/****************************************************************************/

void MeasureAdapter::init(const int sensornumber, const int motornumber, RandGen* randGen) {
  // call the same method of super class
  AbstractControllerAdapter::init(sensornumber, motornumber, randGen);
  // allocate memory for controlled Motors and sensors
  this->motorValues = (motor*) malloc(sizeof(motor) * motornumber);
  this->sensorValues = (sensor*) malloc(sizeof(sensor) * sensornumber);
}

void MeasureAdapter::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
  AbstractControllerAdapter::step(sensors, sensornumber, motors, motornumber);
  // store motor and sensor values in motorValues and sensorValues
  for (int i = 0; i < motornumber; i++)
    motorValues[i] = motors[i];
  for (int i = 0; i < sensornumber; i++)
    sensorValues[i] = sensors[i];
  callBack();
}

void MeasureAdapter::stepNoLearning(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
  AbstractControllerAdapter::stepNoLearning(sensors, sensornumber, motors, motornumber);
  // store motor and sensor values in motorValues and sensorValues
  for (int i = 0; i < motornumber; i++)
    motorValues[i] = motors[i];
  for (int i = 0; i < sensornumber; i++)
    sensorValues[i] = sensors[i];
  callBack();
}

/****************************************************************************/
/*        END methods of AbstractController                                   */
/****************************************************************************/

/****************************************************************************/
/*        BEGIN methods of Storeable                                                   */
/****************************************************************************/

/****************************************************************************/
/*        END methods of Storeable                                                      */
/****************************************************************************/

/****************************************************************************/
/*        BEGIN methods of Inspectable                                        */
/****************************************************************************/

/****************************************************************************/
/*        END methods of Inspectable                                          */
/****************************************************************************/

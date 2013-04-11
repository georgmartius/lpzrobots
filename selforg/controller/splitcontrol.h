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
#ifndef __SPLITCONTROL_H
#define __SPLITCONTROL_H

#include <selforg/abstractcontroller.h>
#include <selforg/onecontrollerperchannel.h>

#include <vector>
#include <functional>


/**
 * class for using multiple controller with one robot. The connection is flexible.
 * The controller are generated on the fly with a generator object.
 *
 */
class SplitControl : public AbstractController {
public:

  struct Assoziation {
    Assoziation();
    void addSensorIdx(int s) { sensors.push_back(s);}
    void addMotorIdx(int m)  { motors.push_back(m);}

    std::list<int> sensors;
    std::list<int> motors;
  };
  typedef std::vector<Assoziation> Assoziations;

  /** @param controllerGenerator generator object for controller
      @param assoziations list decribing which sensors and motors are connected to each controller
      @param controllerName name
      @param numCtrlCreateBeforeInit number of controller that are generated before the init function is called. Useful if they should be put into the inspectable list of the agent
      @param numContextSensors number of context sensors (counted from the end)
       passed to all controllers
   */
  SplitControl(ControllerGenerator* controllerGenerator,
               const Assoziations& assoziations,
               std::string controllerName,
               int numCtrlCreateBeforeInit = 1,
               int numContextSensors = 0
               );

  virtual ~SplitControl();


  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber);

  virtual void stepNoLearning(const sensor* sensors , int sensornumber,
                                    motor* motors, int motornumber);

  virtual int getSensorNumber() const {return sensornumber; }
  virtual int getMotorNumber() const  {return motornumber; }

  /*********** STORABLE **************/

  virtual bool store(FILE* f) const {return false;}

  virtual bool restore(FILE* f) {return false;}


  virtual std::vector<AbstractController*> getControllers() const { return ctrl;}

protected:
  std::vector<AbstractController*> ctrl;


  ControllerGenerator* controllerGenerator;
  Assoziations assoz;
  int numCtrlCreateBeforeInit;
  int numContextSensors;
  int motornumber;
  int sensornumber;
  double* sensorbuffer;
  double* motorbuffer;
};

#endif

/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   $Log$
 *   Revision 1.2  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.1  2005/12/21 00:16:59  robot7
 *   this is a pretty bad controller - but at least it works pretty well with
 *   the nimm4-robot - so its good enough for demo reasons (thus the name).
 *
 *   Revision 1.6  2005/11/15 14:23:44  robot3
 *   raceground testet
 *
 *   Revision 1.5  2005/10/17 13:17:10  martius
 *   converted to new list's
 *
 *   Revision 1.4  2005/10/17 13:07:57  robot3
 *   std lists included
 *
 *   Revision 1.3  2005/10/17 13:05:47  robot3
 *   std lists included
 *
 *   Revision 1.2  2005/08/09 11:06:30  robot1
 *   camera module included
 *
 *   Revision 1.1  2005/08/08 11:14:54  robot1
 *   simple control for moving robot with keyboard
 *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef __DEMO_CONTROLLER_H
#define __DEMO_CONTROLLER_H


#include <stdio.h>
#include "abstractcontroller.h"


/**
 * class for robot controll with sine and cosine
 *
 *
 */
class DemoController : public AbstractController {
public:

  DemoController();

  /** initialisation of the controller with the given sensor/ motornumber
      Must be called before use.
  */
  virtual void init(int sensornumber, int motornumber);

  /// returns the name of the object (with version number)
  //  virtual constparamkey getName() const {return name; }
  virtual paramkey getName() const {return name; }

  /// @return Number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const {return number_sensors;}


  /// @return Number of motors the controller was initialised with or 0 if not initialised
  virtual int getMotorNumber() const {return number_motors;}

  /** performs one step (includes learning).
      Calculates motor commands from sensor inputs.
      @param sensor sensors inputs scaled to [-1,1]
      @param sensornumber length of the sensor array
      @param motor motors outputs. MUST have enough space for motor values!
      @param motornumber length of the provided motor array
  */
  virtual void step(const sensor* sensors, int sensornumber,
                    motor* motors, int motornumber);
  /** performs one step without learning.
      @see step
  */
  virtual void stepNoLearning(const sensor* , int number_sensors,
                              motor* , int number_motors);

  /** The list of the names of all internal parameters given by getInternalParams().
      @param: keylist (do NOT free it! It is a pointer to an internal structure)
      @return: length of the lists
   */
  virtual list<iparamkey> getInternalParamNames() const {return list<iparamkey>();}

  /** The list of the names of all internal parameters given by getInternalParams().
   */
  virtual list<iparamval> getInternalParams() const {return list<iparamval>();}

  virtual paramval getParam(const paramkey& key, bool traverseChildren=true) const;
  virtual bool setParam(const paramkey& key, paramval val, bool traverseChildren=true);
  virtual paramlist getParamList() const ;

  /** Initialises the registers the given callback functions.
      @param handling() is called every step that the camera gets new position
      and view.
  */
//   virtual void setCameraHandling(void (*handling)());
//  virtual void setCameraHandling() const;




protected:
  string name;
  int t;
  int number_sensors;
  int number_motors;
  int cameraHandlingDefined;

  paramval velocity;
  paramval leftRightShift;
  paramval decreaseFactorVelocity;
  paramval decreaseFactorShift;

};

#endif

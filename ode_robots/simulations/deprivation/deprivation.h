/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2008-11-14 11:23:05  martius
 *   added centered Servos! This is useful for highly nonequal min max values
 *   skeleton has now also a joint in the back
 *
 *   Revision 1.4  2008/05/30 11:58:27  martius
 *   use cmath instead of math.h
 *
 *   Revision 1.3  2006/07/14 12:23:58  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.5  2006/07/10 13:05:15  martius
 *   NON-COMMERICAL LICENSE added to controllers
 *
 *   Revision 1.1.2.4  2006/07/10 11:59:23  martius
 *   Matrixlib now in selforg
 *   no namespace std in header files
 *
 *   Revision 1.1.2.3  2006/03/29 15:11:19  martius
 *   damping is now in invertmotornstep
 *
 *   Revision 1.1.2.2  2006/02/20 10:56:21  martius
 *   lowered damping
 *
 *   Revision 1.1.2.1  2006/02/14 10:27:11  martius
 *   investigation about deprivation of worldmodel
 *
 *   Revision 1.1.2.1  2006/02/08 15:14:49  martius
 *   new version of proactive which uses direct H feeding
 *
 *
 ***************************************************************************/
#ifndef __DEPRIVATION_H
#define __DEPRIVATION_H

#include <selforg/invertmotornstep.h>
#include <selforg/onelayerffnn.h>

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>

/**
 * robot controller for self-organized behaviour
 * just like invertmotornstep,
 *  just that is enables us to execute the same motor commands for a long time
 */
class Deprivation : public InvertMotorNStep {
public:
  /// is called with current motor values and returns new motor values
  typedef matrix::Matrix (*MotorCallback)(const matrix::Matrix& y);

  /// is called with current controller matrix C and bias H which can be altered
  typedef void (*ControllerCallback)(matrix::Matrix& C, matrix::Matrix& H);

  /**
   */
  Deprivation(MotorCallback motorCallback, ControllerCallback controllerCallback =0,
              const InvertMotorNStepConf& conf = getDefaultConf())
    : InvertMotorNStep(conf), motorCallback(motorCallback), controllerCallback(controllerCallback) {
    assert(motorCallback);
    // prepare name;
    setName("$RCSfile$");
    setRevision("$Revision$");
    useExternal=false;
  }

  virtual ~Deprivation(){
  }

  virtual void setExternalControlMode(bool useExternal){
    this->useExternal=useExternal;
    if(controllerCallback && initialised){
      controllerCallback(C, H);
    }
  }
  virtual bool getExternalControlMode(){
    return useExternal;
  }


protected:
  /// overloaded
  virtual void learnController(){
    if(!useExternal){
      InvertMotorNStep::learnController();
    }
  }

  /// calculate controller outputs (only of nop external value is set)
  /// @param x_smooth smoothed sensors Matrix(number_channels,1)
  virtual matrix::Matrix calculateControllerValues(const matrix::Matrix& x_smooth){
    matrix::Matrix y = InvertMotorNStep::calculateControllerValues(x_smooth);
    if(useExternal){
      return motorCallback(y);
    }
    else
      return y;
  };

protected:
  bool useExternal;

  MotorCallback motorCallback;
  ControllerCallback controllerCallback;

};

#endif

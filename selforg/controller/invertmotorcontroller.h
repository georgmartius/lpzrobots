/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/

#ifndef __INVERTMOTORCONTROLLER_H
#define __INVERTMOTORCONTROLLER_H

#include "homeokinbase.h"
#include "controller_misc.h"
#include <stdlib.h>
#include <string.h>


/**
 * Extended HomeokinBase class (still abstract) for robot controller work in motorspace
 * and use possibly many steps and adaptive learning rate
 *
 * Parameters like dampA, adaptrate, noiseY...
 */
class InvertMotorController : public HomeokinBase {
public:
  InvertMotorController( unsigned short buffersize ,
                         const std::string& name, const std::string& revision)
    : HomeokinBase(buffersize,name, revision){
    addParameterDef("dampA",&dampA,0 );
    addParameterDef("adaptrate",&adaptRate,0.000);
    addParameterDef("nomupdate",&nomUpdate,0.005);
    addParameterDef("desens",&desens,0);
    addParameterDef("steps",&steps,1);
    addParameterDef("zetaupdate",&zetaupdate,0);
    addParameterDef("noiseY",&noiseY,0);
    addParameterDef("noiseB",&noiseB,0.001);
    addParameterDef("teacher",&teacher,5);
    addParameterDef("relativeE",&relativeE,0);
  }

protected:
  paramval desens;
  paramval steps; ///< number of timesteps is used for controller learning
  paramval zetaupdate;
  paramval dampA; ///< damping term for model (0: no damping, 0.1 high damping)
  parambool relativeE; ///< if not 0: a relative error signal is used (xsi is normalised in respect to |y|)

  paramval adaptRate;  ///< adaptation rate for learning rate adaptation
  paramval nomUpdate;  ///< nominal update of controller in respect to matrix norm
  paramval noiseB;     ///< size of the additional noise for model bias B
  paramval noiseY;     ///< size of the additional noise for motor values
  paramval teacher;    ///< factor for teaching signal

};

#endif

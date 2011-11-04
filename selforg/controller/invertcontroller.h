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
#ifndef __INVERTCONTROLLER_H
#define __INVERTCONTROLLER_H

#include "abstractcontroller.h"
#include <stdlib.h>
#include <string.h>

/**
 * Abstract class (interface) for robot controller that use direct matrix inversion and
 * simple one layer networks.
 * 
 * Implements standart parameters: eps, rho, mu, stepnumber4avg, stepnumber4delay
 */
class InvertController : public AbstractController {
public:
  InvertController(const std::string& name, const std::string& revision)
    : AbstractController(name, revision){
    addParameterDef("eps",&eps,0.3,0,10,"learning rate");
    addParameterDef("rho",&rho,0,0,10);
    addParameterDef("s4delay",&s4delay,1,1,10,"time delay of actions and caused sensor values");
    addParameterDef("s4avg",&s4avg,1,1,50,"time window for sensor values averaging");
    addParameterDef("delta",&delta,0.01,0,1);
    addParameterDef("factor_a",&factor_a,1.0,0,10,"learning rate factor for world model");
    addParameterDef("desens",&desens,0,0,1,"desensitization");
    addParameterDef("number_it",&number_it,0,0,10);
    addParameterDef("epsilon_it",&epsilon_it,0,0,10);
    addParameterDef("damping_c",&damping_c,0,0,.1,"damping value for controller values");
  }

protected:
  paramval eps;
  paramval rho;
  paramval desens;
  paramval s4delay;
  paramval s4avg;
  paramval factor_a;
  paramval number_it;
  paramval epsilon_it;
  paramval delta;
  paramval damping_c;
 
};

#endif

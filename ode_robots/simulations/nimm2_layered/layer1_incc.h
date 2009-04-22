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
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2009-04-22 14:39:02  guettler
 *   moved layeredcontroller, layer2_incc and layer1_incc to ode_robots/simulations/nimm2_hebb and nimm2_layered
 *
 *   Revision 1.4  2008/05/30 11:58:27  martius
 *   use cmath instead of math.h
 *
 *   Revision 1.3  2008/04/17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.2  2007/10/09 16:47:13  fhesse
 *   further testing
 *
 *   Revision 1.1  2007/10/08 20:15:33  fhesse
 *   initial version of layered controller
 *
 *                             *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef __LAYER1_INCC_H
#define __LAYER1_INCC_H

#include "invertnchannelcontroller.h"

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>



/**
 * class for robot controller that uses georg's matrixlib for 
 *  direct matrix inversion for n channels 
 * (simple one layer networks)
 * This derivative is used as a basic layer of a two layered control structure.
 * 
 */
class Layer1_INCC : public InvertNChannelController {

public:
  Layer1_INCC(int _buffersize, bool _update_only_1=false);

  //  virtual ~InvertNChannelControllerHebbH();

  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual void learn(const matrix::Matrix& x_delay, const matrix::Matrix& y_delay);

  virtual matrix::Matrix getH(); 
  virtual void setH(matrix::Matrix tmp); 
  virtual void addtoH(matrix::Matrix tmp);

  virtual matrix::Matrix getdH(); 




protected:
  matrix::Matrix h_update;

};

#endif


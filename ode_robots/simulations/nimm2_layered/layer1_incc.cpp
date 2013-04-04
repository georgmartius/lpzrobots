/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
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
 *   Revision 1.2  2009-08-05 23:25:57  martius
 *   adapted small things to compile with changed Plotoptions
 *
 *   Revision 1.1  2009/04/22 14:39:02  guettler
 *   moved layeredcontroller, layer2_incc and layer1_incc to ode_robots/simulations/nimm2_hebb and nimm2_layered
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
 *                           *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include <selforg/controller_misc.h>
#include "layer1_incc.h"
using namespace matrix;
using namespace std;

Layer1_INCC::Layer1_INCC(int _buffersize, bool _update_only_1/*=false*/)
  : InvertNChannelController(_buffersize, _update_only_1 ){

};
/*
Layer1_INCC::~Layer1_INCC(){

}
*/

void Layer1_INCC::init(int sensornumber, int motornumber, RandGen* randGen){
  InvertNChannelController::init(sensornumber, motornumber, randGen);
  h_update.set(number_channels,1);
}



/// learn values h,C,A
void Layer1_INCC::learn(const Matrix& x_delay, const Matrix& y_delay){

  Matrix C_update(number_channels,number_channels);

  double E_0 = calculateE(x_delay,  y_delay);

  // calculate updates for h,C,A
  for (unsigned int i = 0; i < number_channels; i++)
  {
      h.val(i,0) += delta;
      h_update.val(i,0) = -eps * (calculateE(x_delay, y_delay) - E_0) / delta;
      //h_update[i] = -2*eps *eita[i]*eita[i]*g(y_delay[i]);
      h.val(i,0) -= delta;
 }

  // only weights of one channel adapted in one time step
  unsigned int start=0;
  unsigned int end=number_channels;
  if(update_only_1) {
    start = t%number_channels;
    end = (t%number_channels) + 1;
  }
  for (unsigned int i = start; i < end; i++){
      for (unsigned int j = 0; j < number_channels; j++)
        {
          C.val(i,j) += delta;
          C_update.val(i,j)  = - eps *  (calculateE(x_delay, y_delay) - E_0) / delta ;
          C_update.val(i,j) -= damping_c*C.val(i,j) ;  // damping term
          C.val(i,j) -= delta;
          //A[i][j] += delta;
          //A_update[i][j] = -eps * (calculateE(x_delay, y_delay,eita) - E_0) / delta;
          //A[i][j] -= delta;
        }
    }
  // apply updates to h,C
  h += h_update.map(squash);
  C += C_update.map(squash);
};



Matrix Layer1_INCC::getH(){
  return h;
}

void Layer1_INCC::setH(Matrix tmp){
  h=tmp;
}

void Layer1_INCC::addtoH(Matrix tmp){
  h+=tmp;
}

Matrix Layer1_INCC::getdH(){
  return h_update;
}

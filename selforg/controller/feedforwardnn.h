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
#ifndef __FEEDFORWARDNN_H
#define __FEEDFORWARDNN_H

#include "invertablemodel.h"
#include "controller_misc.h"
#include "regularisation.h"

#include <cmath>

/// activation function type: input: membrane potential
typedef double (*ActivationFunction) (double);
/** inverse of Activation function with respect to some membrane potential
    and a certain output error.
 */
typedef double (*InvActivationFunction) (double, double);

/// abstract class (interface) for feed forward rate based neural networks
class FeedForwardNN : public InvertableModel {
 public:
  // 20110317, guettler: disabled default constructor since it is not needed and would cause difficulties
  //FeedForwardNN() {}
  FeedForwardNN(const std::string& name, const std::string& revision)
    : InvertableModel(name, revision){};
  virtual ~FeedForwardNN(){};

  /// damps the weights and the biases by multiplying (1-damping)
  virtual void damp(double damping) =0 ;

  /******** Activation functions and there derivative
            and inversion with certain output shift (regularised) */
  static double linear(double z)   { return z;}
  static double dlinear(double )   { return 1;}
  static double invlinear(double z, double xsi) { return xsi;}

  static double tanh(double z)     { return ::tanh(z); }
  static double dtanh(double z)    { double k = ::tanh(z); return 1-k*k; }
  static double invtanh(double z, double xsi) { return g_s_expand2(z,xsi)*xsi; }

  // clipped tanh
  static double tanhc(double z)     { return ::tanh(z); }
  static double dtanhc(double z) { double k = ::tanh(clip(z, -3.0, 3.0)); return 1-k*k; }

  // regularised tanh
  static double tanhr(double z)     { return ::tanh(z); }
  // static double dtanhr(double z)    { double k = tanh(z); return 1.01-k*k; }
  static double dtanhr(double z) { return 1.0/(1.0+z*z); }


  static double sigmoid(double z)  { return 1/(1+exp(-z)); }
  static double dsigmoid(double z) { double k = sigmoid(clip(z, -3.0, 3.0)); return k*(1-k); }
  static double invsigmoid(double z, double xsi) { return 1/(0.01+dsigmoid(z))*xsi;}
};


#endif

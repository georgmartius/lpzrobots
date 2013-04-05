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
#ifndef __NETWORKLAYER_H
#define __NETWORKLAYER_H

#include <vector>

#include "feedforwardnn.h"

class Layer : public Storeable {
public:
  /** constructor for Layer.
      The derivative and inverse of activation function is derived automatically.
      @param size number neurons
      @param factor_bias size of bias neuron , if 0 no bias is used
      @param actfun activation function. see also FeedForwardNN
  */
  Layer(int size, double factor_bias=0.1,
        ActivationFunction actfun = FeedForwardNN::linear);

  /** obsolete, use the other constructor for Layer.
      @param size number neurons
      @param factor_bias size of bias neuron , if 0 no bias is used
      @param actfun activation function. see also FeedForwardNN
      @param dactfun derivative of activation function (should be consistent with actfun)
  */
  Layer(int size, double factor_bias,
        ActivationFunction actfun,
        ActivationFunction dactfun) {
    fprintf(stderr, "%s %s\n", "Layer::Layer(): this contructor is obsolete! ",
            "Please use the one without dactfun now!\n");
    exit(1);
  }

  /***STOREABLE ******/
  /// stores the layer binary into file stream
  bool store(FILE* f) const;
  /// restores the layer binary from file stream
  bool restore(FILE* f);

  /// sets the activation function of the layer
  void setActFun(ActivationFunction actfun);

  int size;
  double factor_bias;
  ActivationFunction actfun;  ///< callback activation function
  ActivationFunction dactfun; ///< first derivative of the activation function
  InvActivationFunction invactfun; ///< inversion of activation function

  // prints the Layer data-structure
  friend std::ostream& operator<<(std::ostream& , const Layer&);
};

#endif

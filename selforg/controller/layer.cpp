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

#include "layer.h"
#include "regularisation.h"

using namespace matrix;
using namespace std;

const char* actFun2String(ActivationFunction actfun) {
  if(actfun == FeedForwardNN::linear) {
    return "linear";
  } else if (actfun == FeedForwardNN::sigmoid) {
    return "sigmoid";
  }else if (actfun == FeedForwardNN::tanh) {
    return "tanh";
  }else if (actfun == FeedForwardNN::tanhc) {
    return "tanhc";
  }else if (actfun == FeedForwardNN::tanhr) {
    return "tanhr";
  }else {
    fprintf(stderr, "unknown activation function! Expand the cases in layer.cpp:actFun2String");
    return "unknown";
  }
}

Layer::Layer(int size, double factor_bias,
             ActivationFunction actfun)
  : size(size), factor_bias(factor_bias), actfun(actfun) {
  setActFun(actfun);
}

void Layer::setActFun(ActivationFunction actfun){
  this->actfun=actfun;
  if(actfun == FeedForwardNN::linear) {
    dactfun=FeedForwardNN::dlinear;
    invactfun=FeedForwardNN::invlinear;
  } else if (actfun == FeedForwardNN::sigmoid) {
    dactfun=FeedForwardNN::dsigmoid;
    invactfun=FeedForwardNN::invsigmoid;
  }else if (actfun == FeedForwardNN::tanh) {
    dactfun=FeedForwardNN::dtanh;
    invactfun=FeedForwardNN::invtanh;
  }else if (actfun == FeedForwardNN::tanhc) {
    dactfun=FeedForwardNN::dtanhc;
    invactfun=FeedForwardNN::invtanh;
  }else if (actfun == FeedForwardNN::tanhr) {
    dactfun=FeedForwardNN::dtanhr;
    invactfun=FeedForwardNN::invtanh;
  }else {
    fprintf(stderr, "unknown activation function! expand cases in Layer::setActFun and restore()!");
    exit(1);
  }
}

bool Layer::store(FILE* f) const {
        fprintf(f,"%i %g %s\n", size, factor_bias, actFun2String(actfun));
        return true;
}

bool Layer::restore(FILE* f){
        char buffer[128];
        if((fgets(buffer,128, f))==NULL) return false; // we need to use fgets in order to avoid spurious effects with following matrix (binary)
        if(sscanf(buffer,"%i %lf %s", &size, &factor_bias, buffer) != 3) return false;
        if(strcmp(buffer, "linear") == 0) {
          actfun    = FeedForwardNN::linear;
        } else if (strcmp(buffer, "sigmoid") == 0) {
          actfun  = FeedForwardNN::sigmoid;
        }else if (strcmp(buffer, "tanh") == 0) {
          actfun  = FeedForwardNN::tanh;
        }else if (strcmp(buffer, "tanhc") == 0) {
          actfun  = FeedForwardNN::tanhc;
        }else if (strcmp(buffer, "tanhr") == 0) {
          actfun  = FeedForwardNN::tanhr;
        }else {
          fprintf(stderr, "unknown activation function \"%s\"!", buffer);
          return false;
        }
        setActFun(actfun);
        return true;
}


ostream& operator<<(std::ostream& str, const Layer& l){
  return str << "Layer(" << l.size  << "," << l.factor_bias << "," << actFun2String(l.actfun) << ")";
}


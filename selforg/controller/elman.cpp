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

#include "elman.h"
#include "controller_misc.h"

using namespace matrix;
using namespace std;



/// initialisation of the network with the given number of input and output units
void Elman::init(unsigned int inputDim, unsigned  int outputDim,
                 double unit_map, RandGen* randGen) {
  MultiLayerFFNN::init(inputDim, outputDim, unit_map, randGen);
  // create context neurons and weights
  if(useElman){
    elmanWeights.set(layers[0].size, layers[0].size);
    // elmanWeights=elmanWeights.mapP(randGen, random_minusone_to_one) * 0.05;
    elmanContext.set(layers[0].size,1);
  }
  if(useJordan){
    unsigned int lastlayer = layers.size()-1;
    jordanWeights.set(layers[0].size, layers[lastlayer].size);
    // jordanWeights=jordanWeights.mapP(randGen, random_minusone_to_one) * 0.05;
    jordanContext.set(layers[lastlayer].size,1);
  }
}

const Matrix Elman::process (const Matrix& input) {
  assert(initialised);
  unsigned int layernum  = layers.size();
  assert(weights.size() == layernum);
  assert(bias.size() == layernum);
  this->input = input;

  // update context units from previous time step
  if(useElman){
    elmanContext =  ys[0];
  }
  if(useJordan){
    jordanContext =  ys[layernum-1];
  }

  zs[0]     = weights[0] * input + bias[0];
  if(useElman)
    zs[0] += elmanWeights * elmanContext;
  if(useJordan)
    zs[0] += jordanWeights * jordanContext;
  ys[0]     = zs[0].map(layers[0].actfun);
  for(unsigned int i = 1; i < layernum; i++) {
    zs[i] = weights[i] * ys[i-1] + bias[i];
    if(i==layernum-1 && useBypass)
      zs[i] += bypassWeights * input;
    ys[i] = zs[i].map(layers[i].actfun);
  }
  return ys[layernum-1];
}

/// performs learning and returns the network output before learning
const Matrix Elman::learn (const Matrix& input,
                           const Matrix& nom_output,
                           double learnRateFactor) {
  assert(initialised);
  int layernum  = layers.size();
  double epsilon         = eps*learnRateFactor;

  assert(weights.size() == (unsigned)layernum);
  assert(bias.size() == (unsigned)layernum);

  // process inputs to calculate activations if necessary
  if(!input.equals(this->input)){
    this->input=input;
    process(this->input);
  }

  // calculate weight updates
  Matrix delta;
  for( int i = layernum-1; i >= 0; i--) {
    const Matrix& xsi = ( i == layernum-1 )
      ? nom_output - ys[layernum-1]
      : (weights[i+1]^T) * delta;

    const Matrix& g_prime = zs[i].map(layers[i].dactfun);
    delta           = xsi.multrowwise(g_prime);
    if(i==layernum-1 && useBypass){ // last layers xsi also has to train bypass
      bypassWeights   += delta * (input^T) * epsilon;
    }
    if(i!=0){ // all but first layer
      weights[i]     += delta * (ys[i-1]^T) * epsilon;
      bias[i]        += delta * epsilon * layers[i].factor_bias;
    }else{ // first layer sees real input
      weights[0]     += delta * (input^T) * epsilon;
      bias[0]        += delta * epsilon * layers[0].factor_bias;
      if(useElman)
        elmanWeights   += delta * (elmanContext^T) * epsilon - elmanWeights*0.001;
      if(useJordan)
        jordanWeights  += delta * (jordanContext^T) * epsilon - jordanWeights*0.001;
    }
  }
  return ys[layernum-1];
}

NetUpdate Elman::weightIncrement(const matrix::Matrix& xsi_){
  assert(initialised);
  int layernum  = layers.size();
  assert(weights.size() == (unsigned)layernum);
  assert(bias.size() == (unsigned)layernum);
  NetUpdate update(layernum, layernum, (useBypass!=0) + (useElman!=0) + (useJordan!=0));
  int k = 0;
  // calculate weight updates
  Matrix delta;
  for( int i = layernum-1; i >= 0; i--) {
    const Matrix& xsi = ( i == layernum-1 )
      ? xsi_
      : (weights[i+1]^T) * delta;

    const Matrix& g_prime = zs[i].map(layers[i].dactfun);
    delta           = xsi.multrowwise(g_prime);
    if(i==layernum-1 && useBypass){ // last layers xsi also has to train bypass
      update.other[i] = delta * (input^T);
      k++;
    }
    if(i!=0){ // all but first layer
      update.weights[i] = delta * (ys[i-1]^T);
      update.bias[i]    = delta * layers[i].factor_bias;
    }else{ // first layer sees real input
      update.weights[0] = delta * (input^T);
      update.bias[0]    = delta * layers[0].factor_bias;
      if(useElman){
        update.other[k] = delta * (elmanContext^T) - elmanWeights*0.00001;
        k++;
      }
      if(useJordan)
        update.other[k] = delta * (jordanContext^T) - jordanWeights*0.00001;
    }
  }
  return update;
}


NetUpdate Elman::weightIncrementBlocked(const matrix::Matrix& xsi_,
                                        int blockedlayer, int blockfrom, int blockto){
  assert(initialised);
  int layernum  = layers.size();
  assert(weights.size() == (unsigned)layernum);
  assert(bias.size() == (unsigned)layernum);
  NetUpdate update(layernum, layernum, (useBypass!=0) + (useElman!=0) + (useJordan!=0));
  int k = 0;
  // calculate weight updates
  Matrix delta;
  for( int i = layernum-1; i >= 0; i--) {
    const Matrix& xsi = ( i == layernum-1 )
      ? xsi_
      : (weights[i+1]^T) * delta;
    const Matrix& g_prime = zs[i].map(layers[i].dactfun);
    delta           = xsi.multrowwise(g_prime);
    // block neurons
    if(i == blockedlayer){
      if(blockto==-1) blockto = delta.getM();
      assert(blockfrom <= blockto && blockto <= ((signed)delta.getM()));
      for(int k=blockfrom; k<blockto; k++) delta.val(k,0)=0;
    }

    if(i==layernum-1 && useBypass){ // last layers xsi also has to train bypass
      update.other[k] = delta * (input^T);
      k++;
    }
    if(i!=0){ // all but first layer
      update.weights[i] = delta * (ys[i-1]^T);
      update.bias[i]    = delta * layers[i].factor_bias;
    }else{ // first layer sees real input
      update.weights[0] = delta * (input^T);
      update.bias[0]    = delta * layers[0].factor_bias;
      if(useElman){
        update.other[k] = delta * (elmanContext^T);//  - elmanWeights*0.00001;
        k++;
      }
      if(useJordan)
        update.other[k] = delta * (jordanContext^T) - jordanWeights*0.00001;
    }
  }
  return update;
}


void Elman::updateWeights(const NetUpdate& update){
  int layernum  = layers.size();
  assert(weights.size() == (unsigned)layernum);
  assert(bias.size() == (unsigned)layernum);
  assert((int)update.weights.size() == layernum &&
         (int)update.other.size() == (useBypass!=0) + (useElman!=0) + (useJordan!=0));

  for(int i=0; i<layernum; i++){
    weights[i] += update.weights[i];
    bias[i]    += update.bias[i];
  }
  int k = 0;
  if( useBypass){
    bypassWeights += update.other[k];
    k++;
  }
  if(useElman){
    elmanWeights += update.other[k];
    k++;
  }
  if(useJordan)
    jordanWeights += update.other[k];
}



void Elman::damp(double damping){
  MultiLayerFFNN::damp(damping);

  if(useElman)
    elmanWeights   -= elmanWeights*damping;
  if(useJordan)
    jordanWeights  -= jordanWeights*damping;

}


bool Elman::store(FILE* f) const {
  bool rv = MultiLayerFFNN::store(f);
  fwrite(&useElman, sizeof(bool), 1, f);
  if(useElman){
    elmanWeights.store(f);
    elmanContext.store(f);
  }
  fwrite(&useJordan, sizeof(bool), 1, f);
  if(useJordan){
    jordanWeights.store(f);
    jordanContext.store(f);
  }
  return rv;
}

bool Elman::restore(FILE* f){
  bool rv = MultiLayerFFNN::restore(f);
  if(!rv) return false;
  if(fread(&useElman, sizeof(bool), 1, f)!=1) return false;
  if(useElman){
    elmanWeights.restore(f);
    elmanContext.restore(f);
  }
  if(fread(&useJordan, sizeof(bool), 1, f)!=1) return false;
  if(useJordan){
    jordanWeights.restore(f);
    jordanContext.restore(f);
  }
  initialised = true;
  return true;
}


/************** Inspectable **********************************/
Inspectable::iparamkeylist Elman::getInternalParamNames() const{
  if(useElman || useJordan){
    list<iparamkey> keylist = MultiLayerFFNN::getInternalParamNames();
    if(someInternalParams){
      if(useElman)  keylist += store4x4AndDiagonalFieldNames(elmanWeights,  "El");
      if(useJordan) keylist += store4x4AndDiagonalFieldNames(jordanWeights, "Jo");
    }else{
      if(useElman) keylist += storeMatrixFieldNames(elmanWeights,  "El");
      if(useJordan) keylist += storeMatrixFieldNames(jordanWeights, "Jo");
    }
    return keylist;
  }else return MultiLayerFFNN::getInternalParamNames();
}

Inspectable::iparamvallist Elman::getInternalParams() const{
  if(useElman || useJordan){
    list<iparamval> l = MultiLayerFFNN::getInternalParams();
    if(someInternalParams){
      if(useElman)  l += store4x4AndDiagonal(elmanWeights);
      if(useJordan) l += store4x4AndDiagonal(jordanWeights);
    }else{
      if(useElman)  l += elmanWeights.convertToList();
      if(useJordan) l += elmanWeights.convertToList();
    }
    return l;
  }else return MultiLayerFFNN::getInternalParams();
}


Inspectable::ilayerlist Elman::getStructuralLayers() const{
  list<Inspectable::ILayer> l = MultiLayerFFNN::getStructuralLayers();
  if(useElman)
    l += ILayer("e","", elmanContext.getM(), 0, "Elman");
  if(useElman)
    l += ILayer("j","", jordanContext.getM(), 0, "Jordan");
  return l;
}

Inspectable::iconnectionlist Elman::getStructuralConnections() const{
  list<Inspectable::IConnection> l = MultiLayerFFNN::getStructuralConnections();
  if(useElman)
    l+=IConnection("El", "e", "y0");
  if(useElman)
    l+=IConnection("Jo", "j", "y0");
  return l;
}

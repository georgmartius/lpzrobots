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

#include "multilayerffnn.h"
#include "controller_misc.h"
#include "regularisation.h"

using namespace matrix;
using namespace std;

MultiLayerFFNN::MultiLayerFFNN(double eps, const std::vector<Layer>& layers,
                               bool useBypass, bool someInternalParams)
  : FeedForwardNN("multilayerffnn", "$Id$"),
    eps(eps), layers(layers),
    useBypass(useBypass), someInternalParams(someInternalParams) {

  addParameter("eps",&(this->eps));
  initialised = false;
  addParameterDef("lambda",&lambda, 0.0001);

}

// initialisation of the network with the given number of input and output units
void MultiLayerFFNN::init(unsigned int inputDim, unsigned  int outputDim,
                          double unit_map, RandGen* randGen) {
  if(initialised) return;
  assert(layers.size() > 0);
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
  int last = layers.size() - 1;
  layers[last].size = outputDim; // adjust output dimension
  unsigned int layernum = layers.size();

  ys.resize(layernum);
  zs.resize(layernum);
  weights.resize(layernum);
  bias.resize(layernum);


  // if useBypass is true, than unit_map is implemented via bypass
  Matrix w(layers[0].size, inputDim);
  w=w.mapP(randGen, random_minusone_to_one) * 0.05 + (useBypass ? w : (w^0)*unit_map);
  weights[0] = w;
  bias[0].set(layers[0].size, 1);
  ys[0].set(layers[0].size, 1);
  zs[0].set(layers[0].size, 1);
  for(unsigned int i = 1; i < layers.size(); i++) {
    Matrix w(layers[i].size, layers[i-1].size);
    w=w.mapP(randGen, random_minusone_to_one) * 0.05  + (useBypass ? w : (w^0)*unit_map);
    weights[i] = w;
    bias[i].set(layers[i].size, 1);
    ys[i].set(layers[i].size, 1);
    zs[i].set(layers[i].size, 1);
  }
  if(useBypass){
    bypassWeights.set(layers[layernum-1].size,inputDim);
    bypassWeights=bypassWeights.mapP(randGen, random_minusone_to_one) * 0.05  + (bypassWeights^0)*unit_map;
  }

  initialised = true;
}

// passive processing of the input
const Matrix MultiLayerFFNN::process (const Matrix& input) {
  assert(initialised);
  unsigned int layernum = layers.size();
  assert(weights.size() == layernum);
  assert(bias.size() == layernum);
  this->input = input;

  // calculate outputs (y's) and activations (z's), which are necessary for activation'()
  zs[0]     = weights[0] * input + bias[0];
  ys[0]     = zs[0].map(layers[0].actfun);
  for(unsigned int i = 1; i < layernum; i++) {
    zs[i] = weights[i] * ys[i-1] + bias[i];
    if(i==(layernum-1) && useBypass)
      zs[i] += bypassWeights * input;
    ys[i] = zs[i].map(layers[i].actfun);
  }
  return ys[layernum-1];
}

// performs learning and returns the network output before learning
const Matrix MultiLayerFFNN::learn (const Matrix& input,
                                  const Matrix& nom_output,
                                  double learnRateFactor) {
  assert(initialised);
  int layernum  = layers.size();
  double epsilon         = eps*learnRateFactor;

  assert(weights.size() == (unsigned)layernum);
  assert(bias.size() == (unsigned)layernum);

  // process inputs to calculate activations if necessary
  if(!input.equals(this->input))
    process(input);

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
      weights[i]     += delta * (input^T) * epsilon;
      bias[i]        += delta * epsilon * layers[i].factor_bias;
    }
  }
  return ys[layernum-1];
}

const Matrix MultiLayerFFNN::inversion(const matrix::Matrix& input, const matrix::Matrix& xsi) const {
  assert(initialised);
  int layernum  = (int)layers.size();

  vector<Matrix> xsis(layernum+1);
  vector<Matrix> deltas(layernum);
  if(useBypass){
    xsis[layernum] = xsi*0.5; // use the one half of the error on the network
  }else{
    xsis[layernum] = xsi;
  }

  for(int i=layernum-1; i>=0; i--){

    deltas[i] = Matrix::map2(layers[i].invactfun, zs[i], xsis[i+1]);
    // for pseudo inversion we want to invert the smaller matrix (WW^T or W^TW) (done automatically)
    xsis[i] = weights[i].pseudoInverse()*(weights[i]^T) * deltas[i];
  }
  if(useBypass){    // use the other half of the error on the bypass
    const Matrix& d = Matrix::map2(layers[layernum-1].invactfun, zs[layernum-1], xsi*0.5);
    xsis[0]+= (bypassWeights.pseudoInverse())*(bypassWeights^T) * d;
  }
  return xsis[0];
}



const Matrix MultiLayerFFNN::response(const Matrix& input) const{
  assert(initialised);
  unsigned int layernum  = layers.size();

  assert(weights.size() == layernum);
  assert(bias.size() == layernum);

  // initialisation of jacobian
  const Matrix& g_prime = zs[layernum-1].map(layers[layernum-1].dactfun);
  Matrix jacob = weights[layernum-1].multrowwise(g_prime);
  // loop over layers (backwards)
  for(int i = layernum-2; i >= 0; i--) {
    const Matrix& g_prime = zs[i].map(layers[i].dactfun);
    jacob *= weights[i].multrowwise(g_prime);
  }
  if(useBypass){
    const Matrix& g_prime = zs[layernum-1].map(layers[layernum-1].dactfun);
    jacob+=bypassWeights.multrowwise(g_prime);
  }
  return jacob;
}


void MultiLayerFFNN::damp(double damping){
  unsigned int len = weights.size();
  if(damping==0) return;
  for(unsigned int i = 0; i < len; i++) {
    //    weights[i] *= (1-damping);
    //    bias[i]    *= (1-damping);
    weights[i] -= weights[i]*damping;
    bias[i]    -= bias[i]*damping;
  }
  if(useBypass)
    bypassWeights -= bypassWeights*damping;
}



bool MultiLayerFFNN::store(FILE* f) const {
        fprintf(f,"%g\n", eps);
        int layernum = layers.size();
        fprintf(f,"%i\n", layernum);
        for(int i=0; i<layernum; i++){
                layers[i].store(f);
        }
        int weightsnum = weights.size();
        fprintf(f,"%i\n#", weightsnum);
        for(int i=0; i<weightsnum; i++){
                weights[i].store(f);
                bias[i].store(f);
        }
        fwrite(&useBypass, sizeof(bool), 1, f);
        if(useBypass)
          bypassWeights.store(f);
        return true;
}

bool MultiLayerFFNN::write(FILE* f) const {
        fprintf(f,"%g\n", eps);
        int layernum = layers.size();
        fprintf(f,"%i\n", layernum);
        for(int i=0; i<layernum; i++){
                layers[i].store(f);
        }
        int weightsnum = weights.size();
        fprintf(f,"%i\n#", weightsnum);
        for(int i=0; i<weightsnum; i++){
                weights[i].write(f);
                bias[i].write(f);
        }
        fwrite(&useBypass, sizeof(bool), 1, f);
        if(useBypass)
          bypassWeights.write(f);
        return true;
}


bool MultiLayerFFNN::restore(FILE* f){
        char buffer[128];
        if(fscanf(f,"%s\n", buffer) != 1) return false;
        eps = atof(buffer);
        unsigned int layernum;
        layers.clear();
        if(fscanf(f,"%i\n", &layernum) != 1) return false;
        for(unsigned int i=0; i<layernum; i++){
                Layer l(1);
                l.restore(f);
                layers.push_back(l);
        }
        ys.resize(layernum);
        zs.resize(layernum);
        for(unsigned int i = 0; i < layernum; i++) {
          ys[i].set(layers[i].size, 1);
          zs[i].set(layers[i].size, 1);
        }

        unsigned int weightsnum;
        weights.clear();
        bias.clear();
        if(fscanf(f,"%i\n#", &weightsnum) != 1) return false;
        for(unsigned int i=0; i<weightsnum; i++){
                Matrix m;
                if(!m.restore(f)) return false;
                weights.push_back(m);
                if(!m.restore(f)) return false;
                bias.push_back(m);
        }
        if(fread(&useBypass, sizeof(bool), 1, f)!=1) return false;
        if(useBypass)
          bypassWeights.restore(f);

        initialised = true;
        return true;
}

/************** Inspectable **********************************/
Inspectable::iparamkeylist MultiLayerFFNN::getInternalParamNames() const{
  list<iparamkey> keylist;
  int weightsnum = weights.size();
  for(int i=0; i<weightsnum; i++){
    if(someInternalParams)
      keylist += store4x4AndDiagonalFieldNames(weights[i], "W"+itos(i));
    else{
      keylist += storeMatrixFieldNames(weights[i], "W"+itos(i));
      keylist += storeVectorFieldNames(ys[i], "y"+itos(i));
    }
    keylist += storeVectorFieldNames(bias[i], "B"+itos(i));
  }
  if(useBypass){
    if(someInternalParams)
      keylist += store4x4AndDiagonalFieldNames(bypassWeights, "By");
    else
      keylist += storeMatrixFieldNames(bypassWeights, "By");
  }
  return keylist;
}

Inspectable::iparamvallist MultiLayerFFNN::getInternalParams() const{
  list<iparamval> l;
  int weightsnum = weights.size();
  for(int i=0; i<weightsnum; i++){
    if(someInternalParams)
      l += store4x4AndDiagonal(weights[i]);
    else{
      l += weights[i].convertToList();
      l += ys[i].convertToList();
    }
    l += bias[i].convertToList();
  }
  if(useBypass){
    if(someInternalParams)
      l += store4x4AndDiagonal(bypassWeights);
    else
      l += bypassWeights.convertToList();
  }
  return l;
}

Inspectable::ilayerlist MultiLayerFFNN::getStructuralLayers() const{
  list<Inspectable::ILayer> l;
  int weightsnum = weights.size();
  l+=ILayer("x","", getInputDim(), 0, "Sensors");

  for(int i=0; i<weightsnum; i++){
    l+=ILayer("y" + itos(i),"B" + itos(i), layers[i].size, i+1,
              (i < weightsnum-1) ? "Hidden" + itos(i) : "Output");
  }
  return l;
}

Inspectable::iconnectionlist MultiLayerFFNN::getStructuralConnections() const{
  list<Inspectable::IConnection> l;
  int weightsnum = weights.size();
  for(int i=0; i<weightsnum; i++){
    l+=IConnection("W" + itos(i) , i==0 ? "x" : ("y" + itos(i-1)), "y" + itos(i) );
  }
  return l;
}

std::vector<ActivationFunction> MultiLayerFFNN::setActivationFunction(ActivationFunction actfun) {
  vector<ActivationFunction> actfuns;
  FOREACH (vector<Layer>,layers,l) {
    actfuns.push_back((*l).actfun);
    (*l).setActFun(actfun);
  }
  return actfuns;
}

void MultiLayerFFNN::setActivationFunctions(std::vector<ActivationFunction> actfunList) {
  vector<ActivationFunction>::const_iterator it=actfunList.begin();
  FOREACH (vector<Layer>,layers,l) {
    (*l).setActFun((*it));
    it++;
  }
}

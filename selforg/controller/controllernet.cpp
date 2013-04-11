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

#include "controllernet.h"
#include "controller_misc.h"
#include "regularisation.h"

using namespace matrix;
using namespace std;


ControllerNet::ControllerNet(const std::vector<Layer>& layers, bool useBypass)
  : Configurable("controllernet", "0.7"),
    layers(layers), useBypass(useBypass) {

  initialised = false;
  addParameterDef("lambda",&lambda, 0.001);
}

// initialisation of the network with the given number of input and output units
void ControllerNet::init(unsigned int inputDim, unsigned  int outputDim,
                         double unit_map, double rand, RandGen* randGen) {
  if(initialised) return;
  assert(layers.size() > 0);
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
  int last = layers.size() - 1;
  layers[last].size = outputDim; // adjust output dimension
  unsigned int layernum = layers.size();

  y.resize(layernum);
  z.resize(layernum);
  gp.resize(layernum);
  weights.resize(layernum);
  bias.resize(layernum);

  // if useBypass is true, than unit_map is implemented via bypass
  Matrix w(layers[0].size, inputDim);
  w=w.mapP(randGen, random_minusone_to_one) * rand + (w^0)*unit_map;
  weights[0] = w;
  bias[0].set(layers[0].size, 1);
  y[0].set(layers[0].size, 1);
  z[0].set(layers[0].size, 1);
  gp[0].set(layers[0].size, 1);
  for(unsigned int i = 1; i < layers.size(); i++) {
    Matrix w(layers[i].size, layers[i-1].size);
    w=w.mapP(randGen, random_minusone_to_one) * rand  + (w^0)*unit_map;
    weights[i] = w;
    bias[i].set(layers[i].size, 1);
    y[i].set(layers[i].size, 1);
    z[i].set(layers[i].size, 1);
    gp[i].set(layers[i].size, 1);
  }
  if(useBypass){
    bypassWeights.set(layers[layernum-1].size,inputDim);
    bypassWeights=bypassWeights.mapP(randGen, random_minusone_to_one) * rand;
  }
  L.set(inputDim, outputDim);
  R.set(inputDim, outputDim);

  initialised = true;
}

// passive processing of the input
const Matrix ControllerNet::process (const Matrix& input) {
  assert(initialised);
  unsigned int layernum = layers.size();
  assert(weights.size() == layernum);
  assert(bias.size() == layernum);
  this->input = input;

  for(unsigned int i = 0; i < layernum; i++) {
    if(i==0)
      z[i] = weights[i] * this->input + bias[i];
    else
      z[i] = weights[i] * y[i-1] + bias[i];
    if(i==(layernum-1) && useBypass)
      z[i] += bypassWeights * input;
    y[i] = z[i].map(layers[i].actfun);
    gp[i] = z[i].map(layers[i].dactfun);
  }

  calcResponseIntern();

  return y[layernum-1];
}

const Matrix ControllerNet::processX (const Matrix& input, const Matrix& injection,
                                      unsigned int injectInLayer){
  assert(initialised);
  unsigned int layernum = layers.size();
  assert(injectInLayer >= 0 && injectInLayer < layernum);
  assert((int)injection.getM() == layers[injectInLayer].size && injection.getN() == 1);
  assert(weights.size() == layernum);
  assert(bias.size() == layernum);
  this->input = input;

  for(unsigned int i = 0; i < layernum; i++) {
    if(i==0)
      z[i] = weights[i] * this->input + bias[i];
    else
      z[i] = weights[i] * y[i-1] + bias[i];
    if(i==(layernum-1) && useBypass)
      z[i] += bypassWeights * input;
    gp[i] = z[i].map(layers[i].dactfun);
    if(i == injectInLayer)
      y[i] = injection;
    else
      y[i] = z[i].map(layers[i].actfun);
  }

  calcResponseIntern();

  return y[layernum-1];
}



const Matrix ControllerNet::forwardpropagation(const matrix::Matrix& error,
                                                Matrices* errors, Matrices* zetas) const {
  assert(initialised);
  unsigned int layernum  = layers.size();

  bool errorsgiven = errors != NULL;
  bool zetasgiven  = zetas  != NULL;
  if(!errorsgiven) errors = new Matrices(layernum+1);
  else errors->resize(layernum+1);
  if(!zetasgiven)  zetas  = new Matrices(layernum);
  else zetas->resize(layernum);

  (*errors)[0] = error;

  for(unsigned int i=0; i<=layernum-1; i++){
    (*zetas)[i]    = weights[i] * (*errors)[i];
    if(i==layernum-1 && useBypass){
      (*zetas)[i] += bypassWeights * (*errors)[0];
    }
    // g' o (W * error) (rowwise multiplication)
    (*errors)[i+1] = gp[i] & (*zetas)[i];
  }

  Matrix result = (*errors)[layernum];
  if(!errorsgiven) delete errors;
  if(!zetasgiven) delete zetas;
  return result;
}

// const Matrix ControllerNet::forwardprojection(const matrix::Matrix& error,
//                                                Matrices* errors, Matrices* zetas) const {
//   assert(initialised);
//   if(useBypass) {
//     return forwardprojectionBP(error,errors,zetas);
//   }
//   unsigned int layernum  = layers.size();

//   bool errorsgiven = errors != NULL;
//   bool zetasgiven  = zetas  != NULL;
//   if(!errorsgiven) errors = new Matrices(layernum+1);
//   else errors->resize(layernum+1);
//   if(!zetasgiven)  zetas  = new Matrices(layernum);
//   else zetas->resize(layernum);

//   (*errors)[0] = error;

//   for(unsigned int i=0; i<=layernum-1; i++){
//     (*zetas)[i]    = (weights[i]^T).pseudoInverse(lambda) * (*errors)[i];
//     // 1/g' o (W^T)^-1 * error (rowwise multiplication)
//     (*errors)[i+1] =  gp[i].map(one_over) & (*zetas)[i];
//   }

//   Matrix result = (*errors)[layernum];
//   if(!errorsgiven) delete errors;
//   if(!zetasgiven) delete zetas;
//   return result;
// }

const Matrix ControllerNet::forwardprojection(const matrix::Matrix& error,
                                                Matrices* errors, Matrices* zetas) const {
  assert(initialised);
  unsigned int layernum  = layers.size();

  bool errorsgiven = errors != NULL;
  bool zetasgiven  = zetas  != NULL;
  if(!errorsgiven) errors = new Matrices(layernum+1);
  else errors->resize(layernum+1);
  if(!zetasgiven)  zetas  = new Matrices(layernum);
  else zetas->resize(layernum);

  // TODO here we can optimize using the sandwiching
  Matrix Linv;
  if(useBypass){
    Linv = L.pseudoInverse(lambda);
    (*errors)[0] = ((Linv * (L-bypassWeights))^T)*error*.5;
  }else{
    (*errors)[0] = error;
  }

  for(unsigned int i=0; i<=layernum-1; i++){
    (*zetas)[i]    = (weights[i]^T).pseudoInverse(lambda) * (*errors)[i];
    if(i==layernum-1 && useBypass){
      // the bypass branch gets
      const Matrix& errorBP     = ((Linv*bypassWeights)^T)*error*.5;
      (*zetas)[i] += (bypassWeights^T).pseudoInverse(lambda) * errorBP;
    }
    // 1/g' o (W^T)^-1 * error (rowwise multiplication)
    (*errors)[i+1] =  gp[i].map(one_over) & (*zetas)[i];
  }

  Matrix result = (*errors)[layernum];
  if(!errorsgiven) delete errors;
  if(!zetasgiven) delete zetas;
  return result;
}



const Matrix ControllerNet::backpropagation(const Matrix& error,
                                            Matrices* errors, Matrices* zetas) const {
  assert(initialised);
  int layernum  = (int)layers.size();

  bool errorsgiven = errors != NULL;
  bool zetasgiven  = zetas  != NULL;
  if(!errorsgiven) errors = new Matrices(layernum+1);
  else errors->resize(layernum+1);
  if(!zetasgiven)  zetas  = new Matrices(layernum);
  else zetas->resize(layernum);

  (*errors)[layernum] = error;

  for(int i=layernum-1; i>=0; i--){
    // error o g' (rowwise multiplication)
    (*zetas)[i]  = (*errors)[i+1] & gp[i];
    // W^T * (error o g')
    (*errors)[i] = (weights[i]^T) * (*zetas)[i];
  }
  if(useBypass){
    (*errors)[0]+= (bypassWeights^T) * (*zetas)[layernum-1];
  }
  Matrix result = (*errors)[0];
  if(!errorsgiven) delete errors;
  if(!zetasgiven) delete zetas;
  return result;
}

const Matrix ControllerNet::backpropagationX(const Matrix& error,
                                             Matrices* errors, Matrices* zetas,
                                             int startWithLayer
                                             ) const {
  assert(initialised);
  int layernum  = (int)layers.size();
  if(startWithLayer<0) startWithLayer = layers.size()+startWithLayer;
  assert(startWithLayer>=0 && startWithLayer < layernum);

  bool errorsgiven = errors != NULL;
  bool zetasgiven  = zetas  != NULL;
  if(!errorsgiven) errors = new Matrices(layernum+1);
  else errors->resize(layernum+1);
  if(!zetasgiven)  zetas  = new Matrices(layernum);
  else zetas->resize(layernum);

  (*errors)[startWithLayer+1] = error;

  for(int i=startWithLayer; i>=0; i--){
    // error o g' (rowwise multiplication)
    (*zetas)[i]  = (*errors)[i+1] & gp[i];
    // W^T * (error o g')
    (*errors)[i] = (weights[i]^T) * (*zetas)[i];
  }
  if(useBypass && (startWithLayer == layernum-1)){
    // here we take the unmodified error
    const Matrix& zetabypass = error & gp[layernum-1];
    (*errors)[0]+= (bypassWeights^T) * zetabypass;
  }
  Matrix result = (*errors)[0];
  if(!errorsgiven) delete errors;
  if(!zetasgiven) delete zetas;
  return result;
}

// const Matrix ControllerNet::backprojection(const Matrix& error,
//                                               Matrices* errors, Matrices* zetas) const {
//   assert(initialised);
//   if(useBypass) {
//     return backprojectionBP(error,errors,zetas);
//   }
//   int layernum  = (int)layers.size();
//   bool errorsgiven = errors != NULL;
//   bool zetasgiven  = zetas  != NULL;
//   if(!errorsgiven) errors = new Matrices(layernum+1);
//   else errors->resize(layernum+1);
//   if(!zetasgiven)  zetas  = new Matrices(layernum);
//   else zetas->resize(layernum);

//   (*errors)[layernum] = error;

//   for(int i=layernum-1; i>=0; i--){
//     // 1/g' o error (rowwise multiplication)
//     (*zetas)[i]  = gp[i].map(one_over) & (*errors)[i+1];
//     // W^-1 * (1/g' o error)
//     (*errors)[i] = weights[i].pseudoInverse(lambda) * (*zetas)[i];
//   }

//   Matrix result = (*errors)[0];
//   if(!errorsgiven) delete errors;
//   if(!zetasgiven) delete zetas;
//   return result;
// }

const Matrix ControllerNet::backprojection(const Matrix& error,
                                           Matrices* errors, Matrices* zetas) const {
  assert(initialised);
  int layernum  = (int)layers.size();
  bool errorsgiven = errors != NULL;
  bool zetasgiven  = zetas  != NULL;
  if(!errorsgiven) errors = new Matrices(layernum+1);
  else errors->resize(layernum+1);
  if(!zetasgiven)  zetas  = new Matrices(layernum);
  else zetas->resize(layernum);

  // TODO here we can optimize using the sandwiching
  Matrix Linv;
  if(useBypass)
    Linv = L.pseudoInverse(lambda);

  (*errors)[layernum] = error;

  for(int i=layernum-1; i>=0; i--){
    // 1/g' o error (rowwise multiplication)
    (*zetas)[i]  = gp[i].map(one_over) & (*errors)[i+1];
    if(i==layernum-1 && useBypass){
      const Matrix& zetaNormal = (L-bypassWeights)*Linv*(*zetas)[i]*.5;
      (*errors)[i] = weights[i].pseudoInverse(lambda) * zetaNormal;
    }else{
      // W^-1 * (1/g' o error)
      (*errors)[i] = weights[i].pseudoInverse(lambda) * (*zetas)[i];
    }
  }
  if(useBypass){
    // the bypass branch gets
    const Matrix& zetaBP     = (bypassWeights)*Linv*(*zetas)[layernum-1]*.5;
    (*errors)[0]+= bypassWeights.pseudoInverse(lambda) * zetaBP;
  }

  Matrix result = (*errors)[0];
  if(!errorsgiven) delete errors;
  if(!zetasgiven) delete zetas;
  return result;
}




const Matrix& ControllerNet::response() const{
  assert(initialised);
  return L;
}

const Matrix& ControllerNet::responseLinear() const{
  assert(initialised);
  return R;
}

void ControllerNet::calcResponseIntern() {
  assert(initialised);
  unsigned int layernum  = layers.size();

  assert(weights.size() == layernum);
  assert(bias.size() == layernum);

  // initialisation of jacobian
  L  = weights[0] & gp[0];
  R  = weights[0];
  // loop over layers
  for(unsigned int i = 1; i< layernum; i++) {
    L = (weights[i] & gp[i]) * L;
    R = weights[i] * R;
  }
  if(useBypass){
    L+= bypassWeights & gp[layernum-1];
    R += bypassWeights;
  }
}

Matrix ControllerNet::responsePart(int from, int to) const{
  assert(initialised);
  int layernum  = (int)layers.size();
  Matrix Res;
  assert(from>=-1 && from < layernum-1);
  from++;
  if(to<0) to = layernum+to;
  assert(to>=0 && to < layernum);
  // initialisation of jacobian
  Res  = weights[from] & gp[from];
  // loop over layers
  for(int i = from+1; i <= to; i++) {
    Res = (weights[i] & gp[i]) * Res;
  }
  return Res;
}


void ControllerNet::damp(double damping){
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

bool ControllerNet::store(FILE* f) const {
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

bool ControllerNet::write(FILE* f) const {
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


bool ControllerNet::restore(FILE* f){
        unsigned int layernum;
        layers.clear();
        if(fscanf(f,"%i\n", &layernum) != 1) return false;
        for(unsigned int i=0; i<layernum; i++){
                Layer l(1);
                l.restore(f);
                layers.push_back(l);
        }
        y.resize(layernum);
        z.resize(layernum);
        for(unsigned int i = 0; i < layernum; i++) {
          y[i].set(layers[i].size, 1);
          z[i].set(layers[i].size, 1);
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

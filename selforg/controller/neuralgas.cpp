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

#include "neuralgas.h"
#include <algorithm>

using namespace std;
using namespace matrix;

typedef vector<pair<double,int> > rankingvector;

NeuralGas::NeuralGas(const std::string& name,
    const std::string& revision) : AbstractModel(name, revision) {
  initialised=false;
  eps=0.1;
  lambda=3;
  t=0;
}

NeuralGas::NeuralGas(double lambda, double eps, int maxTime,
      const std::string& name,
      const std::string& revision)
  : AbstractModel(name, revision), eps(eps), lambda(lambda),  maxTime(maxTime) {
  addParameter("eps",&eps);
  t=0;
  initialised=false;
}

void NeuralGas::init(unsigned int inputDim, unsigned int outputDim,
                     double unit_map, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
  weights.resize(outputDim);
  diffvectors.resize(outputDim);
  double factor = (unit_map == 0) ? 1 : unit_map;
  // pure random initialised in the interval (-factor, factor) in all dimensions
  for(unsigned int i=0; i< outputDim; i++){
    weights[i].set(inputDim,1);
    weights[i]=weights[i].mapP(randGen, random_minusone_to_one)*factor;
    diffvectors[i].set(inputDim,1);
  }
  distances.set(outputDim,1);

  cellsizes.set(outputDim,1);
  updateCellSizes();

  initialised=true;
}

double NeuralGas::activationfunction(double rdfsize, double d){
  //  return max(0.0,1-(2*d));
  return exp(- d*d/rdfsize);
}

double ng_print_double(void* f, double d){
  fprintf((FILE*)f,"%g ",d);
  return d;
}

void NeuralGas::printWeights(FILE* f) const {
  int k=0;
  fprintf(f,"# weight elements, cellsize\n");
  FOREACHC(vector<Matrix>, weights, i){
    i->mapP(f,ng_print_double);
    fprintf(f,"\t%f\n", cellsizes.val(k,0));
    k++;
  }
}

void NeuralGas::printCellsizes(FILE* f) const {
  cellsizes.mapP(f,ng_print_double);
}

const Matrix NeuralGas::process (const Matrix& input){
  unsigned int s = weights.size();
  for(unsigned int i=0; i<s; i++){
    diffvectors[i] = (input - weights[i]);
    double d = diffvectors[i].map(sqr).elementSum();
    distances.val(i,0)= d;
  }
  return distances.map2(activationfunction, cellsizes, distances);
}

const Matrix NeuralGas::learn (const Matrix& input,
                         const Matrix& nom_output,
                         double learnRateFactor ){
  // "activation" of network already done in process
  // rank by distance
  rankingvector ranking(distances.getM());
  for(int i=0; i< ((signed)distances.getM()); i++){
    ranking[i].first  = distances.val(i,0);
    ranking[i].second = i;
  }
  std::sort(ranking.begin(), ranking.end());

  int k=0;
  double e = (maxTime==0) ? eps*learnRateFactor : eps*exp(-3.0*t/(double)maxTime)*learnRateFactor;
  double l = (maxTime==0) ? lambda : lambda*exp(-3.0*t/(double)maxTime);
  FOREACHC(rankingvector , ranking, i){
    double e_l = exp(-k/l) * e;
    if(e_l < e-9) break;
    weights[i->second] = weights[i->second] + diffvectors[i->second]*e_l;
    k++;
  }
  if(t%100==0)
    updateCellSizes();
  t++;
  return Matrix();
}


void NeuralGas::updateCellSizes(){
  int k=0;

  unsigned int s = weights.size();

  FOREACHC(vector<Matrix>, weights, w){
    Matrix dists(s,1);
    for(unsigned int i=0; i<s; i++){
      diffvectors[i] = (*w - weights[i]);
      double d = diffvectors[i].map(sqr).elementSum();
      dists.val(i,0)= d;
    }
    double size = getKthSmallestElement(dists,3);
    cellsizes.val(k,0)=size;
    k++;
  }
}

bool NeuralGas::store(FILE* f) const{
  fprintf(f,"%g\n", eps);
  fprintf(f,"%g\n", lambda);
  fprintf(f,"%i\n", maxTime);
  fprintf(f,"%i\n", t);
  fprintf(f,"%i\n", getOutputDim());

  distances.store(f);
  cellsizes.store(f);
  FOREACHC(vector<Matrix>, weights, w){
    w->store(f);
  }
  return true;
}

bool NeuralGas::restore(FILE* f){
  char buffer[128];
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  eps = atof(buffer);
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  lambda = atof(buffer);
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  maxTime = atoi(buffer);
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  t = atoi(buffer);
  if((fgets(buffer,128, f))==NULL) return false; // we need to use fgets in order to avoid spurious effects with following matrix (binary)
  int odim = atoi(buffer);

  distances.restore(f);
  cellsizes.restore(f);
  weights.clear();
  diffvectors.clear();
  for(int i=0; i < odim; i++){
    Matrix w;
    w.restore(f);
    weights.push_back(w);
    diffvectors.push_back(w); // overwritten anyway
  }
  return true;
}



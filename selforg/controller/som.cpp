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

#include "som.h"

using namespace std;
using namespace matrix;


SOM::SOM(const std::string& name, const std::string& revision) : AbstractModel(name, revision) {
  initialised=false;
  dimensions=1;
  eps=0.1;
  sigma=3;
}

SOM::SOM(unsigned int dimensions, double sigma, double eps, double rbfsize,
         const std::string& name, const std::string& revision)
  : AbstractModel(name, revision), eps(eps), dimensions(dimensions),  sigma(sigma), rbfsize(rbfsize){
  addParameter("eps",&eps);
  initialised=false;
}

void SOM::init(unsigned int inputDim, unsigned int outputDim,
               double unit_map, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
  double s = pow(outputDim,1.0/dimensions);
  size = (int)round(s);
  assert(fabs(s - int(size)) < 0.001);
  weights.resize(outputDim);
  diffvectors.resize(outputDim);

  int input_cube_size = (int)round(pow(outputDim,1.0/inputDim));
  Matrix offset(inputDim,1);
  offset.toMapP(unit_map, constant);

  for(unsigned int i=0; i< outputDim; i++){
    weights[i].set(inputDim,1);
    if(unit_map==0){ //random
      weights[i]=weights[i].mapP(randGen, random_minusone_to_one);
    }else{           // uniform
      weights[i]=indexToCoord(i,input_cube_size, inputDim)*(2*unit_map/(input_cube_size-1))-offset;
    }
    diffvectors[i].set(inputDim,1);
  }
  distances.set(outputDim,1);

  /// initialise neighbourhood
  initNeighbourhood(sigma);


  initialised=true;
}

double SOM::activationfunction(void* rdfsize, double d){
  // we could also use RBF here and maybe think about adaptation of cellsize
  //  return max(0.0,1-(2*d));
  return exp(- d*d/ *(double*)rdfsize);
}

int SOM::coordToIndex(const Matrix& m, int size){
  int index=0;
  int d = 1;
  for(int i=0; i<((signed)m.getM()); i++){
    index+=d * (int)m.val(i,0);
    d*=size;
  }
  return index;
}

Matrix SOM::indexToCoord(int index, int size, int dimensions){
  Matrix m(dimensions,1);
  for(int i=0; i<dimensions; i++){
      m.val(i,0) = index%size;
      index/=size;
  }
  return m;
}



bool SOM::validCoord(const Matrix& m, int size){
  for(int i=0; i<((signed)m.getM()); i++){
    const double& v = m.val(i,0);
    if(v<0 || v>=size) return false;
  }
  return true;
}



void SOM::initNeighbourhood(double sigma){
  int r_sigma=(int)round(sigma);
  int i_sigma= max(3,r_sigma%2==1 ? r_sigma : r_sigma+1); // round to next odd number
  int n_size=(int)pow((double)i_sigma,(double)dimensions);
  if(sigma==0){ i_sigma=1; n_size=1; sigma=1;}
  double maxlen = sqrt((double)dimensions)*sigma/2;
  neighbourhood.resize(n_size);
  Matrix middle(dimensions,1);
  double radius = (int)i_sigma/2;
  middle.toMapP(radius, constant);
  for(int i=0; i<n_size; i++){
    Matrix m = indexToCoord(i,i_sigma, dimensions) - middle;
    neighbourhood[i].first = m;
    neighbourhood[i].second = exp(-m.map(sqr).elementSum()/maxlen); /// normalised gaussian
  }
}

SOM::Neighbours SOM::getNeighbours(int winner){
  Neighbours l;
  Matrix c(dimensions,1); // coordinate of winner
  int w=winner;
  for(int i=0;i<dimensions; i++){
    c.val(i,0)=w%size;
    w/=size;
  }
  for(unsigned int i=0; i< neighbourhood.size(); i++){
    Matrix n = c + neighbourhood[i].first;
    if(validCoord(n, size)){
      l.push_back(pair<int,double>(coordToIndex(n,size),neighbourhood[i].second));
    }
  }
  return l;
}

double som_print_double(void* f, double d){
  fprintf((FILE*)f,"%g ",d);
  return d;
}

void SOM::printWeights(FILE* f) const {
  FOREACHC(vector<Matrix>, weights, i){
    i->mapP(f,som_print_double);
    fprintf(f,"\n");
  }
}

const Matrix SOM::process (const Matrix& input){
  unsigned int s = weights.size();
  for(unsigned int i=0; i<s; i++){
    diffvectors[i] = (input - weights[i]);
    double d = diffvectors[i].map(sqr).elementSum();
    distances.val(i,0)= d;
  }

  return distances.mapP(&rbfsize, activationfunction);
}

const Matrix SOM::learn (const Matrix& input,
                         const Matrix& nom_output,
                         double learnRateFactor ){
  //  unsigned int s = weights.size();
  // "activation" of network already done in process
  // select minimum
  int winner = argmin(distances);

  Neighbours neighbs = getNeighbours(winner);
  FOREACH(Neighbours , neighbs, i){
    weights[i->first] = weights[i->first] + diffvectors[i->first]*(eps*learnRateFactor*i->second);
    // printf("learn: %i,%g\n", i->first, i->second);
    //    cout << "unit: " << (weights[i->first]^T) << ;
    //    cout << "DIFF:"<< (diffvectors[i->first]^T) << endl;
  }
  return Matrix();
}

bool SOM::store(FILE* f) const{
  fprintf(f,"%g\n", eps);
  fprintf(f,"%i\n", dimensions);
  fprintf(f,"%g\n", sigma);
  fprintf(f,"%g\n", rbfsize);
  fprintf(f,"%i\n", size);
  fprintf(f,"%i\n", getOutputDim());

  distances.store(f);
  FOREACHC(vector<Matrix>, weights, w){
    w->store(f);
  }
  return true;
}

bool SOM::restore(FILE* f){
  char buffer[128];
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  eps = atof(buffer);
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  dimensions = atoi(buffer);
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  sigma = atof(buffer);
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  rbfsize = atof(buffer);
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  size = atoi(buffer);
  if((fgets(buffer,128, f))==NULL) return false; // we need to use fgets in order to avoid spurious effects with following matrix (binary)
  int odim = atoi(buffer);

  distances.restore(f);
  weights.clear();
  diffvectors.clear();
  for(int i=0; i < odim; i++){
    Matrix w;
    w.restore(f);
    weights.push_back(w);
    diffvectors.push_back(w); // overwritten anyway
  }
  initNeighbourhood(sigma);
  return true;
}



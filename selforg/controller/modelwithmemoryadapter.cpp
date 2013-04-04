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

#include "modelwithmemoryadapter.h"

using namespace matrix;
using namespace std;


ModelWithMemoryAdapter::ModelWithMemoryAdapter(InvertableModel* model,
                                               int memorySize, int numPatternsPerStep)
  : InvertableModel(model->getName(), model->getRevision()), model(model), memorySize(memorySize), numPatternsPerStep(numPatternsPerStep),
    randGen(0){
  assert(model);
  assert(memorySize>0 && memorySize<1000000);
  assert(numPatternsPerStep>=0 && numPatternsPerStep<1000000);

}

void ModelWithMemoryAdapter::init(unsigned int inputDim, unsigned  int outputDim,
                                  double unit_map, RandGen* randGen){
  model->init(inputDim, outputDim, unit_map, randGen);
  // initialise memory
  memory.resize(memorySize);
  if(randGen){
    this->randGen=randGen;
  }else{
    this->randGen=new RandGen(); // this leads to a memory leak
  }
}

const Matrix ModelWithMemoryAdapter::learn (const matrix::Matrix& input,
                                            const matrix::Matrix& nom_output,
                                            double learnRateFactor){
  // learn new mapping
  const Matrix& out =  model->learn(input, nom_output, learnRateFactor);
  // learn some old stuff
  for(int i=0; i<numPatternsPerStep; i++){
    int index = int(randGen->rand()*memorySize);
    const Pat& pat = memory[index];
    if(!pat.inp.isNulltimesNull()){ // if memory place is used
      //      cout << i << " " ;
      model->learn(pat.inp, pat.out, pat.lrFactor);
    }
  }
  //  cout  << endl;
  // store new mapping
  if(numPatternsPerStep>0)
    memory[int(randGen->rand()*memorySize)]=Pat(input, nom_output, learnRateFactor);
  return out;
}


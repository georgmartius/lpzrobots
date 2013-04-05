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
#ifndef __MODELWITHMEMORYADAPTER_H
#define __MODELWITHMEMORYADAPTER_H

#include <vector>

#include "invertablemodel.h"

/// multi layer neural network with configurable activation functions
class ModelWithMemoryAdapter : public InvertableModel {
public:
  /**
     @param model pointer to model to accomplish by memory
     @param memorySize number of pattern that are stored
     @param numPatternsPerStep number of past patterns to learn each step
  */
  ModelWithMemoryAdapter(InvertableModel* model, int memorySize, int numPatternsPerStep);
  virtual ~ModelWithMemoryAdapter(){ }

  virtual void init(unsigned int inputDim, unsigned  int outputDim,
                    double unit_map = 0.0, RandGen* randGen = 0);

  /**
     learn the input output mapping but also learn mappings from the memory.
     \see InvertableModel::learn
   */
  virtual const matrix::Matrix learn (const matrix::Matrix& input,
                                      const matrix::Matrix& nom_output,
                                      double learnRateFactor = 1);

  /* ********* Delegations *****************/

  virtual const matrix::Matrix process (const matrix::Matrix& input){
    return model->process(input);
  }

  virtual const matrix::Matrix response(const matrix::Matrix& input) const{
    return model->response(input);
  }

  virtual const matrix::Matrix inversion(const matrix::Matrix& input,
                                         const matrix::Matrix& xsi) const{
    return model->inversion(input, xsi);
  }
  virtual unsigned int getInputDim() const {
    return model->getInputDim();
  }
  virtual unsigned int getOutputDim() const {
    return model->getOutputDim();
  }
  virtual void damp(double damping) { model->damp(damping);}

  /* ************** Accessors **********************************/

  Inspectable* getModel(){
    return model;
  }
  const Inspectable* getModel() const {
    return model;
  }


  /* *************  STOREABLE **********************************/
  /// stores the layer binary into file stream
  bool store(FILE* f) const { return model->store(f);}
  /// restores the layer binary from file stream
  bool restore(FILE* f){ return model->restore(f);}

  /* ************* Inspectable **********************************/
  virtual iparamkeylist getInternalParamNames() const { return model->getInternalParamNames();}
  virtual iparamvallist getInternalParams() const { return model->getInternalParams();}
  virtual ilayerlist getStructuralLayers() const { return model->getStructuralLayers();}
  virtual iconnectionlist getStructuralConnections() const {
    return model->getStructuralConnections();
  }


protected:
  // Pattern
  struct Pat{
    Pat(){}
    Pat(const matrix::Matrix& inp, const matrix::Matrix& out, const double& lrFactor):
      inp(inp), out(out), lrFactor(lrFactor){}
    matrix::Matrix inp;
    matrix::Matrix out;
    double lrFactor;
  };

  InvertableModel* model;
  int memorySize;
  int numPatternsPerStep;
  /// vector of input output mappings
  std::vector <Pat> memory;
  RandGen* randGen;
};

#endif

/***************************************************************************
 *   Copyright (C) 2005-2011 by                                            *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   ANY COMMERCIAL USE FORBIDDEN!                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 ***************************************************************************/

#include "multiexpertpair.h"
#include <selforg/multilayerffnn.h>
#include <selforg/modelwithmemoryadapter.h>

using namespace matrix;
using namespace std;

Sat::Sat(InvertableModel* _net, double _eps){
  net=_net;
  eps=_eps;
  lifetime=0;
}


MultiExpertPair::MultiExpertPair( const MultiExpertPairConf& _conf)
  : AbstractModel("MultiExpertPair", "$Id: "), conf(_conf)
{
  runcompetefirsttime=true;
  winner=0;
  companion=1;
  t=0;
  initialised = false;
  managementInterval=10;
};


MultiExpertPair::~MultiExpertPair()
{
  FOREACH(vector<Sat>, sats, s){
    if(s->net) delete s->net;
  }
}


void MultiExpertPair::init(unsigned int inputDim, unsigned  int outputDim,
                           double unit_map, RandGen* randGen){
  assert(conf.numSats>1);
  this->inputDim  = inputDim;
  this->outputDim = outputDim;

  for(int i=0; i<conf.numSats; i++){
    vector<Layer> layers;
    if(conf.numHidden!=0)
      layers.push_back(Layer(conf.numHidden, 0.5 , FeedForwardNN::tanh));
    layers.push_back(Layer(1,0.5));
    MultiLayerFFNN* net = new MultiLayerFFNN(1, layers, 1); // learning rate is set to 1 and modulates each step
    ModelWithMemoryAdapter* netwm = new ModelWithMemoryAdapter(net,1000,10);
    netwm->init(inputDim, outputDim);
    Sat sat(netwm, conf.eps0);
    sats.push_back(sat);
  }

  satErrors.set(conf.numSats, 1);
  satModErrors.set(conf.numSats, 1);
  satAvg1Errors.set(conf.numSats, 1);
  satAvg2Errors.set(conf.numSats, 1);
  satEpsMod.set(conf.numSats, 1);
  double d = 1;
  satEpsMod.toMapP(&d,constant); // set all elements to 1;

  errorCov.set(conf.numSats, conf.numSats);

  //  addParameter("lambda_c", &(conf.lambda_comp));
  addParameter("tauE1", &(conf.tauE1));
  addParameter("tauE2", &(conf.tauE2));
  addParameter("tauW", &(conf.tauW));
  addParameter("lw", &(conf.lambda_w));
  //  addParameter("penalty", &(conf.penalty));

  t=0;
  initialised = true;
}


// performs one step (without learning).
const matrix::Matrix MultiExpertPair::process (const matrix::Matrix& input){
  assert((signed)input.getM() == inputDim && input.getN()==1);
  return sats[winner].net->process(input);
}


const matrix::Matrix MultiExpertPair::learn (const matrix::Matrix& input,
                                            const matrix::Matrix& nom_output,
                                            double learnRateFactor){
  const Matrix& errors = compete(input,nom_output);
  int newwinner = argmin(errors);
  bool newsatadded=false;
  if(newwinner != winner){ // select new winner and companion
    // Todo: use cov matrix!
    cout << winner << ": "<< errorCov.rows(winner,winner) << endl;
    if(newwinner==companion){ // the current companion is the best expert
      // select imature expert if exists
      Matrix epsmod = satEpsMod;
      epsmod.val(companion,0)=0; // knock out old companion
      int newcomp = argmax(epsmod);
      cerr << "new companion with epsmod:" << satEpsMod.val(newcomp,0) << "\n";
      if(satEpsMod.val(newcomp,0)< conf.mature){
        if(conf.maxSats > (signed)sats.size()){
          newcomp = addSat(newcomp);
          newsatadded = true;
          cerr << "create new agent #" << newcomp << "\n";
        }else{
          newcomp = companion;           // use simple competition : winner=companion
          cerr << "all agents are used!\n";
        }
      }
      winner = companion;
      companion = newcomp;
      // just to make the companion selection better at the start,
      //  when all experts have the same immaturity
      satEpsMod.val(companion,0)-=0.0001;
    }else{ // another agent wins.
      if(winner==companion){ // we have already no companion anymore
        // then if there is no imature one then we will have non.
        companion = argmax(satEpsMod);
        if(satEpsMod.val(companion,0)<conf.mature)
          companion = newwinner;
      }
      winner=newwinner;
    }
  }
  if(!newsatadded){ // cov matrix stuff
    // update cov matrix only with respect to the winner
    // we assume the mean normalised error is 1 (also because we are more interested
    // in the correlation of low errors than high ones)
    double mu=1;
    // covE_winner = (E/|E| - mu)*(winner_error - mu)
    const Matrix& covE_winner = (errors*(1/matrixNorm1(errors)) -
                                 errors.mapP(&mu, constant)) *
                                (errors.val(winner,0) - mu);
    // we consider rowwise the correlation of the winner with the others
    for(unsigned int i=0; i<errorCov.getN(); i++){
      errorCov.val(winner,i) = (1-1/conf.tauW)*errorCov.val(winner,i)
        + (1/conf.tauW)*covE_winner.val(i,0);
    }
  }

  // let winner learn
  Matrix out= sats[winner].net->learn(input, nom_output,
                                      sats[winner].eps*satEpsMod.val(winner,0));
  // let companion learn
  if(winner != companion){
    sats[companion].net->learn(input, nom_output,
                               sats[winner].eps*satEpsMod.val(companion,0));
  }

  // the winner only matures if he is successful (1 if error=0; 0 if error>0.3
  double maturation = max(0.0,(0.3-satAvg1Errors.val(winner,0))*(1/0.3));

  satEpsMod.val(winner,0) *= (1-maturation/conf.tauW);

  if(t%managementInterval==0){
    management();
  }
  t++;
  return out;
};


Matrix MultiExpertPair::compete(const matrix::Matrix& input,
                                const matrix::Matrix& nom_output) {
  assert(satErrors.getM()>=sats.size());

  // ask all networks to make their predictions on last timestep,
  //  compare with real world
  unsigned int i=0;
  FOREACH(vector<Sat>, sats, s){
    const Matrix& out = s->net->process(input);
    satErrors.val(i,0) =  (nom_output-out).multTM().val(0,0);
    i++;
  }
  if(runcompetefirsttime){
    satAvg1Errors=satErrors;
    satAvg2Errors=satErrors;
    runcompetefirsttime=false;
  }
  satAvg1Errors = satAvg1Errors * (1.0-1.0/conf.tauE1) + satErrors * (1.0/conf.tauE1);
  satAvg2Errors = satAvg2Errors * (1.0-1.0/conf.tauE2) + satErrors * (1.0/conf.tauE2);

//   // modulate predicted error to prefere mature experts
//   //  does not really work that way
//   Matrix lambdaW = satEpsMod.mapP(&conf.lambda_w, constant);
//   satModErrors = satAvg1Errors.multrowwise(satEpsMod + lambdaW);
  satModErrors = satAvg1Errors;

  // modulate predicted error of winner and companion to introduce hysteresis
  //  satModErrors.val(companion,0)+=0.03;
  satModErrors.val(companion,0)*=1+conf.lambda_w; // small disadvantage for comp
  //  satModErrors.val(winner,0)-=conf.lambda_w/5;
  satModErrors.val(winner,0)*=1-conf.lambda_w; // small advantage for winner
  return satModErrors;
}

int MultiExpertPair::addSat(int copySat){
  int numsats = sats.size();
  assert(numsats < conf.maxSats && copySat >=0 && copySat < numsats);

  ModelWithMemoryAdapter* cnet = dynamic_cast<ModelWithMemoryAdapter*>(sats[copySat].net);
  assert(cnet);
  MultiLayerFFNN* cnn = dynamic_cast<MultiLayerFFNN*>(cnet->getModel());
  MultiLayerFFNN* net = new MultiLayerFFNN(*cnn);
  ModelWithMemoryAdapter* netwm = new ModelWithMemoryAdapter(net,1000,10);
  netwm->init(inputDim, outputDim);
  Sat sat(netwm, conf.eps0);
  sats.push_back(sat);

  // resize vectors
  Matrix additional(1,1, 1.0);
  satErrors.toAbove(additional);
  satModErrors.toAbove(additional);
  satAvg1Errors.toAbove(additional);
  satAvg2Errors.toAbove(additional);
  satEpsMod.toAbove(additional);
  Matrix additionalrow(1,numsats);
  Matrix additionalcol(numsats+1,1);
  errorCov.toAbove(additionalrow);
  errorCov.toBeside(additionalcol);
  numsats++;
  return sats.size()-1;
}

double MultiExpertPair::min(void* m, double d){
  return std::min(*(double*)m,d);
}

void MultiExpertPair::management(){
//   // annealing of neighbourhood learning
//   FOREACH(vector<Sat> , sats, s){
//     s->lifetime+=managementInterval;
//   }
  // imature experts
  if(conf.tauI!=0.0){
    Matrix imaturation(satEpsMod.getM(),1);
    double imaturate= ((double)managementInterval/conf.tauI);
    imaturation.toMapP(&imaturate, constant); // fill matrix with "imaturate"
    satEpsMod += imaturation;
    //also limit it to 1
    double m=1.0;
    satEpsMod.toMapP(&m, MultiExpertPair::min );
  }
}


Configurable::paramval MultiExpertPair::getParam(const paramkey& key, bool traverseChildren) const{
  if (key=="epsSat") return sats[0].eps;
  else return AbstractModel::getParam(key);
}

bool MultiExpertPair::setParam(const paramkey& key, paramval val, bool traverseChildren){
  if(key=="epsSat") {
    FOREACH(vector<Sat>, sats, s){
      s->eps=val;
    }
    return true;
  }else return AbstractModel::setParam(key, val);
}

Configurable::paramlist MultiExpertPair::getParamList() const{
  paramlist keylist = AbstractModel::getParamList();
  keylist += pair<paramkey, paramval>("epsSat",sats[0].eps);
  return keylist;
}


bool MultiExpertPair::store(FILE* f) const {
  fprintf(f,"%i\n", sats.size());
  fprintf(f,"%i\n", conf.numHidden);
  fprintf(f,"%i\n", runcompetefirsttime);

  // save matrix values
  satErrors.store(f);
  satAvg1Errors.store(f);
  satAvg2Errors.store(f);
  satModErrors.store(f);
  satEpsMod.store(f);

  // store sats
  FOREACHC(vector<Sat>, sats, s){
    s->net->store(f);
  }

  // save config and controller
  Configurable::print(f,0);
  return true;
}

bool MultiExpertPair::restore(FILE* f){
  if(!initialised)
    init(2,2);

  char buffer[128];
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  conf.numSats = atoi(buffer);
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  conf.numHidden = atoi(buffer);

 // we need to use fgets in order to avoid spurious effects with following matrix (binary)
  if((fgets(buffer,128, f))==NULL) return false;
  runcompetefirsttime = atoi(buffer);

  // restore matrix values
  satErrors.restore(f);
  satAvg1Errors.restore(f);
  satAvg2Errors.restore(f);
  satModErrors.restore(f);
  satEpsMod.restore(f);

  // clean sats array
  sats.clear();
  // restore sats
  for(int i=0; i < conf.numSats; i++){
    MultiLayerFFNN* n = new MultiLayerFFNN(0,vector<Layer>());
    n->restore(f);
    sats.push_back(Sat(n,n->eps));
  }

  // save config and controller
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

void MultiExpertPair::storeSats(const char* filestem){
  int i=0;
  FOREACH(vector<Sat>, sats, s){
    char fname[256];
    sprintf(fname,"%s_%02i.net", filestem, i);
    FILE* f=fopen(fname,"wb");
    if(!f){ cerr << "MultiExpertPair::storeSats() error while writing file " << fname << endl;   return;  }
    s->net->store(f);
    fclose(f);
    i++;
  }
}

/// restore the sat networks from seperate files
void MultiExpertPair::restoreSats(const std::list<std::string>& filenames){
  unsigned int i=0;
  FOREACHC(list<std::string>, filenames, file){
    if(i>= sats.size()) return;
    FILE* f=fopen(file->c_str(),"rb");
    if(!f)
      cerr << "cannot open file " << *file << endl;
    else{
      sats[i].net->restore(f);
      cout << "restore sat " << i << " with " << *file << endl;
      fclose(f);
    }
    i++;
  }
}


list<Inspectable::iparamkey> MultiExpertPair::getInternalParamNames() const {
  list<iparamkey> keylist;
  keylist += storeVectorFieldNames(satErrors, "errs");
  keylist += storeVectorFieldNames(satAvg1Errors, "avg1errs");
  keylist += storeVectorFieldNames(satAvg2Errors, "avg2errs");
  keylist += storeVectorFieldNames(satModErrors, "merrs");
  keylist += storeVectorFieldNames(satEpsMod, "epsmod");
  keylist += string("winner");
  keylist += string("winner_error");
  keylist += string("companion");
  keylist += string("companion_error");
  return keylist;
}

list<Inspectable::iparamval> MultiExpertPair::getInternalParams() const {
  list<iparamval> l;
  // we have to limit the size of vector to the size at initialisation time.
  l += satErrors.rows(0,conf.numSats-1).convertToList();
  l += satAvg1Errors.rows(0,conf.numSats-1).convertToList();
  l += satAvg2Errors.rows(0,conf.numSats-1).convertToList();
  l += satModErrors.rows(0,conf.numSats-1).convertToList();
  l += satEpsMod.rows(0,conf.numSats-1).convertToList();
  l += (double)winner;
  l += (double)satAvg1Errors.val(winner,0);
  l += (double)companion;
  l += (double)satAvg1Errors.val(companion,0);
  return l;
}

list<Inspectable::ILayer> MultiExpertPair::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
  return l;
}

list<Inspectable::IConnection> MultiExpertPair::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
  return l;
}

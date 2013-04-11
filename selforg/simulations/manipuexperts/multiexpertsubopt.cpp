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

#include "multiexpertsubopt.h"
#include <algorithm>
#include <selforg/multilayerffnn.h>
#include <selforg/controller_misc.h>
#include <selforg/modelwithmemoryadapter.h>

using namespace matrix;
using namespace std;

MultiExpertSubopt::Sat::Sat(InvertableModel* _net, double _eps){
  net=_net;
  eps=_eps;
  lifetime=0;
}


MultiExpertSubopt::MultiExpertSubopt( const MultiExpertSuboptConf& _conf)
  : AbstractModel("MultiExpertSubopt", "$Id: "), conf(_conf)
{
  runcompetefirsttime=true;
  managementInterval=10;
  winner=0;
  initialised = false;
};


MultiExpertSubopt::~MultiExpertSubopt()
{
  FOREACH(vector<Sat>, sats, s){
    if(s->net) delete s->net;
  }
}


void MultiExpertSubopt::init(unsigned int inputDim, unsigned  int outputDim,
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
    ModelWithMemoryAdapter* netwm = new ModelWithMemoryAdapter(net,conf.satMemory, conf.satTrainPast);
    netwm->init(inputDim, outputDim, 1, randGen);
    Sat sat(netwm, conf.eps0);
    sats.push_back(sat);
  }

  satErrors.set(conf.numSats, 1);
  satSubOpt.set(conf.numSats, 1);
  satAvg1Errors.set(conf.numSats, 1);
  satAvg2Errors.set(conf.numSats, 1);
  satMinErrors.set(conf.numSats, 1);
  satEpsMod.set(conf.numSats, 1);
  double d = 1;
  satEpsMod.toMapP(&d,constant); // set all elements to 1;
  d = 1;
  satMinErrors.toMapP(&d,constant); // set all elements to 5;
  satAvg1Errors.toMapP(&d,constant); // set all elements to 5;
  satAvg2Errors.toMapP(&d,constant); // set all elements to 5;

  addParameter("tauF", &(conf.tauF));
  addParameter("tauE1", &(conf.tauE1));
  addParameter("tauE2", &(conf.tauE2));
  addParameter("lambda", &(conf.lambda_comp));

  t=0;
  initialised = true;
}


// performs one step (without learning).
const matrix::Matrix MultiExpertSubopt::process (const matrix::Matrix& input){
  assert((signed)input.getM() == inputDim && input.getN()==1);
  return sats[winner].net->process(input);
}


const matrix::Matrix MultiExpertSubopt::learn (const matrix::Matrix& input,
                                            const matrix::Matrix& nom_output,
                                            double learnRateFactor){

  const Matrix& errors = compete(input,nom_output);

  satSubOpt   = errors - satMinErrors;
  satSubOpt   += errors*0.2;
  Matrix out;

  if(conf.version==A){
    // Version a
    winner = argmin(satSubOpt);
    // update min for winner
    satMinErrors.val(winner,0) = std::min(satMinErrors.val(winner,0), satAvg2Errors.val(winner,0));
  // let winner learn
    out = sats[winner].net->learn(input, nom_output,
                                  sats[winner].eps*satEpsMod.val(winner,0));
  }else if(conf.version==B){
    // Version b
    // winner is only for output and prediction and we take the best one
    winner = argmin(errors);
    out= sats[winner].net->process(input);
    // However learning is based on ranked suboptimality
    // rank
    vector<pair<double,int> > ranking(errors.getM());
    for(unsigned int i=0; i< satSubOpt.getM(); i++){
      ranking[i].first  = satSubOpt.val(i,0);
      ranking[i].second = i;
    }
    std::sort(ranking.begin(), ranking.end());
    // winner has lowest ranking
    //winner = ranking[0].second;
    //    out= sats[winner].net->process(input);
    for(unsigned int i=0; i< satSubOpt.getM(); i++){
      if(conf.lambda_comp*i >= 6) continue; // no need for learning (eps factor < 1e-3 )
      //    cout << ranking[i].first << " " << ranking[i].second
      //         << " " << exp(-conf.lambda_comp*i) << "\n";
      sats[ranking[i].second].net->
        learn(input, nom_output, sats[ranking[i].second].eps * exp(-conf.lambda_comp*i));
    }

    // update min for all
    satMinErrors = Matrix::map2(min, satMinErrors, satAvg2Errors);
    //satMinErrors = Matrix::map2P(&conf, mindynamics, satMinErrors, errors);
    //satMinErrors.val(winner,0) = mindynamics(&conf, satMinErrors.val(winner,0), errors.val(winner,0));


  }else{
    assert("" == "Not implemented version");
  }


  if(t%managementInterval==0){
    management();
  }
  t++;
  return out;
};


// minimum dynamics
double MultiExpertSubopt::mindynamics(void *conf, double m, double e){
  MultiExpertSuboptConf* c = (MultiExpertSuboptConf*)conf;
  //  if(e<m){
    return m - (1/c->tauE2)*(m-e);
    //  }else return m;
}


Matrix MultiExpertSubopt::compete(const matrix::Matrix& input,
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
  satAvg1Errors = satAvg1Errors * (1.0-1.0/conf.tauE1) + satErrors * (1.0/conf.tauE1);
  satAvg2Errors = satAvg2Errors * (1.0-1.0/conf.tauE2) + satErrors * (1.0/conf.tauE2);

  return satAvg1Errors;
}


void MultiExpertSubopt::management(){
  // decay minima and learning rate modulations
  double d = 10e-7;
  satMinErrors.toMap2( max, satMinErrors.mapP(&d, constant));
  satMinErrors *= 1.0+(1.0/conf.tauF);
}


Configurable::paramval MultiExpertSubopt::getParam(const paramkey& key, bool traverseChildren) const{
  if (key=="epsSat") return sats[0].eps;
  else return AbstractModel::getParam(key);
}

bool MultiExpertSubopt::setParam(const paramkey& key, paramval val, bool traverseChildren){
  if(key=="epsSat") {
    FOREACH(vector<Sat>, sats, s){
      s->eps=val;
    }
    return true;
  }else return AbstractModel::setParam(key, val);
}

Configurable::paramlist MultiExpertSubopt::getParamList() const{
  paramlist keylist = AbstractModel::getParamList();
  keylist += pair<paramkey, paramval>("epsSat",sats[0].eps);
  return keylist;
}


bool MultiExpertSubopt::store(FILE* f) const {
  fprintf(f,"%i\n", conf.numSats);
  fprintf(f,"%i\n", conf.numHidden);
  fprintf(f,"%i\n", runcompetefirsttime);

  // save matrix values
  satErrors.store(f);
  satAvg1Errors.store(f);
  satAvg2Errors.store(f);
  satSubOpt.store(f);
  satMinErrors.store(f);
  satEpsMod.store(f);

  // store sats
  FOREACHC(vector<Sat>, sats, s){
    s->net->store(f);
  }

  // save config and controller
  Configurable::print(f,0);
  return true;
}

bool MultiExpertSubopt::restore(FILE* f){
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
  satSubOpt.restore(f);
  satMinErrors.restore(f);
  satEpsMod.restore(f);

  // clean sats array
  sats.clear();
  // restore sats
  for(int i=0; i < conf.numSats; i++){
    // FIXME: load and restore ModelWithMemoryAdapter!
    MultiLayerFFNN* n = new MultiLayerFFNN(0,vector<Layer>());
    n->restore(f);
    sats.push_back(Sat(n,n->eps));
  }

  // save config and controller
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

void MultiExpertSubopt::storeSats(const char* filestem){
  int i=0;
  FOREACH(vector<Sat>, sats, s){
    char fname[256];
    sprintf(fname,"%s_%02i.net", filestem, i);
    FILE* f=fopen(fname,"wb");
    if(!f){ cerr << "MultiExpertSubopt::storeSats() error while writing file " << fname << endl;   return;  }
    s->net->store(f);
    fclose(f);
    i++;
  }
}

/// restore the sat networks from seperate files
void MultiExpertSubopt::restoreSats(const std::list<std::string>& filenames){
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


list<Inspectable::iparamkey> MultiExpertSubopt::getInternalParamNames() const {
  list<iparamkey> keylist;
  keylist += storeVectorFieldNames(satErrors, "errs");
  keylist += storeVectorFieldNames(satAvg1Errors, "avg1errs");
  keylist += storeVectorFieldNames(satAvg2Errors, "avg2errs");
  keylist += storeVectorFieldNames(satSubOpt, "subopt");
  keylist += storeVectorFieldNames(satMinErrors, "minerrs");
  keylist += storeVectorFieldNames(satEpsMod, "epsmod");
  keylist += string("winner");
  keylist += string("winner_error");
  return keylist;
}

list<Inspectable::iparamval> MultiExpertSubopt::getInternalParams() const {
  list<iparamval> l;
  l += satErrors.convertToList();
  l += satAvg1Errors.convertToList();
  l += satAvg2Errors.convertToList();
  l += satSubOpt.convertToList();
  l += satMinErrors.convertToList();
  l += satEpsMod.convertToList();
  l += (double)winner;
  l += (double)satAvg1Errors.val(winner,0);
  return l;
}

list<Inspectable::ILayer> MultiExpertSubopt::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
  return l;
}

list<Inspectable::IConnection> MultiExpertSubopt::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
  return l;
}

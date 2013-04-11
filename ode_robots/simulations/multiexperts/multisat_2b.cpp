/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 *   $Log$
 *   Revision 1.3  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.2  2007/07/19 15:44:32  martius
 *   new multisat version without gating
 *
 *   Revision 1.1  2007/06/21 16:31:54  martius
 *   *** empty log message ***
 *
 *   Revision 1.4  2007/06/18 08:11:22  martius
 *   nice version with many agents
 *
 *   Revision 1.3  2007/06/14 08:01:45  martius
 *   Pred error modulation by distance to minimum works
 *
 *   Revision 1.2  2007/06/08 15:37:22  martius
 *   random seed into OdeConfig -> logfiles
 *
 *   Revision 1.1  2007/04/20 12:30:42  martius
 *   multiple sat networks test
 *
 *
 ***************************************************************************/

#include "multisat.h"

using namespace matrix;
using namespace std;


Sat::Sat(MultiLayerFFNN* _net, double _eps){
  net=_net;
  eps=_eps;
  lifetime=0;
}


MultiSat::MultiSat( const MultiSatConf& _conf)
  : AbstractController("MultiSat", "$Id: "), buffersize(_conf.buffersize), conf(_conf)
{
  gatingSom=0;
  gatingNet=0;
  if(conf.numContext==0) {
    cerr << "Please give a nonzero number of context neurons\n";
    exit(1);
  }
  runcompetefirsttime=true;
  managementInterval=100;
  winner=0;
  initialised = false;
};


MultiSat::~MultiSat()
{
  if(x_buffer && y_buffer && xp_buffer){
    delete[] x_buffer;
    delete[] y_buffer;
    delete[] xp_buffer;
  }
  FOREACH(vector<Sat>, sats, s){
    if(s->net) delete s->net;
  }
  if(gatingSom) delete gatingSom;
  if(gatingNet) delete gatingNet;
}


void MultiSat::init(int sensornumber, int motornumber){

  number_motors  = motornumber;
  number_sensors = sensornumber;
  int number_real_sensors = number_sensors - conf.numContext;

  if(!conf.controller){
    cerr << "multisat::init() no main controller given in config!" << endl;
    exit(1);
  }
  conf.controller->init(number_real_sensors, motornumber);

  x_buffer = new Matrix[buffersize];
  xp_buffer = new Matrix[buffersize];
  y_buffer = new Matrix[buffersize];
  x_context_buffer = new Matrix[buffersize];
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_real_sensors,1);
    xp_buffer[k].set(2*number_real_sensors,1);
    y_buffer[k].set(number_motors,1);
    x_context_buffer[k].set(conf.numContext,1);
  }


  for(int i=0; i<conf.numSats; i++){
    vector<Layer> layers;
    layers.push_back(Layer(conf.numHidden, 0.5 , FeedForwardNN::tanh));
    layers.push_back(Layer(1,1));
    MultiLayerFFNN* net = new MultiLayerFFNN(1, layers); // learning rate is set to 1 and modulates each step
    if(conf.useDerive)
      net->init(3*number_real_sensors+number_motors, number_real_sensors+number_motors);
    else
      net->init(2*number_real_sensors+number_motors, number_real_sensors+number_motors);
    Sat sat(net, conf.eps0);
    sats.push_back(sat);
  }

  satErrors.set(conf.numSats, 1);
  satPredErrors.set(conf.numSats, 1);
  satModPredErrors.set(conf.numSats, 1);
  satAvgErrors.set(conf.numSats, 1);
  satMinErrors.set(conf.numSats, 1);

  // initialise gating network
  int numsomneurons = conf.numSomPerDim;
  numsomneurons = (int)pow(numsomneurons,conf.numContext);
  cout << "Init SOM with " << numsomneurons << " units \n";
  gatingSom = new SOM(1,1.0,0.001, 1); // 1D lattice, neighbourhood 1 (little neighbourhood impact)
  gatingSom->init(conf.numContext,numsomneurons,2.0); // uniform distributed in the interval -2..2
  vector<Layer> layers;
  layers.push_back(Layer(1,1)); // one layer linear (output dimension is set on init time)
  gatingNet = new MultiLayerFFNN(0.01, layers);
  gatingNet->init(numsomneurons,conf.numSats);

  addParameter("epsGS", &(gatingSom->eps));
  addParameter("epsGN", &(gatingNet->eps));
  addParameter("lambda_c", &(conf.lambda_comp));
  addParameter("deltaMin", &(conf.deltaMin));
  addParameter("tauC", &(conf.tauC));
  addParameter("tauE", &(conf.tauE));

  t=0;
  initialised = true;
}

// put new value in ring buffer
void MultiSat::putInBuffer(matrix::Matrix* buffer, const matrix::Matrix& vec, int delay){
  buffer[(t-delay)%buffersize] = vec;
}

/// performs one step (includes learning). Calculates motor commands from sensor inputs.
void MultiSat::step(const sensor* x_, int number_sensors, motor* y_, int number_motors)
{

  fillSensorBuffer(x_, number_sensors);
  conf.controller->step(x_, number_sensors-conf.numContext, y_, number_motors);
  fillMotorBuffer(y_, number_motors);
  if(t>buffersize) {

    const Matrix& errors = compete();
    winner = argmin(errors);
    // update min for winner
    satMinErrors.val(winner,0) = min(satMinErrors.val(winner,0), satAvgErrors.val(winner,0));

    //    cout << "Winner: " << winner << endl;
    // rank
    //  the whole ranking thing is actually a residual from former times, keep it for simplicity
    vector<pair<double,int> > ranking(errors.getM());
    for(int i=0; i< errors.getM(); i++){
      ranking[i].first  = errors.val(i,0);
      ranking[i].second = i;
    }
    std::sort(ranking.begin(), ranking.end());

    sats[ranking[0].second].net->learn(satInput, nomSatOutput,
                                       sats[ranking[0].second].eps);
    FOREACH(vector<Sat>, sats, s){
      double e = exp(-(1/conf.tauC)*s->lifetime);
      if(e>10e-12){
        s->net->learn(satInput, nomSatOutput, s->eps*e);
      }
    }
  }
  if(t%managementInterval==0){
    management();
  }
  // update step counter
  t++;
};

/// performs one step without learning. Calulates motor commands from sensor inputs.
void MultiSat::stepNoLearning(const sensor* x, int number_sensors, motor*  y, int number_motors )
{
  fillSensorBuffer(x, number_sensors);
  conf.controller->stepNoLearning(x, number_sensors-conf.numContext, y,number_motors);
  fillMotorBuffer(y, number_motors);
  // update step counter
  t++;
};


void MultiSat::fillSensorBuffer(const sensor* x_, int number_sensors)
{
  assert((unsigned)number_sensors == this->number_sensors);
  Matrix x(number_sensors-conf.numContext, 1, x_);
  Matrix x_c(conf.numContext, 1, x_+number_sensors-conf.numContext);
  // put new input vector in ring buffer x_buffer
  putInBuffer(x_buffer, x);
  if(conf.useDerive){
    const Matrix& xp = calcDerivatives(x_buffer,0);
    putInBuffer(xp_buffer, xp);
  }
  putInBuffer(x_context_buffer, x_c);
}

void MultiSat::fillMotorBuffer(const motor* y_, int number_motors)
{
  assert((unsigned)number_motors == this->number_motors);
  Matrix y(number_motors,1,y_);
  // put new output vector in ring buffer y_buffer
  putInBuffer(y_buffer, y);
}

double multisat_errormodulation(void* fak, double e, double e_min){
  double faktor = *((double*)fak);
  return e*(1 + faktor*sqr(max(0.0,e-e_min)));
}


double multisat_min(double a, double b){
  return min(a,b);
}

Matrix MultiSat::compete()
{
  const Matrix& x_context = x_context_buffer[t%buffersize];
  const Matrix& x = x_buffer[t%buffersize];

  const Matrix& x_tm1 = x_buffer[(t-1)%buffersize];
  const Matrix& x_tm2 = x_buffer[(t-2)%buffersize];
  const Matrix& xp_tm1 = xp_buffer[(t-1)%buffersize];
  const Matrix& y_tm1 = y_buffer[(t-1)%buffersize];
  const Matrix& y_tm2 = y_buffer[(t-2)%buffersize];

  // we have to use F(x_{t-1},x_{t-2} | \dot x_{t-1} ,y_{t-2}) -> (x_t, y_{t-1}) for the sat network

  // let gating network decide about winner:
  const Matrix& somOutput = gatingSom->process(x_context);
  satPredErrors = gatingNet->process(somOutput);

  nomSatOutput = x.above(y_tm1);
  if(conf.useDerive)
    satInput   = x_tm1.above(xp_tm1.above(y_tm2));
  else
    satInput   = x_tm1.above(x_tm2.above(y_tm2));

  // ask all networks to make there predictions on last timestep, compare with real world
  // and train gating network

  assert(satErrors.getM()>=sats.size());
  assert(satPredErrors.getM()>=sats.size());

  unsigned int i=0;
  FOREACH(vector<Sat>, sats, s){
    const Matrix& out = s->net->process(satInput);
    satErrors.val(i,0) =  (nomSatOutput-out).multTM().val(0,0);
    i++;
  }
  if(runcompetefirsttime){
    satAvgErrors=satErrors*2;
    satMinErrors=satAvgErrors;
    runcompetefirsttime=false;
  }
  satAvgErrors = satAvgErrors * (1.0-1.0/conf.tauE) + satErrors * (1.0/conf.tauE);
  // minimum only updated for winner in step()
  //  satMinErrors = Matrix::map2(multisat_min, satMinErrors, satAvgErrors);

  //  cout << "Errors:" << (errors^T) << endl;
  //  cout << "Real winner " << argmin(errorPred) << "winner after training : " << argmin(errors) << endl;

  // train gating network
  gatingSom->learn(x_context,somOutput);
  gatingNet->learn(somOutput,satErrors);

  // modulate predicted error to avoid strong relearning
  satModPredErrors = Matrix::map2P(&conf.penalty, multisat_errormodulation, satPredErrors, satMinErrors);

  return satModPredErrors;

}


Matrix MultiSat::calcDerivatives(const matrix::Matrix* buffer,int delay){
  int t1 = t+buffersize;
  const Matrix& xt    = buffer[(t1-delay)%buffersize];
  const Matrix& xtm1  = buffer[(t1-delay-1)%buffersize];
  const Matrix& xtm2  = buffer[(t1-delay-2)%buffersize];
  return ((xt - xtm1) * 5).above((xt - xtm1*2 + xtm2)*10);
}

void MultiSat::management(){
  // annealing of neighbourhood learning
  FOREACH(vector<Sat> , sats, s){
    s->lifetime+=managementInterval;
  }
  // conf.lambda_comp = t * (1.0/conf.tauC);

  // decay minima
  Matrix deltaM (satMinErrors.getM(),1);
  double delta = (conf.deltaMin*(double)managementInterval/1000.0);
  deltaM.toMapP(&delta, constant); // fill matrix with delta
  satMinErrors += deltaM;
}


Configurable::paramval MultiSat::getParam(const paramkey& key, bool traverseChildren) const{
  if (key=="epsSat") return sats[0].eps;
  else return AbstractController::getParam(key);
}

bool MultiSat::setParam(const paramkey& key, paramval val, bool traverseChildren){
  if(key=="epsSat") {
    FOREACH(vector<Sat>, sats, s){
      s->eps=val;
    }
    return true;
  }else return AbstractController::setParam(key, val);
}

Configurable::paramlist MultiSat::getParamList() const{
  paramlist keylist = AbstractController::getParamList();
  keylist += pair<paramkey, paramval>("eps",sats[0].eps);
  return keylist;
}


bool MultiSat::store(FILE* f) const {
  fprintf(f,"%i\n", conf.numSats);
  fprintf(f,"%i\n", conf.numContext);
  fprintf(f,"%i\n", conf.numSomPerDim);
  fprintf(f,"%i\n", conf.numHidden);

  fprintf(f,"%i\n", runcompetefirsttime);

  // save matrix values
  satErrors.store(f);
  satPredErrors.store(f);
  satModPredErrors.store(f);
  satAvgErrors.store(f);
  satMinErrors.store(f);

  // save gating network
  gatingSom->store(f);
  gatingNet->store(f);

  // store sats
  FOREACHC(vector<Sat>, sats, s){
    s->net->store(f);
  }

  // save config and controller
  Configurable::print(f,0);
  conf.controller->store(f);
  return true;
}

bool MultiSat::restore(FILE* f){
  if(!initialised)
    init(2,2);

  char buffer[128];
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  conf.numSats = atoi(buffer);
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  conf.numContext = atoi(buffer);
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  conf.numSomPerDim = atoi(buffer);
  if(fscanf(f,"%s\n", buffer) != 1) return false;
  conf.numHidden = atoi(buffer);

  if(fscanf(f,"%s\n", buffer) != 1) return false;
  runcompetefirsttime = atoi(buffer);

  // restore matrix values
  satErrors.restore(f);
  satPredErrors.restore(f);
  satModPredErrors.restore(f);
  satAvgErrors.restore(f);
  satMinErrors.restore(f);

  // restore gating network
  if(!gatingSom->restore(f)) return false;
  if(!gatingNet->restore(f)) return false;

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
  conf.controller->restore(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

void MultiSat::storeSats(const char* filestem){
  int i=0;
  FOREACH(vector<Sat>, sats, s){
    char fname[256];
    sprintf(fname,"%s_%02i.net", filestem, i);
    FILE* f=fopen(fname,"wb");
    if(!f){ cerr << "MultiSat::storeSats() error while writing file " << fname << endl;   return;  }
    s->net->store(f);
    fclose(f);
    i++;
  }
}

list<Inspectable::iparamkey> MultiSat::getInternalParamNames() const {
  list<iparamkey> keylist;

//   keylist += storeMatrixFieldNames(y_teaching, "yteach");
//   keylist += storeVectorFieldNames(H, "H");
//   keylist += storeVectorFieldNames(B, "B");
  keylist += storeVectorFieldNames(x_context_buffer[0], "XC");
  keylist += storeVectorFieldNames(satErrors, "errs");
  keylist += storeVectorFieldNames(satPredErrors, "perrs");
  keylist += storeVectorFieldNames(satModPredErrors, "mperrs");
  keylist += storeVectorFieldNames(satAvgErrors, "avgerrs");
  keylist += storeVectorFieldNames(satMinErrors, "minerrs");
  keylist += string("epsSatAn");
  keylist += string("winner");
  return keylist;
}

list<Inspectable::iparamval> MultiSat::getInternalParams() const {
  list<iparamval> l;
  //   l += B.convertToList();
  l += x_context_buffer[t%buffersize].convertToList();
  l += satErrors.convertToList();
  l += satPredErrors.convertToList();
  l += satModPredErrors.convertToList();
  l += satAvgErrors.convertToList();
  l += satMinErrors.convertToList();
  l += (double)exp(-(1/conf.tauC)*sats[0].lifetime);
  l += (double)winner;
  return l;
}

list<Inspectable::ILayer> MultiSat::getStructuralLayers() const {
  list<Inspectable::ILayer> l;
//   l+=ILayer("x","", number_sensors, 0, "Sensors");
//   l+=ILayer("y","H", number_motors, 1, "Motors");
//   l+=ILayer("xP","B", number_sensors, 2, "Prediction");
  return l;
}

list<Inspectable::IConnection> MultiSat::getStructuralConnections() const {
  list<Inspectable::IConnection> l;
//   l+=IConnection("C", "x", "y");
//   l+=IConnection("A", "y", "xP");
//   if(conf.useS) l+=IConnection("S", "x", "xP"); // this is not quite true! it is x' x'' -> xp
  return l;
}

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
 *   Revision 1.2  2011-05-30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.1  2008/11/14 11:23:05  martius
 *   added centered Servos! This is useful for highly nonequal min max values
 *   skeleton has now also a joint in the back
 *
 *   Revision 1.12  2007/12/13 16:57:40  martius
 *   new variation of the multisat controller
 *
 *   Revision 1.11  2007/08/24 11:59:43  martius
 *   *** empty log message ***
 *
 *   Revision 1.10  2007/08/06 14:25:57  martius
 *   new version without gating network
 *
 *   Revision 1.9  2007/07/19 15:44:32  martius
 *   new multisat version without gating
 *
 *   Revision 1.8  2007/07/13 15:34:59  martius
 *   restore bug fixed
 *
 *   Revision 1.7  2007/07/03 13:06:17  martius
 *   *** empty log message ***
 *
 *   Revision 1.6  2007/06/22 14:25:08  martius
 *   *** empty log message ***
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
  runcompetefirsttime=true;
  managementInterval=100;
  winner=0;
  companion=1;
  satControl=false;
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
}


void MultiSat::init(int sensornumber, int motornumber, RandGen* randGen){

  number_motors  = motornumber;
  number_sensors = sensornumber;
  int number_real_sensors = number_sensors - conf.numContext;
  assert(conf.numSats>1);
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak


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

  int satinputdim = (conf.useDerive ? 3 : 2)*number_real_sensors +
    (conf.useY ? number_motors : 0);
  int satoutputdim = number_real_sensors+number_motors;
  for(int i=0; i<conf.numSats; i++){
    vector<Layer> layers;
    layers.push_back(Layer(conf.numHidden, 0.5 , FeedForwardNN::tanh));
    layers.push_back(Layer(1,1));
    MultiLayerFFNN* net = new MultiLayerFFNN(1, layers); // learning rate is set to 1 and modulates each step
    net->init(satinputdim, satoutputdim, 0, randGen);
    Sat sat(net, conf.eps0);
    sats.push_back(sat);
  }


  satErrors.set(conf.numSats, 1);
  satModErrors.set(conf.numSats, 1);
  satAvg1Errors.set(conf.numSats, 1);
  satAvg2Errors.set(conf.numSats, 1);
  satMinErrors.set(conf.numSats, 1);
  satEpsMod.set(conf.numSats, 1);
  double d = 1;
  satEpsMod.toMapP(&d,constant); // set all elements to 1;
  satPredictWeight.set(satoutputdim,1);
  satPredictWeight.toMapP(&d,constant); // set all elements to 1;
  for(int i=0; i < number_real_sensors; i++) satPredictWeight.val(i,0)=0.5;


  //  addParameter("lambda_c", &(conf.lambda_comp));
  addParameter("tauE1", &(conf.tauE1));
  addParameter("tauE2", &(conf.tauE2));
  addParameter("tauW", &(conf.tauW));
  addParameter("lw", &(conf.lambda_w));
  addParameter("satControl", &(conf.satControlFactor));
  //  addParameter("penalty", &(conf.penalty));

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
  assert(number_sensors && number_motors);
  Matrix y_sat;

  fillSensorBuffer(x_, number_sensors);
  if(t>buffersize) {

    const Matrix& errors = compete();
    int newwinner = argmin(errors);
    if(newwinner != winner){ // select new winner and companion
      if(newwinner==companion){
        // select imature expert if exists
        Matrix epsmod = satEpsMod;
        epsmod.val(companion,0)=0; // knock out old companion
        int newcomp = argmax(epsmod);
        cerr << "new companion with epsmod:" << satEpsMod.val(newcomp,0) << "\n";
//         if(satEpsMod.val(newcomp,0)<0.4){
//           // no imature expert go to simple competition without companion
//           winner = companion;
//           cerr << "would need to create new agent\n";
//         }else{
          if(newcomp==companion) {
            cerr << "should never happen!\n";
          }
          winner = companion;
          companion = newcomp;
          // just to make the companion selection better at the start,
          //  when all experts have the same immaturity
          satEpsMod.val(companion,0)-=0.0001;
          //        }
      }else{ // another agent wins.
        if(winner==companion){ // we have already no companion
          // then if there is no imature one then we will have non.
          companion = argmax(satEpsMod);
          if(satEpsMod.val(companion,0)<0.4)
            companion = newwinner;
        }
        winner=newwinner;
      }
    }
    // update min for winner
    satMinErrors.val(winner,0) = min(satMinErrors.val(winner,0), satAvg2Errors.val(winner,0));
    // let winner learn
    sats[winner].net->learn(satInput, nomSatOutput,
                            sats[winner].eps*satEpsMod.val(winner,0));
    // let companion learn
    if(winner != companion){
      sats[companion].net->learn(satInput, nomSatOutput,
                                 sats[winner].eps*satEpsMod.val(companion,0));
    }

    // the winner only matures if he is successful (1 if error=0; 0 if error>0.3
    double maturation = max(0.0,(0.3-satAvg1Errors.val(winner,0))*1/0.3);

    satEpsMod.val(winner,0) *= (1-maturation/conf.tauW);

//     // let all sats learn with their decreasing learning rate
//     FOREACH(vector<Sat>, sats, s){
//       double e = exp(-(1/conf.tauC)*s->lifetime);
//       if(e>10e-12){
//         s->net->learn(satInput, nomSatOutput, s->eps*e);
//       }
//     }

    // check for satelite control
    if(conf.satControlFactor >0)
      y_sat = controlBySat(winner);
  }
  if(t%managementInterval==0){
    management();
  }

  // let main controller give its commands
  conf.controller->step(x_, number_sensors-conf.numContext, y_, number_motors);
  fillMotorBuffer(y_, number_motors); // store the plain c-array "_y" into the y buffer

  // okay, use y from sat to control robot partially (half)
  if(!y_sat.isNulltimesNull()){
    satControl = true;
    const Matrix& y = y_buffer[t % buffersize]; // this is the command to main controller just gave
    Matrix y_res = (y + y_sat) * 0.5;
    y_res.convertToBuffer(y_, number_motors); // store the values into y_ array
    fillMotorBuffer(y_, number_motors); //  overwrite buffer
  }else satControl=false;
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
  //  return e*(1 + faktor*sqr(max(0.0,e-e_min)));
  return e*(1 + sqrt(faktor*max(0.0,e-e_min)));
}

// we need this indirection because of some template error if we use just min
double multisat_min(double a, double b){
  return min(a,b);
}

Matrix MultiSat::controlBySat(int winner){
  /* idea 1:
     sat with below half of the minimal prediction error range is allowed to
     give a control suggestion
   */
  //  double mini = min(satMinErrors);
  //  double maxi = max(satMinErrors);
  //  if( satAvgErrors.val(winner,0) < ( (mini + maxi) / 2 ) ){
  /* idea 2:
     if it is close to its minimum
   */
  if( satAvg1Errors.val(winner,0) < satMinErrors.val(winner,0)*2 ){
    const Matrix& x_t   = x_buffer[t%buffersize];
    if(conf.useDerive){
      const Matrix& xp_t  = xp_buffer[t%buffersize];
      satInput   = x_t.above(xp_t);
    } else {
      const Matrix& x_tm1 = x_buffer[(t-1)%buffersize];
      satInput   = x_t.above(x_tm1);
    }
    if(conf.useY){
      const Matrix& y_tm1 = y_buffer[(t-1)%buffersize];
      satInput.toAbove(y_tm1);
    }
    const Matrix& out = sats[winner].net->process(satInput);
    return out.rows(x_t.getM(), out.getM()-1);
  }else{
    return Matrix();
  }
}


Matrix MultiSat::compete()
{
  //  const Matrix& x_context = x_context_buffer[t%buffersize];
  const Matrix& x = x_buffer[t%buffersize];

  const Matrix& x_tm1 = x_buffer[(t-1)%buffersize];
  const Matrix& y_tm1 = y_buffer[(t-1)%buffersize];

  // depending on useDerive we have
  // to use F(x_{t-1},x_{t-2} | \dot x_{t-1} ,y_{t-2}) -> (x_t, y_{t-1}) for the sat network

  nomSatOutput = x.above(y_tm1);
  if(conf.useDerive){
    const Matrix& xp_tm1 = xp_buffer[(t-1)%buffersize];
    satInput   = x_tm1.above(xp_tm1);
  }else{
    const Matrix& x_tm2 = x_buffer[(t-2)%buffersize];
    satInput   = x_tm1.above(x_tm2);
  }
  if(conf.useY){
    const Matrix& y_tm2 = y_buffer[(t-2)%buffersize];
    satInput.toAbove(y_tm2);
  }

  // ask all networks to make there predictions on last timestep, compare with real world
  assert(satErrors.getM()>=sats.size());

  unsigned int i=0;
  FOREACH(vector<Sat>, sats, s){
    const Matrix& out = s->net->process(satInput);
    satErrors.val(i,0) =  (nomSatOutput-out).multTM().val(0,0);
    i++;
  }
  if(runcompetefirsttime){
    satAvg1Errors=satErrors;
    satAvg2Errors=satErrors;
    satMinErrors=satAvg2Errors;
    runcompetefirsttime=false;
  }
  satAvg1Errors = satAvg1Errors * (1.0-1.0/conf.tauE1) + satErrors * (1.0/conf.tauE1);
  satAvg2Errors = satAvg2Errors * (1.0-1.0/conf.tauE2) + satErrors * (1.0/conf.tauE2);

  // modulate predicted error to prefere mature experts
  Matrix lambdaW = satEpsMod.mapP(&conf.lambda_w, constant);
  satModErrors = satAvg1Errors.multrowwise(satEpsMod + lambdaW);

  // modulate predicted error of winner and companion to introduce hysteresis
    //  satModErrors = satAvg1Errors;
  satModErrors.val(companion,0)+=0.03;
//  satModErrors.val(winner,0)-=conf.lambda_w/5;
  return satModErrors;
}


Matrix MultiSat::calcDerivatives(const matrix::Matrix* buffer,int delay){
  int t1 = t+buffersize;
  const Matrix& xt    = buffer[(t1-delay)%buffersize];
  const Matrix& xtm1  = buffer[(t1-delay-1)%buffersize];
  const Matrix& xtm2  = buffer[(t1-delay-2)%buffersize];
  return ((xt - xtm1) * 5).above((xt - xtm1*2 + xtm2)*10);
}

double multisat_min(void* m, double d){
  return min(*(double*)m,d);
}

void MultiSat::management(){
  // annealing of neighbourhood learning
  FOREACH(vector<Sat> , sats, s){
    s->lifetime+=managementInterval;
  }
  // conf.lambda_comp = t * (1.0/conf.tauC);

  // decay minima and learning rate modulations
  Matrix deltaM (satMinErrors.getM(),1);
  double delta = (conf.deltaMin*(double)managementInterval/1000.0);
  deltaM.toMapP(&delta, constant); // fill matrix with delta
  satMinErrors += deltaM;
  satEpsMod += deltaM; // maybe also limit it to 1
  double m=1.0;
  satEpsMod.toMapP(&m, multisat_min );

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
  keylist += pair<paramkey, paramval>("epsSat",sats[0].eps);
  return keylist;
}


bool MultiSat::store(FILE* f) const {
  fprintf(f,"%i\n", conf.numSats);
  fprintf(f,"%i\n", conf.numContext);
  fprintf(f,"%i\n", conf.numHidden);

  fprintf(f,"%i\n", runcompetefirsttime);

  // save matrix values
  satErrors.store(f);
  satAvg1Errors.store(f);
  satAvg2Errors.store(f);
  satModErrors.store(f);
  satMinErrors.store(f);
  satEpsMod.store(f);

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
  conf.numHidden = atoi(buffer);

 // we need to use fgets in order to avoid spurious effects with following matrix (binary)
  if((fgets(buffer,128, f))==NULL) return false;
  runcompetefirsttime = atoi(buffer);

  // restore matrix values
  satErrors.restore(f);
  satAvg1Errors.restore(f);
  satAvg2Errors.restore(f);
  satModErrors.restore(f);
  satMinErrors.restore(f);
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

  keylist += storeVectorFieldNames(x_context_buffer[0], "XC");
  keylist += storeVectorFieldNames(satErrors, "errs");
  keylist += storeVectorFieldNames(satAvg1Errors, "avg1errs");
  keylist += storeVectorFieldNames(satAvg2Errors, "avg2errs");
  keylist += storeVectorFieldNames(satModErrors, "merrs");
  keylist += storeVectorFieldNames(satMinErrors, "minerrs");
  keylist += storeVectorFieldNames(satEpsMod, "epsmod");
  keylist += string("winner");
  keylist += string("winner_error");
  keylist += string("companion");
  keylist += string("companion_error");
  keylist += string("satctrl");
  return keylist;
}

list<Inspectable::iparamval> MultiSat::getInternalParams() const {
  list<iparamval> l;
  l += x_context_buffer[t%buffersize].convertToList();
  l += satErrors.convertToList();
  l += satAvg1Errors.convertToList();
  l += satAvg2Errors.convertToList();
  l += satModErrors.convertToList();
  l += satMinErrors.convertToList();
  l += satEpsMod.convertToList();
  l += (double)winner;
  l += (double)satAvg1Errors.val(winner,0);
  l += (double)companion;
  l += (double)satAvg1Errors.val(companion,0);
  l += (double)satControl;
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

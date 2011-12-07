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

#include "soml.h"
#include "controller_misc.h"
using namespace matrix;
using namespace std;

SoML::SoML(const SoMLConf& conf)
  : AbstractController("SoML", "0.1"), conf(conf) {
  t=0;

  addParameterDef("creativity",&creativity,0, 0, 100, "creativity term (0: disabled) ");
  addParameterDef("epsC", &epsC, 0.1, 0, 1, "learning rate of the controller");
  addParameterDef("epsA", &epsA, 0.1, 0, 1,"learning rate of the model");
  addParameterDef("harmony", &harmony, 0.00001, 0, 1, "harmony term");
  addParameterDef("s4avg", &s4avg, 1, 0, 20, "number of steps to smooth the sensor values");
  addParameterDef("s4delay", &s4delay, 1, 0, 20,
                  "number of steps to delay motor values (delay in the loop)");
  addParameterDef("dampA", &dampA, 0.00001, 0, 1, "damping factor for model learning");
  if(conf.useS) addParameterDef("discountS", &discountS, 0.005, 0, 1, "discounting od the S term");
  addParameterDef("biasnoise", &biasnoise, 0.01, 0, 1, "noise for h terms");
  addParameterDef("Logarithmic", &loga, true, "whether to use logarithmic error");

  cNet=0;
};

SoML::~SoML(){
  if (cNet) delete cNet;
}

ControllerNet* SoML::getCNet(){
  return cNet;
}


void SoML::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
 
  number_sensors= sensornumber;
  number_motors = motornumber;

  numControllerLayer=1;
  // create model and controller network (all in one)
  vector<Layer> layers;
  if(conf.useHiddenContr){
    layers.push_back(Layer((int)number_motors * conf.hiddenContrUnitsRatio, 
                           1.0, FeedForwardNN::tanhr)); // hidden layer
    numControllerLayer++;
  }
  // controller output layer
  layers.push_back(Layer(number_motors, 1.0, FeedForwardNN::tanhr)); 
  // model hidden layer 
  if(conf.useHiddenModel)
    layers.push_back(Layer((int)number_motors * conf.hiddenModelUnitsRatio, 
                           1, FeedForwardNN::tanhr)); // hidden layer
  // model output layer (dimensionality will be set automatically at init())
  layers.push_back(Layer(1, 1, FeedForwardNN::linear)); 
  
  cNet = new ControllerNet(layers, conf.useS);  

  //cNet->init(number_sensors, number_sensors, 1.0, 0.2, randGen); 
  cNet->init(number_sensors, number_sensors, conf.initUnitMatrix ? 1.0 : 0.2, 0.1, randGen);   

  bool sIP = conf.someInternalParams;
  for(unsigned int l=0; l< numControllerLayer; l++){
    addInspectableMatrix("C" + itos(l+1), &(cNet->getWeights(l)), sIP, 
                         "controller matrix of layer " +  itos(l) );
    addInspectableMatrix("h" + itos(l+1), &(cNet->getBias(l)), sIP, 
                         "controller bias of layer " +  itos(l));    
  }
  for(unsigned int l=numControllerLayer; l< cNet->getLayerNum(); l++){  
    int i = l-numControllerLayer;
    addInspectableMatrix("A" + itos(i+1), &(cNet->getWeights(l)), sIP, 
                         "model matrix of layer " +  itos(i) );
    addInspectableMatrix("b" + itos(i+1), &(cNet->getBias(l)), sIP, 
                         "model bias of layer " +  itos(i));    
  }
  if(conf.useS){
    addInspectableMatrix("S", &(cNet->getByPass()), sIP, 
                         "S (additional model matrix");    
  }
  addInspectableMatrix("L", &cNet->response(), sIP, "Jacobian of sensorimotor loop" ); 
  addInspectableMatrix("R", &cNet->responseLinear(), sIP, "Linearized Jacobian of sensorimotor loop" ); 
  
  addInspectableValue("E", &E, "Error" ); 
  
  //cout << test << endl;

  x.set(number_sensors,1);
  x_smooth.set(number_sensors,1);
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);
    y_buffer[k].set(number_motors,1);
  }
  eta_avg.set(number_motors,1);
  //  addInspectableMatrix("eta_avg", &eta_avg, sIP, "Average shift at motors");

}


/// performs one step (includes learning). Calulates motor commands from sensor inputs.
void SoML::step(const sensor* x_, int number_sensors, 
                       motor* y_, int number_motors){
  x.set(number_sensors,1,x_); // store sensor values  
  x_buffer[t%buffersize] = x; // Put new output vector in ring buffer y_buffer

  // the current x is used to train the controller based on the previous activation
  if(t>=buffersize){
    // calculate effective input/output, which is (actual-steps4delay) element of buffer
    const Matrix& y_effective = y_buffer[(t - max(s4delay,1) + buffersize) % buffersize];
    
    // learn controller with effective input/output
    learn(x, y_effective);
  }
  // then we calculate the new motor output.
  control(x,y_,number_motors);
  
  t++;  // update step counter
};

// performs one step without learning. Calulates motor commands from sensor inputs.
void SoML::stepNoLearning(const sensor* x_, int number_sensors, 
                                 motor* y_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors 
         && (unsigned)number_motors <= this->number_motors);
  
  x.set(number_sensors,1,x_); // store sensor values
  x_buffer[t%buffersize] = x;   // Put new output vector in ring buffer y_buffer
  
  control(x,y_,number_motors);

  t++;  // update step counter
};
  
// performs control step (activates network and stores results in buffer and y_)
void SoML::control(const Matrix& x, motor* y_, int number_motors){
  // averaging over the last s4avg values of x_buffer
  x_smooth += (x - x_smooth)*(1.0/max(s4avg,1));

  cNet->process(x);

  // output is in the controller layer  + creativity
  const Matrix& y =  cNet->getLayerOutput(numControllerLayer-1) + eta_avg*creativity;
  
  y_buffer[t%buffersize] = y;   // Put new output vector in ring buffer y_buffer  
  y.convertToBuffer(y_, number_motors); // convert y to motor* 
}

// learn cNet
void SoML::learn(const Matrix& x, const Matrix& y){
  // Note: The network is activated in the control function (last timestep)
  const Matrix& xp = cNet->getLayerOutput(-1); // previous output of the network  
  const Matrix& xi = x  - xp;
 
  // learn model with backprop
  learnModelBP(1);

  /************ TLE (homeokinetic) learning **********/
  // Matrix  eta = (A^-1) * xi; 
  //  const Matrix&  eta = A.pseudoInverse(0.001) * xi; 

  Matrices zeta;
  Matrices v;
  const Matrix& L = cNet->response();
  const Matrix& chi = (L.multMT().secureInverse()) * xi;
  const Matrix& v0   = (L^T)*chi;
  E = v0.norm_sqr();
  
  cNet->forwardpropagation(v0, &v, &zeta);
  eta_avg = eta_avg*.9 + v[numControllerLayer]*.1;

  E = loga ? 0.1/(E+0.0001) : 1.0; 
  double factor = 1;
      
  Matrices mu;  
  cNet->backpropagation(chi, 0, &mu);
  
  // learning rule
  // TODO: the effective y is not used here!
  for(unsigned int l=0; l < numControllerLayer; l++){    
    const Matrix& epsl =  (mu[l] & zeta[l]) * (epsC * E);
    const Matrix& y     = cNet->getLayerOutput(l);
    const Matrix& y_lm1 = l==0 ? x : cNet->getLayerOutput(l-1);
    cNet->getWeights(l) += ((mu[l] * (v[l]^T) * (epsC * E)) 
                            - ((y * (y_lm1^T)) & epsl * (2 * factor))).mapP(0.03, clip);
    cNet->getBias(l) += (y & epsl * (-2 * factor)).mapP(0.03, clip);
    if(epsC!=0){
      cNet->getBias(l) += cNet->getBias(l).map(random_minusone_to_one)*biasnoise 
        - cNet->getBias(l)*0.001;
    }
    
  }

  // Harmony
  if(harmony!=0){
    Matrices delta;
    cNet->backpropagation(chi, 0, &delta);  
    for(unsigned int l = 0; l<cNet->getLayerNum(); l++){
      const Matrix& ylm1 = l==0 ? x : cNet->getLayerOutput(l-1);
      cNet->getWeights(l) += ((delta[l] * (ylm1^T)) * harmony * epsC).mapP(0.03, clip);
      cNet->getBias(l)    += (delta[l] * harmony * epsC).mapP(0.03, clip);
    } 
  }
        
}


void SoML::learnModelBP(double factor){
  // we assume te network is activated with sensor value from last time step (not current x)
  const Matrix& xp = cNet->getLayerOutput(-1); // previous output of the network  
  const Matrix& xi = x  - xp;
  
  // learn model with backprop
  Matrices delta;
  cNet->backpropagation(xi, 0, &delta);  

  for(unsigned int l = numControllerLayer; l<cNet->getLayerNum(); l++){
    Matrix& Al=cNet->getWeights(l);
    Al += ((delta[l] * (cNet->getLayerOutput(l-1)^T)) * (epsA * factor)
           - (Al * dampA) * ( epsA > 0 ? 1 : 0)).mapP(0.03, clip);
    Matrix& bl=cNet->getBias(l);
    bl += (delta[l] * (epsA * factor) - (bl * dampA) * ( epsA > 0 ? 1 : 0) ).mapP(0.03, clip);  
  } 
  if(conf.useS){    
    Matrix& S = cNet->getByPass();
    // discount is multiplied with learning rate
    S += ((delta[cNet->getLayerNum()-1] * (x^T) - S * discountS) * ( epsA * factor)).mapP(0.03, clip);
  }
}

void SoML::motorBabblingStep(const sensor* x_, int number_sensors,
                            const motor* y_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors 
         && (unsigned)number_motors <= this->number_motors);
  x.set(number_sensors,1,x_);
  x_buffer[t%buffersize] = x;
  Matrix y(number_motors,1,y_);
  y_buffer[t%buffersize] = y;

  double factor = .1; // we learn slower here
  // learn model:
  const Matrix& x_tm1 = x_buffer[(t - 1 + buffersize) % buffersize];
  const Matrix& y_tm1 = y_buffer[(t - 1 + buffersize) % buffersize];
  const Matrix xp = cNet->processX(x_tm1,y_tm1,numControllerLayer-1); // process from controller layer of  
  learnModelBP(factor);
  
  // learn controller:
  Matrices delta;
  cNet->process(x);
  // output is in the controller layer 
  const Matrix& our_y =  cNet->getLayerOutput(numControllerLayer-1);
  // do backprop learning of controller
  cNet->backpropagationX(y-our_y, 0, &delta, numControllerLayer-1);  
  for(unsigned int l = 0; l < numControllerLayer; l++){
    const Matrix& ylm1 = l==0 ? x^T : cNet->getLayerOutput(l-1)^T;
    cNet->getWeights(l) += delta[l] * ylm1 * (epsC * factor);
    cNet->getBias(l) += delta[l] * (epsC * factor);  
  } 
  
  t++;
}


/** stores the controller values to a given file. */
bool SoML::store(FILE* f) const{  
  // save matrix values
  cNet->store(f);  
  Configurable::print(f,0);
  return true;
}

/** loads the controller values from a given file. */
bool SoML::restore(FILE* f){
  // save matrix values
  cNet->restore(f);  
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}


/* Ablage

Discount stuff:
  // if(conf.useS && discountS > 0)
  //   cNet->backpropagationX(xi + cNet->getByPass()*(x_tm1*discountS), 0, &delta);
  // else
    // if(discountS > 0) // now we activate again with the undiscounted error
    //   cNet->backpropagationX(xi - (cNet->response()- cNet->getByPass())*(x_tm1*discountS), 0, &delta)


  // // also try sensitization
  // const Matrix& A = cNet->responsePart(numControllerLayer-1,-1); // effective A
  // const Matrix& kabba = (A.multMT()^(-1))* xi;
  // const Matrix& eta   = (A^T)*kabba;

***/

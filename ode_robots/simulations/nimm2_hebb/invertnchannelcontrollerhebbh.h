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
 *                                                                         *
 *   $Log$
 *   Revision 1.1  2009-04-03 10:43:42  fhesse
 *   invertnchennelcontrollerhebb* moved to
 *   simulations (hand and nimm2_hebb)
 *
 *   Revision 1.6  2008/05/30 11:58:27  martius
 *   use cmath instead of math.h
 *
 *   Revision 1.5  2008/04/17 14:54:45  martius
 *   randomGen added, which is a random generator with long period and an
 *    internal state. Each Agent has an instance and passed it to the controller
 *    and the wiring. This is good for
 *   a) repeatability on agent basis,
 *   b) parallel execution as done in ode_robots
 *
 *   Revision 1.4  2007/10/10 08:44:27  fhesse
 *   testing
 *
 *   Revision 1.3  2007/10/08 20:14:43  fhesse
 *   switch setHbackto0 added
 *   now different methods to limit
 *
 *   Revision 1.2  2007/10/05 19:51:10  fhesse
 *   use_hebb switch added
 *   setting H to zero when context switch off
 *
 *   Revision 1.1  2007/09/28 15:51:48  fhesse
 *   initial version
 *                             *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/
#ifndef __INVERTNCHANNELCONTROLLERHEBBH_H
#define __INVERTNCHANNELCONTROLLERHEBBH_H

#include <selforg/invertnchannelcontroller.h>

#include <assert.h>
#include <cmath>

#include <selforg/matrix.h>



/**
 * class for robot controller that uses the georg's matrixlib for
 *  direct matrix inversion for n channels
 * (simple one layer networks)
 *
 * Implements standart parameters: eps, rho, mu, stepnumber4avg, stepnumber4delay
 */
class InvertNChannelControllerHebbH : public InvertNChannelController {

public:
  InvertNChannelControllerHebbH(int _buffersize, bool _update_only_1=false, bool inactivate_hebb=false);
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

  virtual ~InvertNChannelControllerHebbH();

  /// returns the number of sensors the controller was initialised with or 0 if not initialised
  virtual int getSensorNumber() const { return number_all_sensors; }


  /// performs one step (includes learning).
  /// Calulates motor commands from sensor inputs.
  virtual void step(const sensor* , int number_sensors, motor* , int number_motors);


  /// performs one step without learning. Calulates motor commands from sensor inputs.
  //  virtual void stepNoLearning(const sensor* , int number_sensors,
  //                      motor* , int number_motors);


  // inspectable interface
  virtual std::list<iparamkey> getInternalParamNames() const;
  virtual std::list<iparamval> getInternalParams() const;



  virtual paramval getParam(const paramkey& key) const{
    if(key == "eps_hebb") return eps_hebb;
    else if(key == "fact_eps_h") return fact_eps_h;
    else if(key == "use_hebb") return use_hebb;
    else if(key == "setHbackto0") return setHbackto0;
    else  return InvertNChannelController::getParam(key) ;
  }

  virtual bool setParam(const paramkey& key, paramval val){
    if(key == "eps_hebb") eps_hebb=val;
    else if(key == "fact_eps_h") fact_eps_h=val;
    else if(key == "use_hebb") use_hebb=val;
    else if(key == "setHbackto0") setHbackto0=val;
    else  return InvertNChannelController::setParam(key,val) ;
    return true;
  }

  virtual paramlist getParamList() const{
    paramlist list = InvertNChannelController::getParamList();
    list.push_back(std::pair<paramkey, paramval> ("eps_hebb", eps_hebb));
    list.push_back(std::pair<paramkey, paramval> ("fact_eps_h", fact_eps_h));
    list.push_back(std::pair<paramkey, paramval> ("use_hebb", use_hebb));
    list.push_back(std::pair<paramkey, paramval> ("setHbackto0", setHbackto0));
    return list;
  }


protected:
  /** number of sensors used for the homeokinetic sensor*/
  int number_hom_sensors;
  /** number of context sensors*/
  int number_context_sensors;

  /** buffer for context sensors*/
  matrix::Matrix* context_buffer;



  paramval eps_hebb;
  paramval use_hebb;
  paramval setHbackto0;
  paramval fact_eps_h;
  unsigned int number_motors; // number of motors used

  bool hebb_inactive; //if true: deactivates hebb learning part and the context sensors, so it is the normal homeokinetic controller (invertnchannelcontroller)

  unsigned short number_all_sensors;
  matrix::Matrix xsi_org;  // modeling error homeokinese
  matrix::Matrix xsi_hebb; // modeling error hebb
  sensor* all_sensors; //memory for all sensor values (used for hebbian learning)
  matrix::Matrix p; //weights for hebbian learning
  matrix::Matrix sensors_mean; // mean of sensors for hebb learning

  matrix::Matrix h_pred_update; //Matrix of changes of H predicted by hebbian ...
  matrix::Matrix h_update; // Matrix of changes of H (fomr homeokinetic controller)



  matrix::Matrix x_delay_; //weights for hebbian learning
  matrix:: Matrix lay2_nchan_sens; //sensors for InvertNChannelController (should not contain context sensors!)

  //double p0_pos, p0_neg, p1_pos, p1_neg;

  sensor old_sensors[10]; //memory of old sensor values (used for hebbian learning)

  //  virtual matrix::Matrix hebb(matrix::Matrix& xsi, sensor* sensors);


  /**
   * learn hebb layer to predict dH (h_update.val)
   * @param context_sensors input tu hebbian layer
   * @param h_update desired outputother input of hebbian layer
   */
  void learnHebb(const matrix::Matrix& context_sensors, const matrix::Matrix& h_update);

  /**
   * predict the update of h based on the actual context sensors
   * @param context_sensors prediction is based on these sensors
   */
  matrix::Matrix predictHebb(const matrix::Matrix& context_sensors);


  //  virtual double calculateE_(const matrix::Matrix& x_delay, const matrix::Matrix& y_delay);
  virtual void learn(const matrix::Matrix& x_delay, const matrix::Matrix& y_delay);

};

#endif


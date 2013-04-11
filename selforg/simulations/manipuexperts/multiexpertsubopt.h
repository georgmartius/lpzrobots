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
#ifndef __MULTIEXPERTSUBOPT_H
#define __MULTIEXPERTSUBOPT_H


#include <assert.h>
#include <cmath>
#include <vector>

#include <selforg/matrix.h>
#include <selforg/noisegenerator.h>
#include <selforg/invertablemodel.h>

typedef struct MultiExpertSuboptConf {
  int numHidden;        ///< number of hidden units in the satelite networks
  double eps0;          ///< learning rate for satelite networks
  double tauE1;         ///< time horizont for short averaging error
  double tauE2;         ///< time horizont for long averaging error
  double tauF;          ///< time scale for forgetting minimum (should be very large)
  int    numSats;       ///< number of satelite networks
  double lambda_comp;   ///< competition for ranking

  int    satMemory;    ///< number of memory cells for past patterns
  int    satTrainPast; ///< number of past patterns that are trained each timestep
  int    version;      ///< type of learning schema, see MultiExpertSubopt::Version
} MultiExpertSuboptConf;


/**
 * class for robot controller
 * using several feedforward networks (satelite) and one selforg controller
 */
class MultiExpertSubopt : public  AbstractModel{

public:
  /**
   *     a) only one sat learns, which is least suboptimal                   *
   *     b) learning rates are based on a rank of the suboptimalities        *
   *     c) learning rates are is based on relative suboptimality and softmax*
   *          (not yet tried)                                                  *
  */
  typedef enum Version {A, B, C};

  /// Satelite network struct
  typedef struct Sat {
    Sat(InvertableModel* _net, double _eps);
    InvertableModel* net;
    double eps;
    double lifetime;
  } Sat;
public:
  MultiExpertSubopt(const MultiExpertSuboptConf& conf = getDefaultConf());

  virtual ~MultiExpertSubopt();

  virtual void init(unsigned int inputDim, unsigned  int outputDim,
                    double unit_map = 0.0, RandGen* randGen = 0);

  virtual unsigned int getInputDim() const { return inputDim;}
  virtual unsigned int getOutputDim() const  { return outputDim;}

  virtual void damp(double damping) {};

  virtual const matrix::Matrix process (const matrix::Matrix& input);

  virtual const matrix::Matrix learn (const matrix::Matrix& input,
                                      const matrix::Matrix& nom_output,
                                      double learnRateFactor = 1);

  // !!!!!!!!!!!!!!!!!!! MISC STUFF !!!!!!!!
  /** minimum dynamics
      @param conf MultiExpertSuboptConf
      @param m minimum error
      @param e current error
   */
  static double mindynamics(void *conf, double m, double e);


  /// stores the sat networks into seperate files
  void storeSats(const char* filestem);
  /// restore the sat networks from seperate files
  void restoreSats(const std::list<std::string>& filenames);

  /************** CONFIGURABLE ********************************/
  virtual paramval getParam(const paramkey& key, bool traverseChildren=true) const;
  virtual bool setParam(const paramkey& key, paramval val, bool traverseChildren=true);
  virtual paramlist getParamList() const;


  /**** STOREABLE ****/
  /** stores the controller values to a given file. */
  virtual bool store(FILE* f) const;
  /** loads the controller values from a given file. */
  virtual bool restore(FILE* f);

  /**** INSPECTABLE ****/
  virtual std::list<iparamkey> getInternalParamNames() const;
  virtual std::list<iparamval> getInternalParams() const;
  virtual std::list<ILayer> getStructuralLayers() const;
  virtual std::list<IConnection> getStructuralConnections() const;

  static MultiExpertSuboptConf getDefaultConf(){
    MultiExpertSuboptConf c;
    c.numHidden = 2;
    c.eps0      = 0.05;
    c.tauF      = 10000;
    c.tauE1     = 5;
    c.tauE2     = 50;
    c.numSats   = 20;
    c.lambda_comp = 2;

    c.satMemory    = 1;
    c.satTrainPast = 0;
    c.version = B;
    return c;
  }


protected:
  int inputDim;
  int outputDim;

  std::vector <Sat> sats; ///< satelite networks
  int winner;    ///< index of winner network

  bool runcompetefirsttime;       ///< flag to initialise averaging with proper value
  matrix::Matrix satErrors;       ///< actual errors of the sats
  matrix::Matrix satAvg1Errors;   ///< short averaged errors of sats
  matrix::Matrix satAvg2Errors;   ///< long averaged errors of sats
  matrix::Matrix satSubOpt;       ///< suboptimality of sats
  matrix::Matrix satMinErrors;    ///< minimum errors of sats (calculated from avg2)
  matrix::Matrix satEpsMod;       ///< modulated eps of sats

  MultiExpertSuboptConf conf;
  bool initialised;
  bool managementInterval;
  int t;
protected:
  /// satelite networks competition, return vector of prediction errors of sat networks
  matrix::Matrix compete(const matrix::Matrix& input,
                         const matrix::Matrix& nom_output);

  void management();

};

#endif

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
#ifndef __QLEARNING_H
#define __QLEARNING_H

#include "matrix.h"
#include "configurable.h"
#include "storeable.h"
#include "randomgenerator.h"

/// implements QLearning
class QLearning : public Configurable, public Storeable {
public:
  /**
     \param eps learning rate (typically 0.1)
     \param discount discount factor for Q-values (typically 0.9)
     \param exploration exploration rate (typically 0.02)
     \param eligibility number of steps to update backwards in time
     \param random_initQ if true Q table is filled with small random numbers at the start (default: false)
     \param useSARSA if true, use SARSA strategy otherwise qlearning (default: false)
     \param tau number of time steps to average over reward for col_rew
   */
  QLearning(double eps, double discount, double exploration, int eligibility,
            bool random_initQ = false, bool useSARSA = false, int tau=1000);

  virtual ~QLearning();

  /** initialisation with the given number of action and states
      @param actionDim number of actions
      @param stateDim number of states
      @param unit_map if 0 the parametes are choosen randomly.
      Otherwise the model is initialised to represent a unit_map with the given response strength.
  */
  virtual void init(unsigned  int stateDim, unsigned int actionDim, RandGen* randGen = 0);

  /** selection of action given current state.
      The policy is to take the actions with the highest value,
      or a random action at the rate of exploration
  */
  virtual unsigned int select (unsigned int state);

  /** selection of action given current state.
      The policy is to sample from the above average actions, with bias
      to the old action (also exploration included).
  */
  virtual unsigned int select_sample (unsigned int state);
  /// select with preference to old (90% if good) and 30% second best
  virtual unsigned int select_keepold (unsigned int state);

  /* performs learning and returns current expected reward.
     \param state current state
     \param action we select in current state
     \param reward reinforcement we obtain in this state
     \param learnRateFactor can be given to modify eps for this
     learning step
  */
  virtual double learn (unsigned int state,
                        unsigned int action,
                        double reward,
                        double learnRateFactor = 1);

  /** returns the vector of values for all actions given the current state
   */
  matrix::Matrix getActionValues(unsigned int state);


  /** tells the q learning that the agent was reset, so that it
      forgets it memory. please note, that updating the Q-table is
      one step later, so in case of a reward you should call learn one
      more time before reset.
  */
  virtual void reset();


  /// returns the number of states
  virtual unsigned int getStateDim() const;
  /// returns the number of actions
  virtual unsigned int getActionDim() const;

  /// returns the collectedReward reward
  virtual double getCollectedReward() const;

  /// expects a list of value,range and returns the associated state
  static int valInCrossProd(const std::list<std::pair<int,int> >& vals);

  /// expects a list of ranges and a state/action and return the configuration
  static std::list<int> ConfInCrossProd(const std::list<int>& ranges, int val);

  /// returns q table (mxn) == (states x actions)
  virtual const matrix::Matrix& getQ() const {return Q;} ;

  virtual bool store(FILE* f) const;

  virtual bool restore(FILE* f);


protected:
  double eps;
  double discount;
  double exploration;
  double eligibility; // is used as integer (only for configration)
  bool random_initQ;
public:
  bool useSARSA; ///< if true, use SARSA strategy otherwise qlearning
protected:
  int tau;       ///< time horizont for averaging the reward
  matrix::Matrix Q; /// < Q table (mxn) == (states x actions)


  int* actions;    // ring buffer for actions
  int* states;     // ring buffer for states
  double* rewards; // ring buffer for rewards
  int ringbuffersize; // size of ring buffers, eligibility + 1
  double* longrewards; // long ring buffer for rewards for collectedReward
  int t; // time for ring buffers
  bool initialised;
  double collectedReward; // sum over collected reward

  RandGen* randGen;
};


#endif

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
#ifndef __FEEDBACKWIRING_H
#define __FEEDBACKWIRING_H

#include "abstractwiring.h"
#include "matrix.h"

/** Implements essentionally a one to one wiring with feedback connections.
    The feedback connections from output to input are parameterised
     with a feedback strength.
    It is possible to generate virtual motors for context sensors.

    In order to change the feedback strength after initialisation
    use the following code
    \code
    matrix::Matrix rs = wiring->getFeedbackRatio();
    double c=ratio;
    rs.toMapP(&c,constant);
    wiring->setFeedbackRatio(rs);
    \endcode
 */
class FeedbackWiring :public AbstractWiring{
public:
  typedef enum {Motor=1, Context=2, All=3} Mode;

  /** constructor
      @param noise NoiseGenerator that is used for adding noise to sensor values
      @param mode Motor|Context|All: Motor: motor outputs send feedback;
           Context: virtual motor outputs for each context sensor with feedback
      @param feedbackratio default ratio used to feed back the output to the input,
         meaning \f[ x_t = 0.1*x_t + 0.9*y_{t-1} \f]
   */
  FeedbackWiring(NoiseGenerator* noise, Mode mode = Context,double feedbackratio=0.9, const std::string& name = "FeedBackWiring");
  virtual ~FeedbackWiring();

protected:
  virtual bool initIntern();

  virtual bool wireSensorsIntern(const sensor* rsensors, int rsensornumber,
                                 sensor* csensors, int csensornumber,
                                 double noise);

  virtual bool wireMotorsIntern(motor* rmotors, int rmotornumber,
                                const motor* cmotors, int cmotornumber);

public:

  virtual std::list<iparamkey> getInternalParamNames() const;
  virtual std::list<iparamval> getInternalParams() const;

  /// return the feedback ratio vector
  virtual matrix::Matrix getFeedbackRatio() const;
  /** sets the feedback ratio vector.
      The size of the vector must be at least as large as getFeedbackRatio()*/
  virtual void setFeedbackRatio(const matrix::Matrix&);

protected:

  Mode mode;
  double defaultfeedbackratio;
  matrix::Matrix feedbackratio;
  /// array that stored the values of the motors
  motor *motors;
  int vmotornumber;

};

#endif

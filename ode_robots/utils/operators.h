/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
+ *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
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
#ifndef __OPERATORS_H
#define __OPERATORS_H

#include "operator.h"
#include "axis.h"

namespace lpzrobots {
  /**
     An Operator for limiting the orientation of the main primitive of a robot.
     If the robot axis has a larger enclosing angle with the global axis than
     maxAngle the force is applied until the angle is lower than minAngle.
     If minAngle<0 then minAngle=maxAngle/2.
   */
  class LimitOrientationOperator : public Operator {
  public:
    LimitOrientationOperator(const Axis& robotAxis, const Axis& globalAxis,
                             double maxAngle, double force, double minAngle=-1)
      : Operator("LimitOrientationOperator","1.0"),
        robotAxis(robotAxis), globalAxis(globalAxis),
        maxAngle(maxAngle), force(force), minAngle(minAngle),
        currentforce(force), active(false) {
      if(this->minAngle<0) this->minAngle=maxAngle/2;
    }

    virtual ManipType observe(OdeAgent* agent, GlobalData& global, ManipDescr& descr);
  protected:
    Axis robotAxis;
    Axis globalAxis;
    double maxAngle;
    double force;
    double minAngle;
    double currentforce;
    bool active;
  };


  struct LiftUpOperatorConf {
    bool   resetForceIfLifted;  ///< force is reseted as soon as robot has reached height
    bool   increaseForce;       ///< increase force (until robot has reached height)
    bool   propControl;         ///< if true then applied force is scaled with distance

    bool   intervalMode;        ///< if true then the operator works only in intervals
    double duration;            ///< duration of operation in interval-mode
    double interval;            ///< interval between restart of operation ( > duration)

    double height;              ///< height to which it is lifted
    double force;               ///< initial force
    double visualHeight;        ///< height above the robot main object for the visual sphere
  };

  /**
     An Operator for lifting up a robot from time to time.
   */
  class LiftUpOperator : public Operator {
  public:
    LiftUpOperator(const LiftUpOperatorConf conf = getDefaultConf())
      : Operator("LiftUpOperator","0.8"), conf(conf)
    {
      currentforce = conf.force;
      addParameter("force",    &this->conf.force,   0, 100, "lift up force");
      addParameter("height",   &this->conf.height,  0, 100, "lift up height");
      if(conf.intervalMode){
        addParameter("interval", &this->conf.interval,  0, 1000, "interval of operation");
        addParameter("duration", &this->conf.duration,  0, 1000, "duration of lifting within interval");
      }
    }

    static LiftUpOperatorConf getDefaultConf(){
      LiftUpOperatorConf c;
      c.resetForceIfLifted = true;
      c.increaseForce      = true;
      c.intervalMode       = false;
      c.propControl        = true;
      c.duration           = 1;
      c.interval           = 10;
      c.height             = 1;
      c.force              = 1;
      c.visualHeight       = 0.5;
      return c;
    }

    virtual ManipType observe(OdeAgent* agent, GlobalData& global, ManipDescr& descr);
  protected:
    LiftUpOperatorConf conf;

    double currentforce;
  };


  /**
     An Operator for pulling the main primitive of a robot towards a point
   */
  class PullToPointOperator : public Operator {
  public:
    /// defines which dimensions should be effected
    enum Dimensions { X = 1, Y = 2, Z = 4, XY = X | Y, XZ = X | Z, YZ = Y | Z,
                      XYZ = X | Y | Z };

    /**
       @param dim dimensions along the force acts
        (also only these coordinates of point are considered)
       @minDist threshold for distance below which no force acts
       @damp damping factor for movement of the robot
       @confPos whether to make the point configurable
     */
    PullToPointOperator(const Pos& point, double force,
                        bool showPoint, Dimensions dim = XYZ,
                        double minDist = 0, double damp = 0, bool confPos = false)

      : Operator("PullToPointOperator","1.0"),
        point(point), force(force), showPoint(showPoint), dim(dim),
        minDist(minDist), damp(damp)
    {
      addParameter("force",    &this->force,   0, 100, "pull to point force");
      addParameter("damp",     &this->damp,   0, 1,   "pull to point damping");
      if(confPos){
        if(dim & X)
          addParameterDef("point_x", &px, point.x(), -100, 100,"pull to point x position");
        if(dim & Y)
          addParameterDef("point_y", &py, point.y(), -100, 100,"pull to point y position");
        if(dim & Z)
          addParameterDef("point_z", &pz, point.z(), -100, 100,"pull to point z position");
      }
    }

    virtual ManipType observe(OdeAgent* agent, GlobalData& global, ManipDescr& descr);

    virtual void notifyOnChange(const paramkey& key);

  protected:
    Pos point;
    double force;
    bool showPoint;
    Dimensions dim;
    double minDist;
    double damp;
    double px,py,pz; // used to configure position (are floats and not doubles)
  };

  /**
     An Operator for keeping robots within a sphere / box
   */
  class BoxRingOperator : public Operator {
  public:

    /** a box ring (cube with edges 2*size or sphere with radius size)
        @param offset distance to wall before acting (to compensate for body width)
        @sphere spherical or box shape area
     */

    BoxRingOperator(const Pos& center, double size, double offset,
                    double force, bool sphere = false)
      : Operator("BoxRingOperator","1.0"),
        center(center), size(size), offset(offset), force(force), sphere(sphere)
    {
      addParameter("force",    &this->force,   0, 1000,
                   "force of the boxring to keep robots inside");
      addParameter("boxringsize", &this->size,   .5, 100,
                   "size of boxring/spherical arena (in radius or half-length)");
    }

    virtual ManipType observe(OdeAgent* agent, GlobalData& global, ManipDescr& descr);

  protected:

    Pos center;
    double size;
    double offset;
    double force;
    bool sphere;
  };



}

#endif

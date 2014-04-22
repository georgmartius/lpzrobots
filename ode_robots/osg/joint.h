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
#ifndef __JOINT_H
#define __JOINT_H

#include <assert.h>
#include <list>

#include "primitive.h"
#include "osgforwarddecl.h"
#include "axis.h"
#include "osghandle.h"
#include "odehandle.h"

namespace lpzrobots {


  /***************************************************************************/

  class Joint {
  public:
    Joint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor)
      : joint(0), part1(part1), part2(part2), anchor(anchor), feedback(0) {
      assert(part1 && part2);
    }
    virtual ~Joint();
    /** initialises (and creates) the joint. If visual is true then the joints is
        also drawn. visualSize is the size of the visual representation.
        If ignoreColl is true then the pair of connected parts is ignored
        at collision handling.
        The member variable odeHandle is set to the given handle.
        (To be overloaded, but this init should be called always from children!)
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                      bool withVisual = true, double visualSize = 0.2,
                      bool ignoreColl = true);

    /// should syncronise the Ode stuff and the OSG notes (if any)
    virtual void update() = 0;
    /// sets the ODE joint parameter (see ODE manual)
    virtual void setParam(int parameter, double value) = 0;
    /// return the ODE joint parameter (see ODE manual)
    virtual double getParam(int parameter) const = 0;

    dJointID getJoint() const  { return joint; }
    const Primitive* getPart1() const { return part1; }
    Primitive* getPart1() { return part1; }
    const Primitive* getPart2() const { return part2; }
    Primitive* getPart2() { return part2; }
    const osg::Vec3 getAnchor() const { return anchor; }

    /// returns the number of Axes
    virtual int getNumberAxes() const = 0;
    /// returns the n'th axis of the joint (starting with 0)
    virtual Axis getAxis(int n) const { return Axis();};

    /// returns the positions of all Axes
    virtual std::list<double> getPositions() const { return std::list<double>(); }
    /// returns the position rates of all Axes
    virtual std::list<double> getPositionRates() const { return std::list<double>(); }
    /// stores the positions of all Axes into sensorarray and returns the number of written entries
    virtual int getPositions(double* sensorarray) const { return 0; }
    /** stores the position rates of all Axes into sensorarray and
        returns the number of written entries
     */
    virtual int getPositionRates(double* sensorarray) const { return 0; }

    /// enable or disable the feedback mode
    virtual void setFeedBackMode(bool mode);
    /// torque applied to body 1 and body 2
    virtual bool getTorqueFeedback(Pos& t1, Pos& t2) const;
    /// force applied to body 1 and body 2
    virtual bool getForceFeedback(Pos& f1, Pos& f2) const;

    static osg::Matrix anchorAxisPose(const osg::Vec3& anchor, const Axis& axis);
  protected:
    dJointID joint;
    Primitive* part1;
    Primitive* part2;
    osg::Vec3 anchor;

    dJointFeedback* feedback;
  public:
    OdeHandle odeHandle;
  };

  class OneAxisJoint : public Joint {
  public:
    OneAxisJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor, const Axis axis1)
      : Joint(part1, part2, anchor), axis1(axis1) {}
    virtual Axis getAxis(int n) const { return axis1;}
    virtual Axis getAxis1() const { return axis1; };

    virtual double getPosition1() const = 0;
    virtual double getPosition1Rate() const = 0;
    virtual void addForce1(double force) = 0;

    virtual int getNumberAxes() const { return 1;};
    virtual std::list<double> getPositions() const;
    virtual std::list<double> getPositionRates() const;
    virtual int getPositions(double* sensorarray) const;
    virtual int getPositionRates(double* sensorarray) const;
  protected:
    Axis axis1;
  };

  class TwoAxisJoint : public OneAxisJoint {
  public:
    TwoAxisJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor, const Axis axis1,
                 const Axis axis2 )
      : OneAxisJoint(part1, part2, anchor, axis1), axis2(axis2) {}
    virtual Axis getAxis(int n) const { if (n==0) return axis1; else return axis2;}
    virtual Axis getAxis2() const { return axis2; };

    virtual double getPosition2() const = 0;
    virtual double getPosition2Rate() const = 0;
    virtual void addForce2(double force) = 0;
    void addForces(double force1,double force2){
      addForce1(force1); addForce2(force2);
    }

    virtual int getNumberAxes() const { return 2;};
    virtual std::list<double> getPositions() const;
    virtual std::list<double> getPositionRates() const;
    virtual int getPositions(double* sensorarray) const;
    virtual int getPositionRates(double* sensorarray) const;

  protected:
    Axis  axis2;
  };

  /***************************************************************************/

  class FixedJoint : public Joint {
  public:
    /** the anchor is only required for drawing.
        If not provided than part1 position is used
    */
    FixedJoint(Primitive* part1, Primitive* part2,
               const osg::Vec3& anchor = osg::Vec3(0,0,0));

    virtual ~FixedJoint();

    /** initialises (and creates) the joint.
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                      bool withVisual = true, double visualSize = 0.2,
                      bool ignoreColl = true);

    virtual void update();
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter) const;

    virtual int getNumberAxes() const { return 0; }
  protected:
    OSGPrimitive* visual;
  };


  /***************************************************************************/

  class HingeJoint : public OneAxisJoint {
  public:
    HingeJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor,
                const Axis& axis1);

    virtual ~HingeJoint();

    /** initialises (and creates) the joint. If visual is true then the axis of the joints is
        also drawn as a slim cylinder. visualSize is the length of the cylinder.
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                      bool withVisual = true, double visualSize = 0.2,
                      bool ignoreColl = true);

    virtual void update();

    virtual void addForce1(double t);
    virtual double getPosition1() const;
    virtual double getPosition1Rate() const;
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter) const;

  protected:
    OSGPrimitive* visual;
  };

  /***************************************************************************/

  class Hinge2Joint : public TwoAxisJoint {
  public:
    Hinge2Joint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor,
                const Axis& axis1, const Axis& axis2);

    virtual ~Hinge2Joint();

    /** initialises (and creates) the joint. If visual is true then axis2 of the joints is
        also drawn as a slim cylinder. visualSize is the length of the cylinder.
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                      bool withVisual = true, double visualSize = 0.2,
                      bool ignoreColl = true);

    virtual void update();

    /// adds torques to axis 1 and 2
    virtual void addForce1(double t1);
    virtual void addForce2(double t2);
    virtual double getPosition1() const;
    virtual double getPosition2() const; /// This is not supported by the joint!
    virtual double getPosition1Rate() const;
    virtual double getPosition2Rate() const;
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter) const;

  protected:
    OSGPrimitive* visual;
  };

  /***************************************************************************/

  class UniversalJoint : public TwoAxisJoint {
  public:
    UniversalJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor,
                const Axis& axis1, const Axis& axis2);

    virtual ~UniversalJoint();

    /** initialises (and creates) the joint. If visual is true then axix1 and axis2 of the joints is
        also drawn as a slim cylinder. visualSize is the length of the cylinder.
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                      bool withVisual = true, double visualSize = 0.2,
                      bool ignoreColl = true);

    virtual void update();

    /// adds torques to axis 1 and 2
    virtual void addForce1(double t1);
    virtual void addForce2(double t2);
    virtual double getPosition1() const;
    virtual double getPosition2() const;
    virtual double getPosition1Rate() const;
    virtual double getPosition2Rate() const;

    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter) const;

  protected:
    OSGPrimitive* visual1;
    OSGPrimitive* visual2;
  };

  /***************************************************************************/

  class BallJoint : public Joint {
  public:
    BallJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor);

    virtual ~BallJoint();

    /** initialises (and creates) the joint.
        If visual is true then ball is drawn as a sphere with radius of visualSize.
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                      bool withVisual = true, double visualSize = 0.2,
                      bool ignoreColl = true);

    virtual void update();

    virtual int getNumberAxes() const { return 0; }
    // Ball and Socket has no parameter
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter) const;

  protected:
    OSGPrimitive* visual;
  };


  /***************************************************************************/

  class SliderJoint : public OneAxisJoint {
  public:
    SliderJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor,
                const Axis& axis1);

    virtual ~SliderJoint();

    /** initialises (and creates) the joint. If visual is true then the axis of the joints is
        also drawn as a slim cylinder. VisualSize is added to the lenght of the slider and is used
        for the length of the cylinder. The radius is visualSize/10
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                      bool withVisual = true, double visualSize = 0.1,
                      bool ignoreColl = true);

    virtual void update();

    virtual void addForce1(double t);
    virtual double getPosition1() const;
    virtual double getPosition1Rate() const;
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter) const;

  protected:
    OSGPrimitive* visual;
    double visualSize;
    OsgHandle osgHandle;
  };


  // /***************************************************************************/
  // // does not work
  // class LMotorJoint : public TwoAxisJoint {
  // public:
  //   /// @param relative: how to anchor the axes: 0: global, 1: first 2: second body
  //   LMotorJoint(Primitive* part1, Primitive* part2, int relative, const Axis& axis1, const Axis& axis2);

  //   virtual ~LMotorJoint();

  //   /** initialises (and creates) the joint. If visual is true then the axes of the joints are
  //       also drawn as slim cylinders.
  //   */
  //   virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
  //                     bool withVisual = true, double visualSize = 0.1,
  //                     bool ignoreColl = true);

  //   virtual void update();

  //   virtual void addForce1(double t);
  //   virtual void addForce2(double t);
  //   virtual double getPosition1() const;
  //   virtual double getPosition2() const;
  //   virtual double getPosition1Rate() const;
  //   virtual double getPosition2Rate() const;

  //   virtual void setParam(int parameter, double value);
  //   virtual double getParam(int parameter) const;

  // protected:
  //   OSGPrimitive* visual1;
  //   OSGPrimitive* visual2;
  //   int relative;
  // };

  /***************************************************************************/



}

#endif

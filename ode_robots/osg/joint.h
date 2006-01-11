/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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
 ***************************************************************************
 *                                                                         *
 *  Joint wrapper to ba able to draw joints and abstract from ode details  *
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.8  2006-01-11 14:11:06  fhesse
 *   moved anchor up into Joint and introduced getAnchor()
 *
 *   Revision 1.1.2.7  2005/12/21 15:39:03  martius
 *   OneAxisJoint and TwoAxisJoint as superclasses
 *
 *   Revision 1.1.2.6  2005/12/19 16:34:18  martius
 *   added Ball and Universal joint
 *
 *   Revision 1.1.2.5  2005/12/15 17:03:42  martius
 *   cameramanupulator setPose is working
 *   joints have setter and getter parameters
 *   Primitives are not longer inherited from OSGPrimitive, moreover
 *   they aggregate them
 *
 *   Revision 1.1.2.4  2005/12/14 15:36:45  martius
 *   joints are visible now
 *
 *   Revision 1.1.2.3  2005/12/13 18:11:13  martius
 *   transform primitive added, some joints stuff done, forward declaration
 *
 *   Revision 1.1.2.2  2005/12/12 23:40:22  martius
 *   hinge2joint started
 *
 *   Revision 1.1.2.1  2005/12/12 22:25:16  martius
 *   added joint
 *
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __JOINT_H
#define __JOINT_H

#include "primitive.h"
#include "osgforwarddecl.h"

namespace lpzrobots {

  /***************************************************************************/

  class Joint {
  public: 
    Joint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor) 
      : joint(0), part1(part1), part2(part2), anchor(anchor) {}
    virtual ~Joint();
    /** initialises (and creates) the joint. If visual is true then the joints is
	also drawn. visualSize is the size of the visual representation.
	(To be overloaded)
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		      bool withVisual = true, double visualSize = 0.2) = 0;
    
    /// should syncronise the Ode stuff and the OSG notes (if any)
    virtual void update() = 0;
    /// sets the ODE joint parameter (see ODE manual)
    virtual void setParam(int parameter, double value) = 0;
    /// return the ODE joint parameter (see ODE manual)
    virtual double getParam(int parameter) = 0;
    
    dJointID getJoint() const  { return joint; }
    const Primitive* getPart1() const { return part1; }
    const Primitive* getPart2() const { return part2; } 
    const osg::Vec3 getAnchor() const { return anchor; }
    
    static osg::Matrix anchorAxisPose(const osg::Vec3& anchor, const osg::Vec3& axis);
  protected:
    dJointID joint;
    Primitive* part1;
    Primitive* part2;    
    osg::Vec3 anchor;
  };

  class OneAxisJoint : public Joint {
  public:
    OneAxisJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor, const osg::Vec3 axis1) 
      : Joint(part1, part2, anchor), axis1(axis1) {}
    virtual osg::Vec3 getAxis1() const { return axis1; };
    
    virtual double getPosition1() = 0;
    virtual double getPosition1Rate() = 0;
        
  protected:
    osg::Vec3 axis1;
  };

  class TwoAxisJoint : public OneAxisJoint {
  public:
    TwoAxisJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor, const osg::Vec3 axis1, 
		 const osg::Vec3 axis2 ) 
      : OneAxisJoint(part1, part2, axis1, anchor), axis2(axis2) {}
    virtual osg::Vec3 getAxis2() const { return axis2; };

    virtual double getPosition2() = 0;
    virtual double getPosition2Rate() = 0;

  protected:
    osg::Vec3 axis2;
  };

  /***************************************************************************/

  class HingeJoint : public OneAxisJoint {
  public:
    HingeJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor, 
		const osg::Vec3& axis1);

    virtual ~HingeJoint();

    /** initialises (and creates) the joint. If visual is true then the axis of the joints is
	also drawn as a slim cylinder. visualSize is the length of the cylinder.
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		      bool withVisual = true, double visualSize = 0.2);
    
    virtual void update();    

    virtual void addTorque(double t);
    virtual double getPosition1();
    virtual double getPosition1Rate();
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter);
    
  protected:
    OSGPrimitive* visual;
  };

  /***************************************************************************/
  
  class Hinge2Joint : public TwoAxisJoint {
  public:
    Hinge2Joint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor, 
		const osg::Vec3& axis1, const osg::Vec3& axis2);

    virtual ~Hinge2Joint();

    /** initialises (and creates) the joint. If visual is true then axis2 of the joints is
	also drawn as a slim cylinder. visualSize is the length of the cylinder.
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		      bool withVisual = true, double visualSize = 0.2);
    
    virtual void update();    

    /// adds torques to axis 1 and 2
    virtual void addTorques(double t1, double t2);
    virtual double getPosition1();
    virtual double getPosition2(); /// This is not supported by the joint!
    virtual double getPosition1Rate();
    virtual double getPosition2Rate();
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter);
    
  protected:
    OSGPrimitive* visual;
  };

  /***************************************************************************/

  class UniversalJoint : public TwoAxisJoint {
  public:
    UniversalJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor, 
		const osg::Vec3& axis1, const osg::Vec3& axis2);

    virtual ~UniversalJoint();

    /** initialises (and creates) the joint. If visual is true then axix1 and axis2 of the joints is
	also drawn as a slim cylinder. visualSize is the length of the cylinder.
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		      bool withVisual = true, double visualSize = 0.2);
    
    virtual void update();    

    /// adds torques to axis 1 and 2
    virtual void addTorques(double t1, double t2);
    virtual double getPosition1();
    virtual double getPosition2();
    virtual double getPosition1Rate();
    virtual double getPosition2Rate();

    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter);
    
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
		      bool withVisual = true, double visualSize = 0.2);
    
    virtual void update();    

    // Ball and Socket has no parameter
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter);
    
  protected:
    OSGPrimitive* visual;
  };

  /***************************************************************************/

  class SliderJoint : public OneAxisJoint {
  public:
    SliderJoint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor, 
		const osg::Vec3& axis1);

    virtual ~SliderJoint();

    /** initialises (and creates) the joint. If visual is true then the axis of the joints is
	also drawn as a slim cylinder. VisualSize is added to the lenght of the slider and is used
	for the length of the cylinder. The radius is visualSize/10
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		      bool withVisual = true, double visualSize = 0.1);
    
    virtual void update();    

    virtual void addForce(double t);
    virtual double getPosition1();
    virtual double getPosition1Rate();
    virtual void setParam(int parameter, double value);
    virtual double getParam(int parameter);
    
  protected:
    OSGPrimitive* visual;
    double visualSize;
    OsgHandle osgHandle;
  };

  /***************************************************************************/



}

#endif

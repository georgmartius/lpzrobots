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
 *   Revision 1.1.2.2  2005-12-12 23:40:22  martius
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

namespace osg{
  class Vec3f;
  typedef Vec3 Vec3f;

  class Matrixd;
  typedef Matrix Matrixd;
}

namespace lpzrobots {

  class Joint {
  public: 
    Joint(Primitive* part1, Primitive* part2) : joint(0), part1(part1), part2(part2) {}
    virtual ~Joint() {}
    /** initialises (and creates) the joint. If visual is true then the joints is
	also drawn. visualSize is the size of the visual representation.
	(To be overloaded)
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		      bool withVisual = true, double visualSize = 0.2) = 0;
    
    /// should syncronise the Ode stuff and the OSG notes (if any)
    virtual void update() = 0;
    
    
    dJointID getJointID() { return joint; }

    static osg::Matrix anchorAxisPose(const osg::Vec3& anchor, const osg::Vec3& axis);
  protected:
    dJointID joint;
    Primitive* part1;
    Primitive* part2;    
  };


  class Hinge2Joint : public Joint {
  public:
    Hinge2Joint(Primitive* part1, Primitive* part2, const osg::Vec3& anchor, 
		const osg::Vec3& axis1, const osg::Vec3& axis2);

    ~Hinge2Joint();

    /** initialises (and creates) the joint. If visual is true then axis2 of the joints is
	also drawn as a slim cylinder. visualSize is the length of the capped cylinder.
    */
    virtual void init(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		      bool withVisual = true, double visualSize = 0.2);
    
    virtual void update();    
    
  protected:
    osg::Vec3 anchor;
    osg::Vec3 axis1;
    osg::Vec3 axis2;
    OSGCapsule* visual;
  };

}

#endif

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
 *                                                                         *
 ***************************************************************************
 *                                                                         *
 *   This file provides basic primitives for ODE and openscenegraph        *
 *                                                                         *
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.2  2006-01-12 14:21:00  martius
 *   drawmode, material
 *
 *   Revision 1.1.2.1  2006/01/10 16:06:30  martius
 *   invisible primitives
 *
 *                                                                 *
 *                                                                         *
 ***************************************************************************/
#ifndef __INVISIBLEPRIMITIVE_H
#define __INVISIBLEPRIMITIVE_H

#include "primitive.h"

namespace lpzrobots {

/**************************************************************************/
class InvisibleBox : public Primitive {
public:

  InvisibleBox(float lengthX, float lengthY, float lengthZ);
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw);
  virtual void update() {}
  virtual OSGPrimitive* getOSGPrimitive() { return 0; }

protected:
  float lengthX;
  float lengthY;
  float lengthZ;  
};


/**************************************************************************/
class InvisibleSphere : public Primitive {
public:
  InvisibleSphere(float radius);
  virtual void init(const OdeHandle& odeHandle, double mass, 
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw);

  virtual void update() {}
  virtual OSGPrimitive* getOSGPrimitive() { return 0; }

protected:
  float radius;
};

/**************************************************************************/
class InvisibleCapsule : public Primitive {
public:
  InvisibleCapsule(float radius, float height);
  virtual void init(const OdeHandle& odeHandle, double mass,
		    const OsgHandle& osgHandle,
		    char mode = Body | Geom | Draw);

  virtual void update() {}
  virtual OSGPrimitive* getOSGPrimitive() { return osgcapsule; }

protected:
  OSGCapsule* osgcapsule;
  float radius;
  float height;
};

}
#endif


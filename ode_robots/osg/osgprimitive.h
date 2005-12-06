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
 *   This file provides basic primitives for openscenegraph usage.         *
 *                                                                         *
 *                                                                         *
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.1  2005-12-06 10:13:24  martius
 *   openscenegraph integration started
 *
 *                                                                 *
 *                                                                         *
 ***************************************************************************/
#ifndef __OSGPRIMITIVE_H
#define __OSGPRIMITIVE_H

#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>

#include "osghandle.h"

namespace lpzrobots {

/**************************************************************************/
class OSGPrimitive {
public:
  OSGPrimitive () {}
  virtual ~OSGPrimitive () {}
  virtual void init(const OsgHandle& osgHandle) = 0;
  virtual void setPose( const osg::Matrix& m4x4 );
  virtual osg::Group* getGroup();

protected:
  osg::ref_ptr<osg::Geode> geode;
  osg::ref_ptr<osg::MatrixTransform> transform;  
};


/**************************************************************************/
class OSGPlane : public OSGPrimitive {
public:
  OSGPlane();

  virtual void init(const OsgHandle& osgHandle);
};


/**************************************************************************/
class OSGBox : public OSGPrimitive {
public:
  OSGBox(float lengthX, float lengthY, float lengthZ);

  virtual void init(const OsgHandle& osgHandle);

protected:
  float lengthX;
  float lengthY;
  float lengthZ;  
};


/**************************************************************************/
class OSGSphere : public OSGPrimitive {
public:
  OSGSphere(float radius);

  virtual void init(const OsgHandle& osgHandle);

protected:
  float radius;  
};

/**************************************************************************/
class OSGCapsule : public OSGPrimitive {
public:
  OSGCapsule(float radius, float height);

  virtual void init(const OsgHandle& osgHandle);

protected:
  float radius;  
  float height;
};

}

#endif


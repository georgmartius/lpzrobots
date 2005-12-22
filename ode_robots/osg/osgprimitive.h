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
 *   Revision 1.1.2.7  2005-12-22 14:14:12  martius
 *   quality level
 *
 *   Revision 1.1.2.6  2005/12/15 17:03:43  martius
 *   cameramanupulator setPose is working
 *   joints have setter and getter parameters
 *   Primitives are not longer inherited from OSGPrimitive, moreover
 *   they aggregate them
 *
 *   Revision 1.1.2.5  2005/12/14 15:36:45  martius
 *   joints are visible now
 *
 *   Revision 1.1.2.4  2005/12/13 18:11:13  martius
 *   transform primitive added, some joints stuff done, forward declaration
 *
 *   Revision 1.1.2.3  2005/12/11 23:35:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.2  2005/12/09 16:54:16  martius
 *   camera is woring now
 *
 *   Revision 1.1.2.1  2005/12/06 10:13:24  martius
 *   openscenegraph integration started
 *
 *                                                                 *
 *                                                                         *
 ***************************************************************************/
#ifndef __OSGPRIMITIVE_H
#define __OSGPRIMITIVE_H

#include <string>
#include <osg/ref_ptr>
#include "osgforwarddecl.h"
#include "osghandle.h"

namespace lpzrobots {


  /**************************************************************************/
  class OSGPrimitive {
  public:

    typedef enum Quality {Low, Middle, High};

    OSGPrimitive ();
    virtual ~OSGPrimitive ();
    // this function should be overloaded
    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle) = 0;
    virtual void setMatrix( const osg::Matrix& m4x4 );
    virtual osg::Group* getGroup();
    virtual void setTexture(const std::string& filename);
    virtual void setColor(const Color& color);
    /// returns a osg transformation object;
    virtual osg::Transform* getTransform();

  protected:
    osg::ref_ptr<osg::Geode> geode;
    osg::ref_ptr<osg::MatrixTransform> transform;  
    osg::ref_ptr<osg::ShapeDrawable> shape;
  };

  /**************************************************************************/
  class OSGDummy : public OSGPrimitive {
  public:
    OSGDummy();

    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);
    virtual void setMatrix( const osg::Matrix& m4x4 );
    virtual osg::Group* getGroup();
    virtual void setTexture(const std::string& filename);
    virtual void setColor(const Color& color);
    /// returns a osg transformation object;
    virtual osg::Transform* getTransform();  
  };


  /**************************************************************************/
  class OSGPlane : public OSGPrimitive {
  public:
    OSGPlane();

    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);
  };


  /**************************************************************************/
  class OSGBox : public OSGPrimitive {
  public:
    OSGBox(float lengthX, float lengthY, float lengthZ);

    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);

    float getLengthX() { return lengthX; }
    float getLengthY() { return lengthY; }
    float getLengthZ() { return lengthZ; }
  
  protected:
    float lengthX;
    float lengthY;
    float lengthZ;  
  };


  /**************************************************************************/
  class OSGSphere : public OSGPrimitive {
  public:
    OSGSphere(float radius);

    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);

    float getRadius() { return radius; }
  protected:
    float radius;  
  };

  /**************************************************************************/
  class OSGCapsule : public OSGPrimitive {
  public:
    OSGCapsule(float radius, float height);

    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);

    float getRadius() { return radius; }
    float getHeight() { return height; }
  protected:
    float radius;
    float height;
  };

  /**************************************************************************/
  class OSGCylinder : public OSGPrimitive {
  public:
    OSGCylinder(float radius, float height);

    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);

    float getRadius() { return radius; }
    float getHeight() { return height; }
  protected:
    float radius;  
    float height;
  };

}

#endif


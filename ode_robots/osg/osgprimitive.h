
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
#ifndef __OSGPRIMITIVE_H
#define __OSGPRIMITIVE_H

#include <string>
#include <vector>
#include <osg/ref_ptr>
#include "osgforwarddecl.h"
#include "osghandle.h"
#include <osgDB/ReadFile>
#include <osgText/Text>

namespace lpzrobots {

  /**
     holds texture file and repeat information.

   */
  class TextureDescr {
  public:
    TextureDescr(){}
    /**
       If repeatOnX is negativ then it is used as a unit length for the texture.
    */
    TextureDescr(const std::string& filename, double repeatOnR, double repeatOnS)
      : filename(filename), repeatOnR(repeatOnR), repeatOnS(repeatOnS)
    {
    }
    std::string filename;
    double repeatOnR;
    double repeatOnS;
  };

  /**
     Interface class for graphic primitives like spheres, boxes, and meshes,
     which can be drawn by OSG. The idea is to hide all the details of the OSG
     implementation.
  */
  class OSGPrimitive {
  public:
    /* typedef */ enum Quality {Low, Middle, High};

    OSGPrimitive ();
    virtual ~OSGPrimitive ();
    /** Initialisation of the primitive. Must in order to place the object into the scene.
        This function should be overloaded */
    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle) = 0;
    /// Sets the transformation matrix of this object (position and orientation)
    virtual void setMatrix( const osg::Matrix& m4x4 );
    /// returns the group object which is the root of all subcomponents of this primitive
    virtual osg::Group* getGroup();
    /// assigns a texture to the primitive
    virtual void setTexture(const std::string& filename);
    /// assigns a texture to the primitive, you can choose how often to repeat
    virtual void setTexture(const TextureDescr& texture);
    /// assigns a texture to the x-th surface of the primitive, you can choose how often to repeat
    virtual void setTexture(int surface, const TextureDescr& texture);
    /// assign a set of texture to the surfaces of the primitive
    virtual void setTextures(const std::vector<TextureDescr>& textures);
    /// returns the list of textures
    virtual std::vector<TextureDescr> getTextures() const;
    /// sets the color for painting this primitive
    virtual void setColor(const Color& color);
    /// sets the color using the colorschema of osgHandle
    virtual void setColor(const std::string& color);
    /// returns the current color
    virtual Color getColor();

    /// returns a osg transformation object;
    virtual osg::Transform* getTransform();
    /// returns the osgHandle object
    virtual const OsgHandle& getOsgHandle();

  protected:
    /// this actually sets the textures
    virtual void applyTextures();

    osg::ref_ptr<osg::Geode> geode;
    osg::ref_ptr<osg::MatrixTransform> transform;
    osg::ref_ptr<osg::ShapeDrawable> shape;

    std::vector<TextureDescr > textures;

    OsgHandle osgHandle;
  };

  /**
     A dummy graphical object, which has no representation in the graphical world.
  */
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


  /**
     Graphical plane (represented as a large thin box, because OSG does not draw planes)
  */
  class OSGPlane : public OSGPrimitive {
  public:
    OSGPlane();

    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);
  };


  /**
     Graphical box
  */
  class OSGBox : public OSGPrimitive {
  public:
    OSGBox(float lengthX, float lengthY, float lengthZ);
    OSGBox(osg::Vec3 dim);

    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);

    virtual osg::Vec3 getDim();
    virtual void setDim(osg::Vec3);

  protected:
    osg::Vec3 dim;
    osg::Box* box;
  };

 /**
     Graphical box with Textures
  */
  class OSGBoxTex : public OSGPrimitive {
  public:
    OSGBoxTex(float lengthX, float lengthY, float lengthZ);
    OSGBoxTex(osg::Vec3 dim);

    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);

    virtual osg::Vec3 getDim() const { return dim;}
    virtual void setDim(const osg::Vec3& _dim) { dim = _dim;}

    virtual void setColor(const Color& color);

  protected:
    /// this actually sets the textures, overwritten
    virtual void applyTextures();

    osg::Vec3 dim;
    // we use one geode for each face of the box for the texture handling
    osg::ref_ptr<osg::Geode> faces[6];
  };


  /**
     Graphical sphere
  */
  class OSGSphere : public OSGPrimitive {
  public:
    OSGSphere(float radius);

    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);

    float getRadius() { return radius; }
  protected:
    float radius;
  };

  /**
     Graphical capsule (a cylinder with round ends)
  */
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


  /**
     Graphical cylinder
  */
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

  class OSGLine : public OSGPrimitive {
  public:
    // the list of points is considered pairwise, start-end points of each line segment
    OSGLine(const std::list<osg::Vec3>& points);

    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);

    virtual void applyTextures(){}

    virtual void setColor(const Color& color);

    // use the new points
    virtual void setPoints(const std::list<osg::Vec3>& points);

  protected:
    std::list<osg::Vec3> points;
    osg::Geometry *geometry;

    virtual void updatePoints();

  };

  /**
     Graphical Mesh or arbitrary OSG model.
  */
  class OSGMesh : public OSGPrimitive {
  public:
    /**
       Constuctor
       @param filename filename of the model file (search path is osg data path)
       @param scale scale factor used for scaling the model
       @param options for model reader
     */
    OSGMesh(const std::string& filename, float scale = 1, const osgDB::ReaderWriter::Options* options = 0);
    ~OSGMesh();
    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);
    /**
     * Same as init, but the mesh file is not loaded and therefore not displayed.
     * This method ensures that the transform is correctly initialised.
     * @param osgHandle
     */
    virtual void virtualInit(const OsgHandle& osgHandle);
    virtual float getRadius();
    float getScale() { return scale; }

  protected:
    std::string filename;
    float scale;
    const osgDB::ReaderWriter::Options* options;
    osg::ref_ptr<osg::Node> mesh;
    osg::ref_ptr<osg::MatrixTransform> scaletrans;

    virtual void internInit(const OsgHandle& osgHandle, bool loadAndDisplayMesh, Quality quality = Middle);

  };

  /**
     Text to be displayed on the hud
  */
  class OSGText : public OSGPrimitive {
  public:
    OSGText(const std::string& text, int fontsize = 12,
            osgText::Text::AlignmentType align = osgText::Text::LEFT_BASE_LINE);

    virtual ~OSGText();

    virtual void init(const OsgHandle& osgHandle, Quality quality = Middle);
    virtual void setMatrix( const osg::Matrix& m4x4 );
    virtual osg::Group* getGroup();
    virtual void setColor(const Color& color);
    /// returns a osg transformation object;
    virtual osg::Transform* getTransform();
  private:
    osgText::Text* osgText;
  };


}

#endif


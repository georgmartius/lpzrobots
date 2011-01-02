 
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
 *   Revision 1.12  2011-01-02 23:09:52  martius
 *   texture handling of boxes changed
 *   playground walls changed
 *
 *   Revision 1.11  2010/09/17 10:07:45  martius
 *   changing size requires invalidation of display list
 *
 *   Revision 1.10  2010/07/02 05:45:06  martius
 *   comments improved
 *
 *   Revision 1.9  2010/03/16 15:47:46  martius
 *   osgHandle has now substructures osgConfig and osgScene
 *    that minimized amount of redundant data (this causes a lot of changes)
 *   Scenegraph is slightly changed. There is a world and a world_noshadow now.
 *    Main idea is to have a world without shadow all the time avaiable for the
 *    Robot cameras (since they do not see the right shadow for some reason)
 *   tidied up old files
 *
 *   Revision 1.8  2010/03/07 22:45:35  guettler
 *   - OSGMesh supports now virtual initialisation (necessary for Meshes not visible)
 *
 *   Revision 1.7  2009/07/30 11:34:15  guettler
 *   added check if noGraphics in OsgHandle is set
 *
 *   Revision 1.6  2009/04/26 10:28:49  martius
 *   added setColor for OsgBoxTex
 *
 *   Revision 1.5  2009/01/20 17:29:10  martius
 *   changed texture handling. In principle it is possible to set multiple textures
 *   per osgPrimitive.
 *   New osgboxtex started that supports custom textures.
 *
 *   Revision 1.4  2008/05/07 16:45:51  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.3  2007/08/23 14:52:26  martius
 *   box is resizeable
 *
 *   Revision 1.2  2006/07/14 12:23:35  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.13  2006/07/14 11:23:38  martius
 *   revert to older revision of robot3
 *
 *   Revision 1.1.2.11  2006/05/24 12:23:10  robot3
 *   -passive_mesh works now (simple bound_version)
 *   -Primitive Mesh now exists (simple bound_version)
 *
 *   Revision 1.1.2.10  2006/05/18 07:39:41  robot3
 *   -setTexture(string& filename,bool repeatOnX, bool repeatOnY) added
 *    note that this does not work yet (the bool parameter have no effect)
 *
 *   Revision 1.1.2.9  2006/04/04 14:13:24  fhesse
 *   documentation improved
 *
 *   Revision 1.1.2.8  2006/03/29 15:06:40  martius
 *   OSGMesh
 *
 *   Revision 1.1.2.7  2005/12/22 14:14:12  martius
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
#include <vector>
#include <osg/ref_ptr>
#include "osgforwarddecl.h"
#include "osghandle.h"
#include <osgDB/ReadFile>

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

}

#endif


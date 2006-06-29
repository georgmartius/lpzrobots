/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   This file provides basic primitives for openscenegraph usage          *
 *   with ODE.                                                             *
 *                                                                         *
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.18  2006-06-29 16:35:32  robot3
 *   -Mesh code optimized
 *   -includes cleared up, more using forward declarations
 *    (sometimes additionally #include "osgprimitive.h" is needed)
 *
 *   Revision 1.1.2.17  2006/06/23 09:04:48  robot3
 *   added #include <assert.h>
 *
 *   Revision 1.1.2.16  2006/05/28 22:14:56  martius
 *   heightfield included
 *
 *   Revision 1.1.2.15  2006/05/24 12:23:10  robot3
 *   -passive_mesh works now (simple bound_version)
 *   -Primitive Mesh now exists (simple bound_version)
 *
 *   Revision 1.1.2.14  2006/05/18 12:28:39  robot3
 *   added a dummy texture to every osgprimitive for correct shadowing
 *
 *   Revision 1.1.2.13  2006/05/18 10:33:39  robot3
 *   fixed error which outshined other objects
 *
 *   Revision 1.1.2.12  2006/05/18 07:16:36  robot3
 *   -setTexture(string& filename,bool repeatOnX, bool repeatOnY) added
 *    note that this does not work yet (the bool parameter have no effect)
 *
 *   Revision 1.1.2.11  2006/03/29 15:06:40  martius
 *   OSGMesh
 *
 *   Revision 1.1.2.10  2006/01/12 14:21:00  martius
 *   drawmode, material
 *
 *   Revision 1.1.2.9  2005/12/22 14:14:21  martius
 *   quality level
 *
 *   Revision 1.1.2.8  2005/12/15 17:03:42  martius
 *   cameramanupulator setPose is working
 *   joints have setter and getter parameters
 *   Primitives are not longer inherited from OSGPrimitive, moreover
 *   they aggregate them
 *
 *   Revision 1.1.2.7  2005/12/14 15:36:45  martius
 *   joints are visible now
 *
 *   Revision 1.1.2.6  2005/12/13 18:11:13  martius
 *   transform primitive added, some joints stuff done, forward declaration
 *
 *   Revision 1.1.2.5  2005/12/12 23:42:53  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.4  2005/12/11 23:35:08  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.3  2005/12/09 16:54:16  martius
 *   camera is woring now
 *
 *   Revision 1.1.2.2  2005/12/06 17:38:15  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.1  2005/12/06 10:13:24  martius
 *   openscenegraph integration started
 *
 *                                                                 *
 *                                                                         *
 ***************************************************************************/

#include <assert.h>

#include <osg/Texture2D>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
//#include <osg/Texture>
//#include <osg/TexGen>
//#include <osg/PolygonOffset>
//#include <osg/Light>
//#include <osg/LightSource>
#include <osg/Material>
#include <osg/TexEnv>


#include "osgprimitive.h"

namespace lpzrobots {

  using namespace osg;
  using namespace osgDB;


  // returns a material with the given color
  ref_ptr<Material> getMaterial (const Color& c, Material::ColorMode mode = Material::DIFFUSE );


  /******************************************************************************/


  OSGPrimitive::OSGPrimitive(){  }

  OSGPrimitive::~OSGPrimitive(){    
    Node::ParentList l = transform->getParents();
    for(Node::ParentList::iterator i = l.begin(); i != l.end(); i++){
      (*i)->removeChild(transform.get());  
    }
  }


  /******************************************************************************/
  void OSGPrimitive::setMatrix(const Matrix& m4x4){
    assert(!transform == false);
    transform->setMatrix(m4x4);
  }

  Group* OSGPrimitive::getGroup() { 
    return transform.get(); 
  }

  Transform* OSGPrimitive::getTransform() { 
    return transform.get(); 
  }

 void OSGPrimitive::setTexture(const std::string& filename){
   setTexture(filename,false,false);
  }

 void OSGPrimitive::setTexture(const std::string& filename, bool repeatOnX, bool repeatOnY){
    osg::Group* grp = getGroup();
    osg::Texture2D* texture = new osg::Texture2D;
    texture->setDataVariance(osg::Object::DYNAMIC); // protect from being optimized away as static state.
    texture->setImage(osgDB::readImageFile(filename));
    ///TODO: needs to be fixed (why does this not work???)
    if (repeatOnX)
      texture->setWrap( Texture2D::WRAP_S, Texture2D::REPEAT );
    if (repeatOnY)
      texture->setWrap( Texture2D::WRAP_T, Texture2D::REPEAT );
    //    texture->setWrap( Texture2D::WRAP_R, Texture2D::REPEAT ); // ???
    osg::StateSet* stateset = grp->getOrCreateStateSet();
    stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
    stateset->setTextureAttribute(0, new TexEnv );

  }


  void OSGPrimitive::setColor(const Color& color){
    if(shape.valid())
      shape->setColor(color);
  }



  /******************************************************************************/
  OSGDummy::OSGDummy(){}
  
  void OSGDummy::init(const OsgHandle& osgHandle, Quality quality){
  }
  
  void OSGDummy::setMatrix( const osg::Matrix& m4x4 ) {
  }
  
  Group* OSGDummy::getGroup() { 
    return 0;
  }

  void OSGDummy::setTexture(const std::string& filename) {
    
  }

  void OSGDummy::setColor(const Color& color) {

  }
  
  Transform* OSGDummy::getTransform() {
    return 0;
  }

  /******************************************************************************/
  OSGPlane::OSGPlane() {
  }

  void OSGPlane::init(const OsgHandle& osgHandle, Quality quality){
    assert(osgHandle.scene);
    geode = new Geode;  
    transform = new MatrixTransform;
    transform->addChild(geode.get());
    osgHandle.scene->addChild(transform.get());
  
    //  shape = new ShapeDrawable(new InfinitePlane(), osgHandle.tesselhints);
    shape = new ShapeDrawable(new Box(Vec3(0.0f, 0.0f, 0.0f), 
				      50, 50, 0.01), osgHandle.tesselhints[quality]); // TODO add larger values here
    shape->setColor(osgHandle.color);
    geode->addDrawable(shape.get());
    if(osgHandle.color.alpha() < 1.0){
      shape->setStateSet(osgHandle.transparentState);
    }else{
      shape->setStateSet(osgHandle.normalState);
    }
    shape->getOrCreateStateSet()->setAttributeAndModes(getMaterial(osgHandle.color).get(), 
						       StateAttribute::ON);
    // set dummy texture
    setTexture("Images/really_white.rgb");
  }


  /******************************************************************************/
  OSGBox::OSGBox(float lengthX, float lengthY, float lengthZ)
    : lengthX(lengthX), lengthY(lengthY), lengthZ(lengthZ) {
  }

  void OSGBox::init(const OsgHandle& osgHandle, Quality quality){
    assert(osgHandle.scene);
    geode = new Geode;  
    transform = new MatrixTransform;
    transform->addChild(geode.get());
    osgHandle.scene->addChild(transform.get());

    shape = new ShapeDrawable(new Box(Vec3(0.0f, 0.0f, 0.0f), 
				      lengthX, lengthY, lengthZ), osgHandle.tesselhints[quality]);
    shape->setColor(osgHandle.color);
    geode->addDrawable(shape.get());
    if(osgHandle.color.alpha() < 1.0){
      shape->setStateSet(osgHandle.transparentState);
    }else{
      shape->setStateSet(osgHandle.normalState);
    }
    shape->getOrCreateStateSet()->setAttributeAndModes(getMaterial(osgHandle.color).get(), 
						       StateAttribute::ON);
    setTexture("Images/really_white.rgb");
  }

  /******************************************************************************/
  OSGSphere::OSGSphere(float radius)
    : radius(radius) {
  }

  void OSGSphere::init(const OsgHandle& osgHandle, Quality quality){
    assert(osgHandle.scene);
    
    geode = new Geode;  
    transform = new MatrixTransform;
    transform->addChild(geode.get());
    osgHandle.scene->addChild(transform.get());

    shape = new ShapeDrawable(new Sphere(Vec3(0.0f, 0.0f, 0.0f), radius), osgHandle.tesselhints[quality]);
    shape->setColor(osgHandle.color);
    geode->addDrawable(shape.get());
    if(osgHandle.color.alpha() < 1.0){
      shape->setStateSet(osgHandle.transparentState);
    }else{
      shape->setStateSet(osgHandle.normalState);
    }
    shape->getOrCreateStateSet()->setAttributeAndModes(getMaterial(osgHandle.color).get(), 
						       StateAttribute::ON);

    setTexture("Images/really_white.rgb");
  }

  /******************************************************************************/
  OSGCapsule::OSGCapsule(float radius, float height)
    : radius(radius), height(height) {
  }

  void OSGCapsule::init(const OsgHandle& osgHandle, Quality quality){
    assert(osgHandle.scene);

    geode = new Geode;  
    transform = new MatrixTransform;
    transform->addChild(geode.get());
    osgHandle.scene->addChild(transform.get());

    shape = new ShapeDrawable(new Capsule(Vec3(0.0f, 0.0f, 0.0f), 
					  radius, height), osgHandle.tesselhints[quality]);
    shape->setColor(osgHandle.color);
    geode->addDrawable(shape.get());
    if(osgHandle.color.alpha() < 1.0){
      shape->setStateSet(osgHandle.transparentState);
    }else{
      shape->setStateSet(osgHandle.normalState);
    }
    shape->getOrCreateStateSet()->setAttributeAndModes(getMaterial(osgHandle.color).get(), 
						       StateAttribute::ON);
    setTexture("Images/really_white.rgb");
  }

  /******************************************************************************/
  OSGCylinder::OSGCylinder(float radius, float height)
    : radius(radius), height(height) {
  }

  void OSGCylinder::init(const OsgHandle& osgHandle, Quality quality){
    assert(osgHandle.scene);

    geode = new Geode;  
    transform = new MatrixTransform;
    transform->addChild(geode.get());
    osgHandle.scene->addChild(transform.get());

    shape = new ShapeDrawable(new Cylinder(Vec3(0.0f, 0.0f, 0.0f), 
					   radius, height), osgHandle.tesselhints[quality]);
    shape->setColor(osgHandle.color);
    geode->addDrawable(shape.get());
    if(osgHandle.color.alpha() < 1.0){
      shape->setStateSet(osgHandle.transparentState);
    }else{
      shape->setStateSet(osgHandle.normalState);
    }
    shape->getOrCreateStateSet()->setAttributeAndModes(getMaterial(osgHandle.color).get(), 
						       StateAttribute::ON);

    setTexture("Images/really_white.rgb");
  }

  /******************************************************************************/
  OSGMesh::OSGMesh(const std::string& filename, float scale, 
		   const ReaderWriter::Options* options)
    : filename(filename), scale(scale), options(options) 
  {
  }

  OSGMesh::~OSGMesh(){
  }

  float OSGMesh::getRadius() {
    return getGroup()->getBound().radius(); 
  }

  void OSGMesh::init(const OsgHandle& osgHandle, Quality quality){
    assert(osgHandle.scene);
    transform = new MatrixTransform;        
    osgHandle.scene->addChild(transform.get());
    scaletrans = new MatrixTransform;    
    scaletrans->setMatrix(osg::Matrix::scale(scale,scale,scale));
    transform->addChild(scaletrans.get());
    mesh  = osgDB::readNodeFile(filename, options);
    scaletrans->addChild(mesh.get());
    
//     if(osgHandle.color.alpha() < 1.0){
//       shape->setStateSet(osgHandle.transparentState);
//     }else{
//       shape->setStateSet(osgHandle.normalState);
//     }
//     shape->getOrCreateStateSet()->setAttributeAndModes(getMaterial(osgHandle.color).get(), 
// 						       StateAttribute::ON);

//    setTexture("Images/really_white.rgb");
//    setColor(osgHandle.color); // doesn't work with Mesh(es)

    /***********************************************************************************************
     * the following code is for setTexture() for a Mesh, since the normal setTexture() doesn't    *
     * work. This works with the cow.osg example, but NOT with the dumptruck.osg example (why?)    *
     **********************************************************************************************/
    /*
    osg::Geode* geode = dynamic_cast<osg::Geode*> (mesh.get()->asGroup()->getChild(0));
    osg::Drawable* geom = geode->getDrawable(0);
    osg::Texture2D* texture = new osg::Texture2D;
    texture->setDataVariance(osg::Object::DYNAMIC); // protect from being optimized away as static state.
    texture->setImage(osgDB::readImageFile("Images/whitemetal_farbig.rgb"));
    ///TODO: needs to be fixed (why does this not work???)
//     if (repeatOnX)
       texture->setWrap( Texture2D::WRAP_S, Texture2D::REPEAT );
//     if (repeatOnY)
       texture->setWrap( Texture2D::WRAP_T, Texture2D::REPEAT );
    //    texture->setWrap( Texture2D::WRAP_R, Texture2D::REPEAT ); // ???
    osg::StateSet* stateset = geom->getOrCreateStateSet();
    stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
    stateset->setTextureAttribute(0, new TexEnv );
    */
  }


  /********************* HELPER ************************************************/

  // returns a material with the given color
  ref_ptr<Material> getMaterial (const Color& c, Material::ColorMode mode) {
    ref_ptr<Material> m = new Material ();
    m->setColorMode(mode);
    //    Color amb (c*0.3);
    //    amb.alpha()=c.alpha();
    Color dif(c*0.7);
    dif.alpha()=c.alpha();
    Color spec(c*0.2);
    spec.alpha()=c.alpha();
    //    m->setAmbient(Material::FRONT_AND_BACK, amb);
    m->setDiffuse(Material::FRONT_AND_BACK, dif);
    m->setSpecular(Material::FRONT_AND_BACK, spec);
    m->setShininess(Material::FRONT_AND_BACK, 5.0f);
    //  m->setShininess(Material::FRONT_AND_BACK, 25.0f);
    return m;
  }


}

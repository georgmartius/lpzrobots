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
 *   Revision 1.25  2010-10-14 15:42:40  martius
 *   checking for the config
 *
 *   Revision 1.24  2010/09/24 09:00:04  martius
 *   fixed lighing bug! Juhu. The StateSet was shared such that the material
 *    was shared. In this way the last material was used for all primitives
 *
 *   Revision 1.23  2010/09/17 10:07:45  martius
 *   changing size requires invalidation of display list
 *
 *   Revision 1.22  2010/03/29 16:28:21  martius
 *   abstract ground rembers groundsubstance
 *   comments and typos
 *   osgprimitive uses white for empty texture
 *
 *   Revision 1.21  2010/03/16 15:47:46  martius
 *   osgHandle has now substructures osgConfig and osgScene
 *    that minimized amount of redundant data (this causes a lot of changes)
 *   Scenegraph is slightly changed. There is a world and a world_noshadow now.
 *    Main idea is to have a world without shadow all the time avaiable for the
 *    Robot cameras (since they do not see the right shadow for some reason)
 *   tidied up old files
 *
 *   Revision 1.20  2010/03/15 09:06:07  guettler
 *   improved lightning for meshes
 *
 *   Revision 1.18  2010/03/12 13:57:50  guettler
 *   support for transparent textures in meshes
 *
 *   Revision 1.17  2010/03/07 22:45:19  guettler
 *   - OSGMesh supports now virtual initialisation (necessary for Meshes not visible)
 *   - Bugfix: osgHandle was not checked if valid (occurs when Primitive is not initialised)
 *
 *   Revision 1.16  2009/08/10 14:56:24  der
 *   transform.get() to compile on older gcc/OSG versions
 *
 *   Revision 1.15  2009/08/07 13:25:55  martius
 *   destructor does not segfault if oject is uninitialized
 *
 *   Revision 1.14  2009/07/30 11:34:15  guettler
 *   added check if noGraphics in OsgHandle is set
 *
 *   Revision 1.13  2009/04/26 10:28:16  martius
 *   added setColor for OsgBoxTex. This should be implemented and only gives a error message now.
 *
 *   Revision 1.12  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.11  2009/02/04 09:36:31  martius
 *   osgboxtex theoretically correct, pratically does not work yet
 *
 *   Revision 1.10  2009/01/20 17:29:10  martius
 *   changed texture handling. In principle it is possible to set multiple textures
 *   per osgPrimitive.
 *   New osgboxtex started that supports custom textures.
 *
 *   Revision 1.9  2008/05/07 16:45:51  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.8  2007/08/23 14:52:40  martius
 *   box is resizeable
 *
 *   Revision 1.7  2007/07/31 08:21:33  martius
 *   OSGMesh does not need GlobalData
 *   drawBoundings moved to OsgHandle
 *
 *   Revision 1.6  2006/09/21 22:09:40  martius
 *   material has no spec because everthing shines then liek the coloured objects
 *
 *   Revision 1.5  2006/09/20 12:55:44  martius
 *   Light
 *
 *   Revision 1.4  2006/08/11 15:42:45  martius
 *   larger plane
 *
 *   Revision 1.3  2006/08/04 15:05:43  martius
 *   documentation
 *
 *   Revision 1.2  2006/07/14 12:23:35  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.20  2006/07/14 11:23:38  martius
 *   revert to older revision of robot3
 *
 *   Revision 1.1.2.18  2006/06/29 16:35:32  robot3
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
#include <osg/Geometry>
//#include <osg/Texture>
//#include <osg/TexGen>
//#include <osg/PolygonOffset>
//#include <osg/Light>
//#include <osg/LightSource>
#include <osg/Material>
#include <osg/TexEnv>
#include <osg/AlphaFunc>



#include "osgprimitive.h"

namespace lpzrobots {

  using namespace osg;
  using namespace osgDB;


  // returns a material with the given color
  ref_ptr<Material> getMaterial (const Color& c, Material::ColorMode mode = Material::DIFFUSE );

  osg::Geode* createRectangle(const OsgHandle&,
			      const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3,
			      double repeatOnR, double repeatOnS);

  // attached a texture to a geode
  void addTexture(Geode* geode, const TextureDescr& tex);

  Geode* test();

  /******************************************************************************/


  OSGPrimitive::OSGPrimitive() {
    setTexture("Images/really_white.rgb",1,1);
  }

  OSGPrimitive::~OSGPrimitive(){
    if(transform.get()){
      Node::ParentList l = transform->getParents();
      for(Node::ParentList::iterator i = l.begin(); i != l.end(); i++){
	(*i)->removeChild(transform.get());  
      }
    }
    textures.clear();
  }


  /******************************************************************************/
  void OSGPrimitive::setMatrix(const osg::Matrix& m4x4){
    assert(!transform == false);
    transform->setMatrix(m4x4);
  }

  Group* OSGPrimitive::getGroup() { 
    return transform.get(); 
  }

  Transform* OSGPrimitive::getTransform() { 
    return transform.get(); 
  }

  const OsgHandle& OSGPrimitive::getOsgHandle() { 
    return osgHandle; 
  }


 void OSGPrimitive::setTexture(const std::string& filename){
   setTexture(filename,1,1);
  }

 void OSGPrimitive::setTexture(const std::string& filename, 
                               double repeatOnR, double repeatOnS){
   setTexture(0,filename,repeatOnR,repeatOnS);
  }

  void OSGPrimitive::setTexture(int surface, const std::string& filename, 
				double repeatOnR, double repeatOnS){
    if((signed)textures.size()<=surface){
      textures.resize(surface+1);
    }
    if(!filename.empty())
      textures[surface]=TextureDescr(filename, repeatOnR, repeatOnS);    
    else 
      textures[surface]=TextureDescr("Images/really_white.rgb", 1, 1);    
    if(transform.valid()){ // is the object already initialized?
      applyTextures();
    }
  }

  
  std::vector<TextureDescr> OSGPrimitive::getTextures(){
    return textures;
  }
  
  void OSGPrimitive::applyTextures(){
    if (!osgHandle.cfg || osgHandle.cfg->noGraphics)
      return;
    // this is only the default implementation. For Non-ShapeDrawables this most prob. be overloaded
    if(textures.size() > 0){
      osg::Group* grp = getGroup();
      osg::Texture2D* texture = new osg::Texture2D;
      texture->setDataVariance(osg::Object::DYNAMIC); // protect from being optimized away as static state.
      texture->setImage(osgDB::readImageFile(textures[0].filename));
      // The wrapping does not work in general
      //  because the texture coordinates only go from 0 to 1 in the shapedrawables
      osg::StateSet* stateset = grp->getOrCreateStateSet();
      // maybe we don't need the StateAttr. ?
      stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
      stateset->setTextureAttribute(0, new TexEnv );    
    }
  }

  void OSGPrimitive::setColor(const Color& color){
    if (!osgHandle.cfg || osgHandle.cfg->noGraphics)
      return;
    if(shape.valid())
      shape->setColor(color);
  }


  /******************************************************************************/
  OSGDummy::OSGDummy(){}
  
  void OSGDummy::init(const OsgHandle& osgHandle, Quality quality){
    this->osgHandle=osgHandle;
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
    this->osgHandle=osgHandle;
    assert(osgHandle.parent || osgHandle.cfg->noGraphics);
    transform = new MatrixTransform;
    if (osgHandle.cfg->noGraphics)
      return;
    geode = new Geode;  
    transform->addChild(geode.get());
    osgHandle.parent->addChild(transform.get());
  
    //  shape = new ShapeDrawable(new InfinitePlane(), osgHandle.cfg->tesselhints);
    shape = new ShapeDrawable(new Box(Vec3(0.0f, 0.0f, 0.0f), 
				      100, 100, 0.01), osgHandle.cfg->tesselhints[quality]);
    shape->setColor(osgHandle.color);
    geode->addDrawable(shape.get());
    if(osgHandle.color.alpha() < 1.0){
      shape->setStateSet(new StateSet(*osgHandle.cfg->transparentState));
    }else{
      shape->setStateSet(new StateSet(*osgHandle.cfg->normalState));
    }
    shape->getOrCreateStateSet()->setAttributeAndModes(getMaterial(osgHandle.color).get(), 
						       StateAttribute::ON);
    applyTextures();
  }


  /******************************************************************************/
  OSGBox::OSGBox(float lengthX, float lengthY, float lengthZ)
    : dim(lengthX, lengthY, lengthZ) {
  }
  OSGBox::OSGBox(Vec3 dim)
    : dim(dim){
  }

  void OSGBox::init(const OsgHandle& osgHandle, Quality quality){
    this->osgHandle=osgHandle;
    assert(osgHandle.parent || osgHandle.cfg->noGraphics);
    transform = new MatrixTransform;
    if (osgHandle.cfg->noGraphics)
      return;
    geode = new Geode;  
    transform->addChild(geode.get());
    osgHandle.parent->addChild(transform.get());

    box = new Box(Vec3(0.0f, 0.0f, 0.0f), 
		  dim.x(), dim.y(), dim.z());
    shape = new ShapeDrawable(box, osgHandle.cfg->tesselhints[quality]);
    shape->setColor(osgHandle.color);
    geode->addDrawable(shape.get());
    if(osgHandle.color.alpha() < 1.0){
      shape->setStateSet(new StateSet(*osgHandle.cfg->transparentState));
    }else{
      shape->setStateSet(new StateSet(*osgHandle.cfg->normalState));
    }

    shape->getOrCreateStateSet()->setAttributeAndModes(getMaterial(osgHandle.color).get(), 
						       StateAttribute::ON);
    applyTextures();
  }

  Vec3 OSGBox::getDim(){
    return dim;  
  }
  void OSGBox::setDim(Vec3 d){
    dim = d;
    if (!osgHandle.cfg || osgHandle.cfg->noGraphics)
      return;
    box->setHalfLengths(d/2.0);
    shape->dirtyDisplayList(); // this is important, otherwise we don't see the changes.
  }

  /******************************************************************************/
  OSGBoxTex::OSGBoxTex(float lengthX, float lengthY, float lengthZ)
    : dim(lengthX, lengthY, lengthZ) {
  }
  OSGBoxTex::OSGBoxTex(Vec3 dim)
    : dim(dim){
  }

  void OSGBoxTex::init(const OsgHandle& osgHandle, Quality quality){
    this->osgHandle=osgHandle;
    assert(osgHandle.parent || osgHandle.cfg->noGraphics);
    transform = new MatrixTransform;
    if (!osgHandle.cfg || osgHandle.cfg->noGraphics)
      return;
    osgHandle.parent->addChild(transform.get());

    if(osgHandle.color.alpha() < 1.0){
      transform->setStateSet(new StateSet(*osgHandle.cfg->transparentState));
    }else{
      transform->setStateSet(new StateSet(*osgHandle.cfg->normalState));
    }
    
    transform->getOrCreateStateSet()->setAttributeAndModes(getMaterial(osgHandle.color).get(), 
							   StateAttribute::ON);

    Vec3 half = dim*(-0.5);    
    Vec3 dx(dim.x(),0.0f,0.0f);
    Vec3 dy(0.0f,dim.y(),0.0f);
    Vec3 dz(0.0f,0.0f,dim.z());

    // create faces (we keep the quader and have: front side counter clockwise and then backside
    Vec3 vs[8];
    vs[0] = half;
    vs[1] = half + dx;
    vs[2] = half + dx + dy;
    vs[3] = half + dy;
    vs[4] = vs[0] + dz;
    vs[5] = vs[1] + dz;
    vs[6] = vs[2] + dz;
    vs[7] = vs[3] + dz;

    unsigned int tex = 0; 
    assert(textures.size()); 
    faces[0] = createRectangle(osgHandle, vs[0], vs[1], vs[2], 
			       textures[tex].repeatOnR, textures[tex].repeatOnS);
    addTexture(faces[0].get(),textures[tex]);
    if(textures.size()>tex+1) tex++;
    faces[1] = createRectangle(osgHandle, vs[3], vs[2], vs[6], 
			       textures[tex].repeatOnR, textures[tex].repeatOnS);
    addTexture(faces[1].get(),textures[tex]);
    if(textures.size()>tex+1) tex++;
    faces[2] = createRectangle(osgHandle, vs[7], vs[6], vs[5], 
			       textures[tex].repeatOnR, textures[tex].repeatOnS);
    addTexture(faces[2].get(),textures[tex]);
    if(textures.size()>tex+1) tex++;
    faces[3] = createRectangle(osgHandle, vs[4], vs[5], vs[1], 
			       textures[tex].repeatOnR, textures[tex].repeatOnS);
    addTexture(faces[3].get(),textures[tex]);
    if(textures.size()>tex+1) tex++;
    faces[4] = createRectangle(osgHandle, vs[2], vs[1], vs[5], 
			       textures[tex].repeatOnR, textures[tex].repeatOnS);
    addTexture(faces[4].get(),textures[tex]);
    if(textures.size()>tex+1) tex++;
    faces[5] = createRectangle(osgHandle, vs[7], vs[4], vs[0], 
			       textures[tex].repeatOnR, textures[tex].repeatOnS);
    addTexture(faces[5].get(),textures[tex]);

    for(int i=0; i<6; i++){
      transform->addChild(faces[i].get());
    }
    
  }

  void OSGBoxTex::setColor(const Color& color){
    fprintf(stderr,"setcolor of OsgBoxTex at the moment not implemented. use OsgHandle.changeColor() at initialization!");
    if(transform.get()){
      // Todo: destroy and init again
    }
  }

  void OSGBoxTex::applyTextures(){ 
    assert("Do not call setTexture after initialization of OSGBoxTex" == 0);
  }


  /******************************************************************************/
  OSGSphere::OSGSphere(float radius)
    : radius(radius) {
  }

  void OSGSphere::init(const OsgHandle& osgHandle, Quality quality){
    this->osgHandle=osgHandle;
    assert(osgHandle.parent || osgHandle.cfg->noGraphics);
    transform = new MatrixTransform;
    if (osgHandle.cfg->noGraphics)
      return;
    geode = new Geode;  
    transform->addChild(geode.get());
    osgHandle.parent->addChild(transform.get());

    shape = new ShapeDrawable(new Sphere(Vec3(0.0f, 0.0f, 0.0f), radius), 
                              osgHandle.cfg->tesselhints[quality]);
    shape->setColor(osgHandle.color);
    geode->addDrawable(shape.get());
    if(osgHandle.color.alpha() < 1.0){
      shape->setStateSet(new StateSet(*osgHandle.cfg->transparentState));
    }else{
      shape->setStateSet(new StateSet(*osgHandle.cfg->normalState));
    }

    shape->getOrCreateStateSet()->setAttributeAndModes(getMaterial(osgHandle.color).get(), 
						       StateAttribute::ON);
    applyTextures();
  }

  /******************************************************************************/
  OSGCapsule::OSGCapsule(float radius, float height)
    : radius(radius), height(height) {
  }

  void OSGCapsule::init(const OsgHandle& osgHandle, Quality quality){
    this->osgHandle=osgHandle;
    assert(osgHandle.parent || osgHandle.cfg->noGraphics);
    transform = new MatrixTransform;
    if (osgHandle.cfg->noGraphics)
      return;
    geode = new Geode;  
    transform->addChild(geode.get());
    osgHandle.parent->addChild(transform.get());

    shape = new ShapeDrawable(new Capsule(Vec3(0.0f, 0.0f, 0.0f), 
					  radius, height), osgHandle.cfg->tesselhints[quality]);
    shape->setColor(osgHandle.color);
    geode->addDrawable(shape.get());
    if(osgHandle.color.alpha() < 1.0){
      shape->setStateSet(new StateSet(*osgHandle.cfg->transparentState));
    }else{
      shape->setStateSet(new StateSet(*osgHandle.cfg->normalState));
    }

    shape->getOrCreateStateSet()->setAttributeAndModes(getMaterial(osgHandle.color).get(), 
						       StateAttribute::ON);
    applyTextures();
  }

  /******************************************************************************/
  OSGCylinder::OSGCylinder(float radius, float height)
    : radius(radius), height(height) {
  }

  void OSGCylinder::init(const OsgHandle& osgHandle, Quality quality){
    this->osgHandle=osgHandle;
    assert(osgHandle.parent || osgHandle.cfg->noGraphics);
    transform = new MatrixTransform;
    if (osgHandle.cfg->noGraphics)
      return;
    geode = new Geode;  
    transform->addChild(geode.get());
    osgHandle.parent->addChild(transform.get());

    shape = new ShapeDrawable(new Cylinder(Vec3(0.0f, 0.0f, 0.0f), 
					   radius, height), osgHandle.cfg->tesselhints[quality]);
    shape->setColor(osgHandle.color);
    geode->addDrawable(shape.get());
    if(osgHandle.color.alpha() < 1.0){
      shape->setStateSet(new StateSet(*osgHandle.cfg->transparentState));
    }else{
      shape->setStateSet(new StateSet(*osgHandle.cfg->normalState));
    }

    shape->getOrCreateStateSet()->setAttributeAndModes(getMaterial(osgHandle.color).get(), 
						       StateAttribute::ON);

    applyTextures();
  }

  /******************************************************************************/
  OSGMesh::OSGMesh(const std::string& filename, float scale, 
		   const osgDB::ReaderWriter::Options* options)
    : filename(filename), scale(scale), options(options) 
  {
  }

  OSGMesh::~OSGMesh(){
  }

  float OSGMesh::getRadius() {
    return getGroup()->getBound().radius(); 
  }



  void OSGMesh::internInit(const OsgHandle& osgHandle, bool loadAndDisplayMesh, Quality quality) {
    this->osgHandle=osgHandle;
       assert(osgHandle.parent || osgHandle.cfg->noGraphics);
       transform = new MatrixTransform;
       if (osgHandle.cfg->noGraphics)
         return;
       osgHandle.parent->addChild(transform.get());
       if (loadAndDisplayMesh) {
         scaletrans = new MatrixTransform;
         scaletrans->setMatrix(osg::Matrix::scale(scale,scale,scale));
         transform->addChild(scaletrans.get());
         mesh  = osgDB::readNodeFile(filename, options);
         if(mesh==0){
           fprintf(stderr,"OSGMesh: init: cannot load file: %s\n Abort!\n",filename.c_str());
           exit(1);
         }
           osg::StateSet* state = mesh->getOrCreateStateSet();
           //stateset->setMode(StateAttribute::ALPHAFUNC, StateAttribute::OFF);
           TexEnv* blendTexEnv = new TexEnv;
           blendTexEnv->setMode(TexEnv::BLEND);
           state->setTextureAttribute(1,blendTexEnv);
           state->setRenderBinDetails( 11, "DepthSortedBin");
           state->setRenderBinDetails( 2, "RenderBin" );
           state->setMode(GL_BLEND,osg::StateAttribute::ON);
          // state->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
           state->setRenderingHint( StateSet::TRANSPARENT_BIN );
           AlphaFunc* alphaFunc = new AlphaFunc;
           alphaFunc->setFunction(AlphaFunc::GEQUAL,0.1f);
           state->setAttributeAndModes( alphaFunc, StateAttribute::ON);
           state->setAttributeAndModes(getMaterial(osgHandle.color, Material::EMISSION).get(),
                          StateAttribute::OVERRIDE);
           /*   AMBIENT = GL_AMBIENT,
                DIFFUSE = GL_DIFFUSE,
                SPECULAR = GL_SPECULAR,
                EMISSION = GL_EMISSION,
                AMBIENT_AND_DIFFUSE = GL_AMBIENT_AND_DIFFUSE,
                OFF */
         scaletrans->addChild(mesh.get());

         applyTextures();
       }
  }

  void OSGMesh::virtualInit(const OsgHandle& osgHandle){
    internInit(osgHandle, false);
  }

  void OSGMesh::init(const OsgHandle& osgHandle, Quality quality){
    internInit(osgHandle, true, quality);
    
    // if(osgHandle.color.alpha() < 1.0){
    //   shape->setStateSet(osgHandle.cfg->transparentState);
    // }else{
    //   shape->setStateSet(osgHandle.cfg->normalState);
    // }
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
    Color amb (c*0.3);
    amb.alpha()=c.alpha();
    Color dif(c*0.7);
    dif.alpha()=c.alpha();
    Color spec(c*0.05);
    spec.alpha()=c.alpha();
    m->setAmbient(Material::FRONT_AND_BACK, amb);
    m->setDiffuse(Material::FRONT_AND_BACK, dif);
    m->setSpecular(Material::FRONT_AND_BACK, spec);
    m->setShininess(Material::FRONT_AND_BACK, 5.0f);
    //  m->setShininess(Material::FRONT_AND_BACK, 25.0f);
    return m;
  }



  osg::Geode* createRectangle(const OsgHandle& osgHandle, 
			      const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3,
			      double repeatOnR, double repeatOnS)
  {
    osg::Geode* geode = new osg::Geode();
    osg::Geometry* geometry = new osg::Geometry();
    geode->addDrawable(geometry); 

    // Specify the vertices:
    osg::Vec3Array* vertices = new osg::Vec3Array;
    vertices->push_back( v1 );
    vertices->push_back( v2 );
    vertices->push_back( v3 );
    vertices->push_back( v1 + (v3-v2));
    geometry->setVertexArray( vertices );

    // Create a QUAD primitive for the base by specifying the 
    // vertices from our vertex list that make up this QUAD:
    osg::DrawElementsUInt* base = 
      new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
    base->push_back(3);
    base->push_back(2);
    base->push_back(1);
    base->push_back(0);

    geometry->addPrimitiveSet(base);
    // one normal for the all corners
    osg::Vec3Array* normals = new osg::Vec3Array;
    Vec3 normal = (v1-v2) ^ (v3-v2);
    normal.normalize();
    normals->push_back(normal);
    geometry->setNormalArray(normals);
    geometry->setNormalBinding(osg::Geometry::BIND_PER_PRIMITIVE);

    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osgHandle.color);
    geometry->setColorArray(colors);
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

    if(repeatOnS<0){
      repeatOnS = (v1-v2).length() / (-repeatOnS);
    }
    if(repeatOnR<0){
      repeatOnR = (v3-v2).length() / (-repeatOnR);
    }

    osg::Vec2Array* texcoords = new osg::Vec2Array(4);
    (*texcoords)[0].set(0.00f,0.0f); 
    (*texcoords)[1].set(repeatOnS,0.0f);
    (*texcoords)[2].set(repeatOnS,repeatOnR); 
    (*texcoords)[3].set(0,repeatOnR); 
    geometry->setTexCoordArray(0,texcoords);

    return geode;
  }


  void addTexture(Geode* geode, const TextureDescr& tex){
    osg::Texture2D* texture = new osg::Texture2D;    
    // protect from being optimized away as static state:
    texture->setDataVariance(osg::Object::DYNAMIC); 
   // load an image by reading a file: 
   osg::Image* img = osgDB::readImageFile(tex.filename);
   // Assign the texture to the image we read from file: 
   texture->setImage(img);
   texture->setWrap( Texture2D::WRAP_S, Texture2D::REPEAT );   
   texture->setWrap( Texture2D::WRAP_T, Texture2D::REPEAT );
   texture->setWrap( Texture2D::WRAP_R, Texture2D::REPEAT );

   // Assign texture unit 0 of our new StateSet to the texture 
   // we just created and enable the texture.
   geode->getOrCreateStateSet()->setTextureAttributeAndModes
      (0,texture,osg::StateAttribute::ON);

  }


}

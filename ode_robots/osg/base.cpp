/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.27  2009-03-26 19:20:57  martius
 *   setUserLight  compability  with osg<2.6
 *
 *   Revision 1.26  2009/03/25 15:44:23  guettler
 *   ParallelSplitShadowMap: corrected light direction (using directional light), complete ground is now shadowed
 *
 *   Revision 1.25  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.24  2009/01/20 17:27:34  martius
 *   texture for background changeable from outside
 *
 *   Revision 1.23  2009/01/05 13:18:48  martius
 *   ambient light even more
 *
 *   Revision 1.22  2008/10/10 14:05:15  martius
 *   time display was wrong
 *
 *   Revision 1.21  2008/07/29 15:44:00  guettler
 *   removed forceFronCullFace option for pssm to obtain compatibiltity with OSG 2.6
 *
 *   Revision 1.20  2008/06/26 10:15:55  der
 *   changed far distance from 10 to 30 for parallel split shadow map
 *
 *   Revision 1.19  2008/05/05 09:35:35  guettler
 *   hud now displays if in pause mode
 *
 *   Revision 1.18  2008/04/30 13:13:20  guettler
 *   caption corrected
 *
 *   Revision 1.17  2008/04/23 07:17:16  martius
 *   makefiles cleaned
 *   new also true realtime factor displayed,
 *    warning if out of sync
 *   drawinterval in full speed is 10 frames, independent of the speed
 *
 *   Revision 1.16  2008/04/18 14:00:09  guettler
 *   cosmetic changes, added some printouts
 *
 *   Revision 1.15  2008/04/17 15:59:00  martius
 *   OSG2 port finished
 *
 *   Revision 1.14.2.7  2008/04/17 15:04:55  martius
 *   shadow is 5 on default, also improved cmd line parsing of shadow and texsize
 *
 *   Revision 1.14.2.6  2008/04/11 13:46:50  martius
 *   quickMP multithreading included
 *
 *   Revision 1.14.2.5  2008/04/11 10:41:35  martius
 *   config file added
 *
 *   Revision 1.14.2.4  2008/04/10 07:40:17  guettler
 *   Optimised parameters for the ShadowTechnique ParallelSplitShadowMap.
 *
 *   Revision 1.14.2.3  2008/04/09 14:25:35  martius
 *   shadow cmd line option
 *
 *   Revision 1.14.2.2  2008/04/09 13:57:59  guettler
 *   New ShadowTechnique added.
 *
 *   Revision 1.14.2.1  2008/04/09 10:18:41  martius
 *   fullscreen and window options done
 *   fonts on hud changed
 *
 *   Revision 1.14  2007/09/28 12:31:49  robot3
 *   The HUDSM is not anymore deduced from StatisticalTools, so the statistics
 *   can be updated independently from the HUD
 *   addPhysicsCallbackable and addGraphicsCallbackable now exists in Simulation
 *
 *   Revision 1.13  2007/09/28 10:24:04  robot3
 *   The WindowStatisticsManager is now called HUDStatisticsManager
 *
 *   Revision 1.12  2007/09/28 10:08:49  robot3
 *   fixed memory bugs, statistics are from now on aligned right
 *
 *   Revision 1.11  2007/09/27 10:47:04  robot3
 *   mathutils: moved abs to selforg/stl_adds.h
 *   simulation,base: added callbackable support,
 *   added WSM (HUDStatisticsManager) funtionality
 *
 *   Revision 1.10  2007/08/29 13:08:04  martius
 *   added HUD with time and caption
 *
 *   Revision 1.9  2007/04/05 15:10:36  martius
 *   different ground
 *
 *   Revision 1.8  2007/03/16 11:37:11  martius
 *   ground plane gets primitive to support substances
 *
 *   Revision 1.7  2007/02/21 14:26:18  martius
 *   increased ambient light
 *
 *   Revision 1.6  2006/09/20 15:30:47  martius
 *   shadowsize, light
 *
 *   Revision 1.5  2006/09/20 12:55:44  martius
 *   Light
 *
 *   Revision 1.4  2006/08/30 08:59:21  martius
 *   categories and collision mask used for static geoms to reduce number of collision checks
 *
 *   Revision 1.3  2006/08/04 15:05:42  martius
 *   documentation
 *
 *   Revision 1.2  2006/07/14 12:23:33  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.9  2006/06/29 16:35:56  robot3
 *   includes cleared up
 *
 *   Revision 1.1.2.8  2006/05/28 22:12:16  martius
 *   - noshadow cmdline flag
 *
 *   Revision 1.1.2.7  2006/05/18 11:42:51  robot3
 *   -shadowing the normal scene integrated (first version)
 *   -note that there is a bug that the shadow disappears
 *    after some time (e.g. 60 minutes)
 *
 *   Revision 1.1.2.6  2006/01/12 22:34:06  martius
 *   *** empty log message ***
 *
 *   Revision 1.1.2.5  2006/01/12 14:21:00  martius
 *   drawmode, material
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
 *   Revision 1.1.2.1  2005/12/06 17:40:59  martius
 *   base class for simulation
 *
 *                                                                 *
 ***************************************************************************/

#include <iostream>
#include <osg/Node>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/TexEnv>
#include <osg/TexGen>
#include <osg/Depth>
#include <osg/StateSet>
#include <osg/ClearNode>
#include <osg/Transform>
#include <osg/MatrixTransform>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/ShapeDrawable>
#include <osg/PolygonOffset>
#include <osg/CullFace>
#include <osg/TexGenNode>
#include <osgText/Font>

#include <osgUtil/CullVisitor>

#include <osgDB/Registry>
#include <osgDB/ReadFile>

#include <osgGA/AnimationPathManipulator>

#include "shadowcallback.h"
#include "base.h"
#include "primitive.h"

#include <selforg/callbackable.h>

#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowVolume>
#include <osgShadow/ShadowTexture>
#include <osgShadow/ShadowMap>
#include <osgShadow/SoftShadowMap>
#include <osgShadow/ParallelSplitShadowMap>


using namespace osg;

namespace lpzrobots {


  /********************************************************************
   * fragment shader for non textured objects (non-default, not used) *
   *******************************************************************/
  char fragmentShaderSource_noBaseTexture[] =
  "uniform sampler2DShadow shadowTexture; \n"
  "uniform vec2 ambientBias; \n"
  "\n"
  "void main(void) \n"
  "{ \n"
  "    ambientBias.x=0.8f; \n"
  "    ambientBias.y=0.4f; \n"
  "    gl_FragColor = gl_Color * (ambientBias.x + shadow2DProj( shadowTexture, gl_TexCoord[0] ) * ambientBias.y - 0.4f); \n"
  "}\n";


  /********************************************************************
   * fragment shader for textured objects (default, used)             *
   *******************************************************************/
  char fragmentShaderSource_withBaseTexture[] =
  "uniform sampler2D baseTexture; \n"
  "uniform sampler2DShadow shadowTexture; \n"
  "uniform vec2 ambientBias; \n"
  "\n"
  "void main(void) \n"
  "{ \n"
  "    vec4 color = gl_Color* texture2D( baseTexture, gl_TexCoord[0].xy ); \n"
  "    gl_FragColor = color * (ambientBias.x + shadow2DProj( shadowTexture, gl_TexCoord[1])  * ambientBias.y); \n"
  "}\n";

    Base::Base(const char* caption)
      : ground(0), caption(caption), groundTexture("Images/greenground.rgb"),
      hud(0), timestats(0), ReceivesShadowTraversalMask(0x1), CastsShadowTraversalMask(0x2),
	shadow(5), shadowTexSize(2048), useNVidia(1)
    {
    }


  Base::~Base(){
    if(ground ){
      //      Plane* plane = (Plane*) dGeomGetData(ground);
      //      delete plane;
      dGeomDestroy(ground);
    }
  }

  /* Shadow types: 1 - ShadowVolume 2 - ShadowTextue 3 - ParallelSplitShadowMap
   * 4 - SoftShadowMap 5 - ShadowMap
   */
  osg::Group* Base::createShadowedScene(osg::Node* sceneToShadow, LightSource* lightSource)
{
  // some conf variables for ShadowVolume
  bool twoSided=false;
  bool twoPass=false;
  bool updateLightPosition = false;
  // some conf variables for ParallelSplitShadowMap
  int mapCount =3;
  bool debugColor=false;
  int minNearSplit=0;
  int maxFarDist=50;
  int moveVCamFactor = 0;
  double polyoffsetfactor = -0.02;
  double polyoffsetunit = 1.0;
  // 20080728; guettler: commented out for OSG 2.6 compatibility
  //  bool cullFaceFront=false;

  // some conf variables for SoftShadowMap
  // make the shadow prenumba a little bit sharper then default (0.005)
  float softnessWidth = 0.002;


  // create root of shadowedScene
  osgShadow::ShadowedScene* shadowedScene = new osgShadow::ShadowedScene;

  shadowedScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
  shadowedScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);

  // add ShadowTechnique
  int shadowType=(int)shadow;
  switch(shadowType) {
  case 1: /// ShadowVolume
    {
      // hint to tell viewer to request stencil buffer when setting up windows
      osg::DisplaySettings::instance()->setMinimumNumStencilBits(8);

      osg::ref_ptr<osgShadow::ShadowVolume> sv = new osgShadow::ShadowVolume;
      sv->setDynamicShadowVolumes(updateLightPosition);
      if (twoSided)
        sv->setDrawMode(osgShadow::ShadowVolumeGeometry::STENCIL_TWO_SIDED);
      if (twoPass)
        sv->setDrawMode(osgShadow::ShadowVolumeGeometry::STENCIL_TWO_PASS);
      shadowedScene->setShadowTechnique(sv.get());
    }
    break;
  case 2: /// ShadowTexture
    {
      osg::ref_ptr<osgShadow::ShadowTexture> st = new osgShadow::ShadowTexture;
      shadowedScene->setShadowTechnique(st.get());
    }
    break;
  case 3: /// ParallelSplitShadowMap
    {
        osg::ref_ptr<osgShadow::ParallelSplitShadowMap> pssm = new osgShadow::ParallelSplitShadowMap(NULL,mapCount);

        pssm->setTextureResolution(shadowTexSize);

        if (debugColor)
          pssm->setDebugColorOn();

      	pssm->setMinNearDistanceForSplits(minNearSplit);

        pssm->setMaxFarDistance(maxFarDist);

        if ( maxFarDist > 0 ) {
        	int moveVCamFactor = 0;
            pssm->setMoveVCamBehindRCamFactor(moveVCamFactor);
        }

/*        double polyoffsetfactor = pssm->getPolygonOffset().x();
        double polyoffsetunit   = pssm->getPolygonOffset().y();
        while (arguments.read("--PolyOffset-Factor", polyoffsetfactor));
        while (arguments.read("--PolyOffset-Unit", polyoffsetunit));
        pssm->setPolygonOffset(osg::Vec2(polyoffsetfactor,polyoffsetunit));*/

        shadowedScene->setShadowTechnique(pssm.get());

#if OPENSCENEGRAPH_MAJOR_VERSION == 2 &&  OPENSCENEGRAPH_MINOR_VERSION >= 6
        pssm->setUserLight(lightSource->getLight());
#endif
	/*
	osg::ref_ptr<osgShadow::ParallelSplitShadowMap> pssm = new osgShadow::ParallelSplitShadowMap(NULL,mapCount);
	
	if (debugColor)
	  pssm->setDebugColorOn();
	
	if (useNVidia!=0)
        pssm->setPolygonOffset(osg::Vec2(10.0f,20.0f)); //NVidea
	else
	  pssm->setPolygonOffset(osg::Vec2(polyoffsetfactor,polyoffsetunit)); //ATI Radeon
	
	// 20080728; guettler: commented out for OSG 2.6 compatibility
	//      if (cullFaceFront)
	//        pssm->forceFrontCullFace();

	shadowedScene->setShadowTechnique(pssm.get());*/
    }
    break;
  case 4: /// SoftShadowMap
    {
      osg::ref_ptr<osgShadow::SoftShadowMap> sm = new osgShadow::SoftShadowMap;
      sm->setSoftnessWidth(softnessWidth);
      shadowedScene->setShadowTechnique(sm.get());
    }
    break;
  case 5: /// ShadowMap
  default:
    {
        osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
        shadowedScene->setShadowTechnique(sm.get());
      sm->setTextureSize(osg::Vec2s((int)shadowTexSize,(int)shadowTexSize));
    }
    break;
  }
  shadowedScene->addChild(sceneToShadow);

  return shadowedScene;
}

  osg::Group* Base::createShadowedScene(osg::Node* shadowed,
					osg::Vec3 posOfLight,
					unsigned int unit)
  {
    osg::Group* group = new osg::Group;

    unsigned int tex_width  = (int)shadowTexSize; // 1024; // up to 2048 is possible but slower
    unsigned int tex_height = (int)shadowTexSize; // 1024; // up to 2048 is possible but slower

    osg::Texture2D* texture = new osg::Texture2D;
    texture->setTextureSize(tex_width, tex_height);

    texture->setInternalFormat(GL_DEPTH_COMPONENT);
    texture->setShadowComparison(true);
    texture->setShadowTextureMode(Texture::LUMINANCE);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

    // set up the render to texture camera.
    {
      // create the camera
      osg::CameraNode* camera = new osg::CameraNode;

      camera->setClearMask(GL_DEPTH_BUFFER_BIT);
      camera->setClearColor(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
      camera->setComputeNearFarMode(osg::CameraNode::DO_NOT_COMPUTE_NEAR_FAR);
      //      std::cerr << camera->getNearFarRatio() <<  std::endl;
      //      camera->setNearFarRatio(0.0001);

      // set viewport
      camera->setViewport(0,0,tex_width,tex_height);

      osg::StateSet*  _local_stateset = camera->getOrCreateStateSet();

      _local_stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);


      float factor = 0.0f;
      float units = 1.0f;

      ref_ptr<PolygonOffset> polygon_offset = new PolygonOffset;
      polygon_offset->setFactor(factor);
      polygon_offset->setUnits(units);
      _local_stateset->setAttribute(polygon_offset.get(), StateAttribute::ON | StateAttribute::OVERRIDE);
      _local_stateset->setMode(GL_POLYGON_OFFSET_FILL, StateAttribute::ON | StateAttribute::OVERRIDE);

      ref_ptr<CullFace> cull_face = new CullFace;
      cull_face->setMode(CullFace::FRONT);
      _local_stateset->setAttribute(cull_face.get(), StateAttribute::ON | StateAttribute::OVERRIDE);
      _local_stateset->setMode(GL_CULL_FACE, StateAttribute::ON | StateAttribute::OVERRIDE);


      // set the camera to render before the main camera.
      camera->setRenderOrder(osg::CameraNode::PRE_RENDER);

      // tell the camera to use OpenGL frame buffer object where supported.
      camera->setRenderTargetImplementation(osg::CameraNode::FRAME_BUFFER_OBJECT);

      // attach the texture and use it as the color buffer.
      camera->attach(osg::CameraNode::DEPTH_BUFFER, texture);

      // add subgraph to render
      camera->addChild(shadowed);

      group->addChild(camera);

      // create the texgen node to project the tex coords onto the subgraph
      osg::TexGenNode* texgenNode = new osg::TexGenNode;
      texgenNode->setTextureUnit(unit);
      group->addChild(texgenNode);

      // set an update callback to keep moving the camera and tex gen in the right direction.
      group->setUpdateCallback(new ShadowDrawCallback(posOfLight, camera, texgenNode));
    }


    // set the shadowed subgraph so that it uses the texture and tex gen settings.
    {
      osg::Group* shadowedGroup = new osg::Group;
      shadowedGroup->addChild(shadowed);
      group->addChild(shadowedGroup);

      osg::StateSet* stateset = shadowedGroup->getOrCreateStateSet();
      stateset->setTextureAttributeAndModes(unit,texture,osg::StateAttribute::ON);
      stateset->setTextureMode(unit,GL_TEXTURE_GEN_S,osg::StateAttribute::ON);
      stateset->setTextureMode(unit,GL_TEXTURE_GEN_T,osg::StateAttribute::ON);
      stateset->setTextureMode(unit,GL_TEXTURE_GEN_R,osg::StateAttribute::ON);

      stateset->setTextureMode(unit,GL_TEXTURE_GEN_Q,osg::StateAttribute::ON);


      osg::Program* program = new osg::Program;
      stateset->setAttribute(program);
      if (unit==0) {
	std::cout << "not using textures." << std::endl;
	osg::Shader* fragment_shader =
	  new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource_noBaseTexture);
	program->addShader(fragment_shader);

	// uniforms are for the shader program
	osg::Uniform* shadowTextureSampler = new osg::Uniform("shadowTexture",(int)unit);
	stateset->addUniform(shadowTextureSampler);
      } else {
	std::cout << "using textures." << std::endl;
	osg::Shader* fragment_shader =
	  new osg::Shader(osg::Shader::FRAGMENT, fragmentShaderSource_withBaseTexture);
	program->addShader(fragment_shader);

	// uniforms are for the shader program
	osg::Uniform* baseTextureSampler = new osg::Uniform("baseTexture",0);
	stateset->addUniform(baseTextureSampler);

	osg::Uniform* shadowTextureSampler = new osg::Uniform("shadowTexture",(int)unit);
	stateset->addUniform(shadowTextureSampler);
      }

      // uniform is for the shader program
      osg::Uniform* ambientBias = new osg::Uniform("ambientBias",osg::Vec2(0.7f,0.5f));
      stateset->addUniform(ambientBias);

    }

    return group;
  }


  osg::Node* Base::createHUD()
  {
    osg::Geode* geode = new osg::Geode();

    // turn lighting off for the text and disable depth test to ensure its always ontop.
    osg::StateSet* stateset = geode->getOrCreateStateSet();
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    osgText::Font* font = osgText::readFontFile("fonts/fudd.ttf");

    osg::Vec3 position(500.0f,9.0f,0.0f);
    Color textColor(0.2,0.2,0.0);
    int fontsize=12;

    {
      osgText::Text* text = new  osgText::Text;
      geode->addDrawable( text );
      text->setCharacterSize(fontsize);
      text->setFont(font);
      text->setPosition(position);
      text->setColor(textColor);
      text->setAlignment(osgText::Text::RIGHT_BASE_LINE);
      if(caption) text->setText(caption);
      else text->setText("lpzrobots Simulator          Martius, Der, G�ttler");
    }

    // timing
    position=osg::Vec3(12.0f,9.0f,0.0f);
    {
      timestats = new  osgText::Text;
      geode->addDrawable( timestats);
      timestats->setCharacterSize(fontsize);
      timestats->setFont(font);
      timestats->setPosition(position);
      timestats->setColor(textColor);
      setTimeStats(0,0,0,false);
    }

    {
      osg::Geometry* geom = new osg::Geometry;

      osg::Vec3Array* vertices = new osg::Vec3Array;
      double xMin=6;
      double xMax=506;
      double yMin=6;
      double yMax=22;
      double depth=-0.1;
      vertices->push_back(osg::Vec3(xMin,yMax,depth));
      vertices->push_back(osg::Vec3(xMin,yMin,depth));
      vertices->push_back(osg::Vec3(xMax,yMin,depth));
      vertices->push_back(osg::Vec3(xMax,yMax,depth));

      vertices->push_back(osg::Vec3());
      geom->setVertexArray(vertices);

      osg::Vec3Array* normals = new osg::Vec3Array;
      normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
      geom->setNormalArray(normals);
      geom->setNormalBinding(osg::Geometry::BIND_OVERALL);

      osg::Vec4Array* colors = new osg::Vec4Array;
      colors->push_back(osg::Vec4(1.0f,1.0,0.8f,0.5f));
      geom->setColorArray(colors);
      geom->setColorBinding(osg::Geometry::BIND_OVERALL);

      geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));

      osg::StateSet* stateset = geom->getOrCreateStateSet();
      stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
      //stateset->setAttribute(new osg::PolygonOffset(1.0f,1.0f),osg::StateAttribute::ON);
      stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

      geode->addDrawable(geom);

      // create HUDStatisticsManager and register it for being called back every step
      hUDStatisticsManager = new HUDStatisticsManager(geode,font);
      this->addGraphicsCallbackable(hUDStatisticsManager);
      this->addPhysicsCallbackable(hUDStatisticsManager->getStatisticTools());
    }

    osg::CameraNode* camera = new osg::CameraNode;

    // set the projection matrix
    camera->setProjectionMatrix(osg::Matrix::ortho2D(0,512,0,384));

    // set the view matrix
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());

    // only clear the depth buffer
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);

    // draw subgraph after main camera view.
    camera->setRenderOrder(osg::CameraNode::POST_RENDER);

    camera->addChild(geode);

    return camera;
  }

  void Base::setTimeStats(double time, double realtimefactor,
			  double truerealtimefactor, bool pause){
    if(timestats){
      char buffer[100];
      int minutes = int(time)/60;
      int seconds = int(time)%60;
      if (pause) {
        sprintf(buffer,"Time: %02i:%02i  Speed: %.1fx (paused)",minutes,
                seconds,realtimefactor);
      } else if (realtimefactor>0){
	if(fabs(truerealtimefactor/realtimefactor-1)<0.15)
	  sprintf(buffer,"Time: %02i:%02i  Speed: %.1fx",minutes,
		  seconds,realtimefactor);
	else
	  sprintf(buffer,"Time: %02i:%02i  Speed: %.1fx(%.1fx!)",minutes,
		  seconds,truerealtimefactor, realtimefactor);
      } else
        sprintf(buffer,"Time: %02i:%02i  Speed: %.1fx (max)",minutes, seconds,truerealtimefactor);
      timestats->setText(buffer);
    }
  }


  Group* Base::makeScene(){
    // no database loaded so automatically create Ed Levin Park..
    root = new Group;
    // the base and sky subgraphs go to set the earth sky of the
    // model and clear the color and depth buffer for us, by using
    // osg::Depth, and setting their bin numbers to less than 0,
    // to force them to draw before the rest of the scene.
    ClearNode* clearNode = new ClearNode;

    // use a transform to make the sky and base around with the eye point.
    osg::Transform* transform = new osg::Transform;//MoveEarthySkyWithEyePointTransform;

    // add the transform to the earth sky.
    clearNode->addChild(transform);

    root->addChild(clearNode);
    hud = createHUD();
    if(hud) root->addChild(hud);

    // transform's value isn't known until in the cull traversal so its bounding
    // volume can't be determined, therefore culling will be invalid,
    // so switch it off, this cause all our parents to switch culling
    // off as well. But don't worry culling will be back on once underneath
    // this node or any other branch above this transform.

    transform->setCullingActive(false);

    // add the sky and base layer.
    transform->addChild(makeSky());  // bin number -2 so drawn first.

    int shadowType=(int)shadow;
    // 20090325; guettler: if using pssm (shadowtype 3), add also the ground to the shadowed scene
    if (shadowType!=3)
    	transform->addChild(makeGround()); // bin number -1 so draw second.

    LightSource* lightSource = makeLights(root->getOrCreateStateSet());
    transform->addChild(lightSource);

    Group* scene = new Group; // create an extra group for the normal scene

    if(shadowType){
      // enable shadows
      Group* shadowedScene;

      // transform the Vec4 in a Vec3
      //osg::Vec3 posOfLight;
      // posOfLight=lightSource->getLight()->getPosition();

      // create the shadowed scene, using textures
      //shadowedScene = createShadowedScene(scene,posOfLight,1);
      shadowedScene = createShadowedScene(scene,lightSource);

      // 20090325; guettler: if using pssm (shadowtype 3), add also the ground to the shadowed scene
      if (shadowType==3)
      	shadowedScene->addChild(makeGround()); // bin number -1 so draw second.

      // add the shadowed scene to the root
      root->addChild(shadowedScene);
    }else {
      root->addChild(scene);
    }

    // the normal scene
    return scene;
  }

  Node* Base::makeSky() {
    // taken from osghangglider example
    int i, j;
    float lev[] = {-5.0, -1.0, 2.0, 12.0, 30.0, 60.0, 90.0  };
    float cc[][4] =
      {
        { 0.0, 0.0, 0.15 },
        { 0.0, 0.0, 0.15 },
        { 0.4, 0.4, 0.7 },
        { 0.2, 0.2, 0.6 },
        { 0.1, 0.1, 0.6 },
        { 0.1, 0.1, 0.6 },
        { 0.1, 0.1, 0.6 },
      };
    float x, y, z;
    float alpha, theta;
    float radius = 1000.0f;
    int nlev = sizeof( lev )/sizeof(float);

    Geometry *geom = new Geometry;

    Vec3Array& coords = *(new Vec3Array(19*nlev));
    Vec4Array& colors = *(new Vec4Array(19*nlev));
    Vec2Array& tcoords = *(new Vec2Array(19*nlev));


    int ci = 0;

    for( i = 0; i < nlev; i++ )
      {
        for( j = 0; j <= 18; j++ )
	  {
            alpha = osg::DegreesToRadians(lev[i]);
            theta = osg::DegreesToRadians((float)(j*20));

            x = radius * cosf( alpha ) * cosf( theta );
            y = radius * cosf( alpha ) * -sinf( theta );
            z = radius * sinf( alpha );

            coords[ci][0] = x;
            coords[ci][1] = y;
            coords[ci][2] = z;

            colors[ci][0] = cc[i][0];
            colors[ci][1] = cc[i][1];
            colors[ci][2] = cc[i][2];
            colors[ci][3] = 1.0;

            tcoords[ci][0] = (float)j/18.0;
            tcoords[ci][1] = (float)i/(float)(nlev-1);

            ci++;
	  }


      }

    for( i = 0; i < nlev-1; i++ )
      {
        DrawElementsUShort* drawElements = new DrawElementsUShort(PrimitiveSet::TRIANGLE_STRIP);
        drawElements->reserve(38);

        for( j = 0; j <= 18; j++ )
	  {
            drawElements->push_back((i+1)*19+j);
            drawElements->push_back((i+0)*19+j);
	  }

        geom->addPrimitiveSet(drawElements);
      }

    geom->setVertexArray( &coords );
    geom->setTexCoordArray( 0, &tcoords );

    geom->setColorArray( &colors );
    geom->setColorBinding( Geometry::BIND_PER_VERTEX );


    Texture2D *tex = new Texture2D;
    tex->setImage(osgDB::readImageFile("Images/white.rgb"));

    StateSet *dstate = new StateSet;

    dstate->setTextureAttributeAndModes(0, tex, StateAttribute::OFF );
    dstate->setTextureAttribute(0, new TexEnv );
    dstate->setMode( GL_LIGHTING, StateAttribute::OFF );
    dstate->setMode( GL_CULL_FACE, StateAttribute::ON );


    // clear the depth to the far plane.
    osg::Depth* depth = new osg::Depth;
    depth->setFunction(osg::Depth::ALWAYS);
    depth->setRange(1.0,1.0);
    dstate->setAttributeAndModes(depth,StateAttribute::ON );

    dstate->setRenderBinDetails(-2,"RenderBin");

    geom->setStateSet( dstate );

    Geode *geode = new Geode;
    geode->addDrawable( geom );

    geode->setName( "Sky" );

    return geode;
  }

  Node* Base::makeGround(){ // the old ground, is NOT used for shadowing!
    float ir = 1000.0f;
    float texscale =0.2;
    Vec3Array *coords = new Vec3Array(4);
    Vec2Array *tcoords = new Vec2Array(4);
    Vec4Array *colors = new Vec4Array(1);

    (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);

    (*coords)[0].set(-ir,-ir,0.0f);
    (*coords)[1].set( ir,-ir,0.0f);
    (*coords)[2].set( ir, ir,0.0f);
    (*coords)[3].set(-ir, ir,0.0f);
    (*tcoords)[0].set(-texscale*ir,-texscale*ir);
    (*tcoords)[1].set(-texscale*ir, texscale*ir);
    (*tcoords)[2].set( texscale*ir, texscale*ir);
    (*tcoords)[3].set( texscale*ir,-texscale*ir);

//     int i, c;
//     float theta;
//     float ir = 1000.0f;

//     Vec3Array *coords = new Vec3Array(19);
//     Vec2Array *tcoords = new Vec2Array(19);
//     Vec4Array *colors = new Vec4Array(1);

//     (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);

//     c = 0;
//     (*coords)[c].set(0.0f,0.0f,0.0f);
//     (*tcoords)[c].set(0.0f,0.0f);

//     for( i = 0; i <= 18; i++ )
//       {
//         theta = osg::DegreesToRadians((float)i * 20.0);

//         (*coords)[c].set(ir * cosf( theta ), ir * sinf( theta ), -0.001f);
//         (*tcoords)[c].set((*coords)[c][0],(*coords)[c][1]);

//         c++;
//       }

    Geometry *geom = new Geometry;

    geom->setVertexArray( coords );

    geom->setTexCoordArray( 0, tcoords );

    geom->setColorArray( colors );
    geom->setColorBinding( Geometry::BIND_OVERALL );

    //    geom->addPrimitiveSet( new DrawArrays(PrimitiveSet::TRIANGLE_FAN,0,19) );
    geom->addPrimitiveSet( new DrawArrays(PrimitiveSet::TRIANGLE_FAN,0,4) );

    Texture2D *tex = new Texture2D;

    tex->setImage(osgDB::readImageFile(groundTexture));
    tex->setWrap( Texture2D::WRAP_S, Texture2D::REPEAT );
    tex->setWrap( Texture2D::WRAP_T, Texture2D::REPEAT );

    StateSet *dstate = new StateSet;
    dstate->setMode( GL_LIGHTING, StateAttribute::OFF );
    dstate->setTextureAttributeAndModes(0, tex, StateAttribute::ON );

    dstate->setTextureAttribute(0, new TexEnv );

    //     // clear the depth to the far plane.
    //     osg::Depth* depth = new osg::Depth;
    //     depth->setFunction(osg::Depth::ALWAYS);
    //     depth->setRange(1.0,1.0);
    //     dstate->setAttributeAndModes(depth,StateAttribute::ON );

    dstate->setRenderBinDetails(-1,"RenderBin");
    geom->setStateSet( dstate );

    Geode *geode = new Geode;
    geode->addDrawable( geom );
    geode->setName( "Ground" );

    // add ODE Ground here (physical plane)
    ground = dCreatePlane ( odeHandle.space , 0 , 0 , 1 , 0 );
    dGeomSetCategoryBits(ground,Primitive::Stat);
    dGeomSetCollideBits(ground,~Primitive::Stat);
    // assign a dummy primitive to the ground plane to have substance (material) support
    Plane* plane = new Plane();
    dGeomSetData(ground, (void*)plane);
    //    std::cout << "GROUND: " << ground << std::endl;

    return geode;
  }


  osg::LightSource* Base::makeLights(osg::StateSet* stateset)
  {
	  /*
    // create a spot light.
    Light* light_0 = new Light;
    light_0->setLightNum(0);
    //    light_0->setPosition(Vec4(0.0f, 0.0f, 50.0f, 1.0f));
    light_0->setPosition(Vec4(40.0f, 40.0f, 50.0f, 1.0f));
    //    light_0->setAmbient(Vec4(0.25f, 0.25f, 0.25f, 1.0f));
    light_0->setAmbient(Vec4(0.7f, 0.7f, 0.7f, 1.0f));  // Georg 21.07.2007 changed from 0.5 to 0.7
    //light_0->setAmbient(Vec4(0.9f, 0.9f, 0.9f, 1.0f));  // Georg 05.01.2008 changed from 0.7 to 0.9
    light_0->setDiffuse(Vec4(0.8f, 0.8f, 0.8f, 1.0f));
    //    light_0->setDirection(Vec3(-1.0f, -1.0f, -1.2f));
    light_0->setSpecular(Vec4(1.0f, 0.9f, 0.8f, 1.0f));

    LightSource* light_source_0 = new LightSource;
    light_source_0->setLight(light_0);
    light_source_0->setLocalStateSetModes(StateAttribute::ON);
    light_source_0->setStateSetModes(*stateset, StateAttribute::ON);

    return light_source_0;
    */

    // create a directional light (infinite distance place at 45 degrees)
    osg::Light* myLight = new osg::Light;
    myLight->setLightNum(1);
    myLight->setPosition(osg::Vec4(1.0,1.0,1.0,0.0f));
    myLight->setDirection(osg::Vec3(-1.0, -1.0, -1.0));
    myLight->setAmbient(osg::Vec4(0.5f,0.5f,0.5f,1.0f));
    myLight->setDiffuse(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    myLight->setConstantAttenuation(1.0f);

    osg::LightSource* lightS = new osg::LightSource;
    lightS->setLight(myLight);
    lightS->setLocalStateSetModes(osg::StateAttribute::ON);

    lightS->setStateSetModes(*stateset,osg::StateAttribute::ON);

    return lightS;


  }


  /********************************************************************************/

  bool MoveEarthySkyWithEyePointTransform::computeLocalToWorldMatrix(osg::Matrix& matrix,osg::NodeVisitor* nv) const
  {
    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
    if (cv)
      {
	osg::Vec3 eyePointLocal = cv->getEyeLocal();
	matrix.preMult(osg::Matrix::translate(eyePointLocal.x(),eyePointLocal.y(),0.0f));
      }
    return true;
  }

  bool MoveEarthySkyWithEyePointTransform::computeWorldToLocalMatrix(osg::Matrix& matrix,osg::NodeVisitor* nv) const
  {
    std::cout<<"computing transform"<<std::endl;

    osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
    if (cv)
      {
	osg::Vec3 eyePointLocal = cv->getEyeLocal();
	matrix.postMult(osg::Matrix::translate(-eyePointLocal.x(),-eyePointLocal.y(),0.0f));
      }
    return true;
  }

  void Base::addGraphicsCallbackable(Callbackable* callbackable){
    graphicsCallbackables.push_back(callbackable);
  }

  void Base::addPhysicsCallbackable(Callbackable* callbackable){
    physicsCallbackables.push_back(callbackable);
  }


// Helper
int Base::contains(char **list, int len,  const char *str) {
  for(int i=0; i<len; i++) {
    if(strcmp(list[i],str) == 0)
      return i+1;
  }
  return 0;
}


}

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

#include <iostream>
#include <assert.h>
#include <osg/Node>
#include <osg/Camera>
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

#include "base.h"

#include "osgprimitive.h"
#include "primitive.h"

#include <selforg/callbackable.h>

#include <osgShadow/ShadowedScene>
// #include <osgShadow/ShadowVolume>
// #include <osgShadow/LightSpacePerspectiveShadowMap>
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

     Base::Base(const std::string& caption)
       : Configurable("lpzrobots-ode_robots", "0.7"), ground(0),
         caption(caption), title(""),
         groundTexture("Images/whiteground.jpg"),
         dummy(0), hud(0), timestats(0), captionline(0), titleline(0),
         statlineprop(11,12,"text"),
         plane(0), hUDStatisticsManager(0), ReceivesShadowTraversalMask(0x1),
         CastsShadowTraversalMask(0x2), shadowTexSize(2048), useNVidia(1)
     {
     }


  Base::~Base(){
  }

  void Base::base_close(){
    if(plane) delete plane;
    //    if(dummy) dummy->unref(); // this happens automatically
    if(ground ){
      dGeomDestroy(ground);
    }
    if(hUDStatisticsManager) delete hUDStatisticsManager;
  }


  /** Shadow types: 1 - LightSpacePerspectiveShadowMap
   * 2 - ShadowTextue 3 - ParallelSplitShadowMap
   * 4 - SoftShadowMap 5 - ShadowMap
   */
  osgShadow::ShadowedScene* Base::createShadowedScene(osg::Node* sceneToShadow, LightSource* lightSource, int shadowType)
{

  // int moveVCamFactor = 0;
  // double polyoffsetfactor = -0.02;
  // double polyoffsetunit = 1.0;
  // 20080728; guettler: commented out for OSG 2.6 compatibility

  // some conf variables for SoftShadowMap
  // make the shadow prenumba a little bit sharper then default (0.005)

  osgShadow::ShadowedScene* shadowedScene = new osgShadow::ShadowedScene;

  shadowedScene->setReceivesShadowTraversalMask(ReceivesShadowTraversalMask);
  shadowedScene->setCastsShadowTraversalMask(CastsShadowTraversalMask);
  shadowedScene->setNodeMask(CastsShadowTraversalMask | ReceivesShadowTraversalMask);

  // add ShadowTechnique

  switch(shadowType) {
//   case 1: /// LightSpacePerspectiveShadowMap
//     {
//       osg::ref_ptr<osgShadow::MinimalShadowMap> sm =
//         new osgShadow::LightSpacePerspectiveShadowMapDB();
//       shadowedScene->setShadowTechnique( sm.get() );
//       float minLightMargin = 10.f;
//       float maxFarPlane = 50;
//       unsigned int baseTexUnit = 0;
//       unsigned int shadowTexUnit = 7;
//       sm->setMinLightMargin( minLightMargin );
//       sm->setMaxFarPlane( maxFarPlane );
//       sm->setTextureSize( osg::Vec2s((int)shadowTexSize,(int)shadowTexSize) );
//       sm->setShadowTextureCoordIndex( shadowTexUnit );
//       sm->setShadowTextureUnit( shadowTexUnit );
//       sm->setBaseTextureCoordIndex( baseTexUnit );
//       sm->setBaseTextureUnit( baseTexUnit );

//     }
  // case 1: /// ShadowVolume
  //   {
  //     bool twoSided=false;
  //     bool twoPass=false;
  //     // some conf variables for ShadowVolume
  //     bool updateLightPosition = false;

  //     // hint to tell viewer to request stencil buffer when setting up windows
  //     osg::DisplaySettings::instance()->setMinimumNumStencilBits(8);

  //     osg::ref_ptr<osgShadow::ShadowVolume> sv = new osgShadow::ShadowVolume;
  //     sv->setDynamicShadowVolumes(updateLightPosition);
  //     if (twoSided)
  //       sv->setDrawMode(osgShadow::ShadowVolumeGeometry::STENCIL_TWO_SIDED);
  //     if (twoPass)
  //       sv->setDrawMode(osgShadow::ShadowVolumeGeometry::STENCIL_TWO_PASS);
  //     shadowedScene->setShadowTechnique(sv.get());
  //   }
  //   break;
  case 2: /// ShadowTexture
    {
      osg::ref_ptr<osgShadow::ShadowTexture> st = new osgShadow::ShadowTexture;
      shadowedScene->setShadowTechnique(st.get());
    }
    break;
  case 3: /// ParallelSplitShadowMap
    {
      // some conf variables for ParallelSplitShadowMap
      int mapCount =3;
      bool debugColor=false;
      int minNearSplit=3;
      int maxFarDist=50;

        osg::ref_ptr<osgShadow::ParallelSplitShadowMap> pssm = new osgShadow::ParallelSplitShadowMap(NULL,mapCount);
        //        std::cout << pssm->getAmbientBias().x  << " " << pssm->getAmbientBias().y << std::endl
        pssm->setAmbientBias(Vec2(0,.3));

        pssm->setTextureResolution(shadowTexSize);

        if (debugColor)
          pssm->setDebugColorOn();

              pssm->setMinNearDistanceForSplits(minNearSplit);

        pssm->setMaxFarDistance(maxFarDist);

        if ( maxFarDist > 0 ) {
                int moveVCamFactor = 0;
            pssm->setMoveVCamBehindRCamFactor(moveVCamFactor);
        }

        if (useNVidia!=0)
          // pssm->setPolygonOffset(osg::Vec2(10.0f,20.0f));
        //        pssm->setPolygonOffset(osg::Vec2(1.0f,4.0f));
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
              if (cullFaceFront)
                pssm->forceFrontCullFace();

        shadowedScene->setShadowTechnique(pssm.get());*/
    }
    break;
  case 4: /// SoftShadowMap
    {
      float softnessWidth = 0.002;
      osg::ref_ptr<osgShadow::SoftShadowMap> sm = new osgShadow::SoftShadowMap;
      sm->setSoftnessWidth(softnessWidth);
      sm->setTextureSize(osg::Vec2s((int)shadowTexSize,(int)shadowTexSize));
      shadowedScene->setShadowTechnique(sm.get());
      sm->setAmbientBias(Vec2(0.8,.2));
    }
    break;
  case 5: /// ShadowMap
  default:
    {
      osg::ref_ptr<osgShadow::ShadowMap> sm = new osgShadow::ShadowMap;
      shadowedScene->setShadowTechnique(sm.get());
      sm->setTextureSize(osg::Vec2s((int)shadowTexSize,(int)shadowTexSize));
      sm->setAmbientBias(Vec2(0.8,.2));

    }
    break;
  }
  shadowedScene->addChild(sceneToShadow);

  return shadowedScene;
}

  osg::Node* Base::createHUD(OsgScene* scene, const OsgConfig& config)
  {
    osg::Geode* geode = new osg::Geode();

    // turn lighting off for the text and disable depth test to ensure its always ontop.
    osg::StateSet* stateset = geode->getOrCreateStateSet();
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    osgText::Font* font = osgText::readFontFile("fonts/fudd.ttf");

    Color textColor = config.cs->color(statlineprop.fontColor);
    int fontsize=statlineprop.fontSizeText;

    // caption (right)
    osg::Vec3 position(500.0f,9.0f,0.0f);
    {
      captionline = new osgText::Text;
      geode->addDrawable( captionline );
      captionline->setCharacterSize(fontsize);
      captionline->setFont(font);
      captionline->setPosition(position);
      captionline->setColor(textColor);
      captionline->setAlignment(osgText::Text::RIGHT_BASE_LINE);
      captionline->setText(caption.c_str());
    }

    // title (center)
    position = osg::Vec3(255.0f,9.0f,0.0f);
    {
      titleline = new  osgText::Text;
      geode->addDrawable( titleline );
      titleline->setCharacterSize(fontsize);
      titleline->setFont(font);
      titleline->setPosition(position);
      titleline->setColor(textColor);
      titleline->setAlignment(osgText::Text::CENTER_BASE_LINE);
      titleline->setText(title.c_str());
    }

    fontsize=statlineprop.fontSizeTime;

    // timing (left)
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
      double yMax=yMin+4+std::max(statlineprop.fontSizeTime, statlineprop.fontSizeText);
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
      Color hudcolor=config.cs->color("hud");
      hudcolor.alpha() = 0.5f;
      colors->push_back(hudcolor);
      geom->setColorArray(colors);
      geom->setColorBinding(osg::Geometry::BIND_OVERALL);

      geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));

      osg::StateSet* stateset = geom->getOrCreateStateSet();
      stateset->setMode(GL_BLEND,osg::StateAttribute::ON);
      //stateset->setAttribute(new osg::PolygonOffset(1.0f,1.0f),osg::StateAttribute::ON);
      stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

      geode->addDrawable(geom);

      // create HUDStatisticsManager and register it for being called back every step
      createHUDManager(geode,font);
    }

    osg::Camera* camera = new osg::Camera;

    // set the projection matrix
    camera->setProjectionMatrix(osg::Matrix::ortho2D(0,512,0,384));

    // set the view matrix
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());

    // only clear the depth buffer
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);

    // draw subgraph after main camera view.
    camera->setRenderOrder(osg::Camera::POST_RENDER);

    camera->addChild(geode);
    scene->hud = geode;

    return camera;
  }

  void  Base::createHUDManager(osg::Geode* geode, osgText::Font* font){
    hUDStatisticsManager = new HUDStatisticsManager(geode,font,
                                                    15+std::max(statlineprop.fontSizeTime, statlineprop.fontSizeText));
    this->addCallbackable(hUDStatisticsManager->getStatisticTools(), Base::PHYSICS_CALLBACKABLE);
    this->addCallbackable(hUDStatisticsManager, Base::GRAPHICS_CALLBACKABLE);
  }


  HUDStatisticsManager* Base::getHUDSM()
  {
    if (hUDStatisticsManager==0)
      {
        // create HUDStatisticsManager and register it for being called back every step
        // but do not display if the system is initialised with -nographics
        createHUDManager(new osg::Geode(),osgText::readFontFile("fonts/fudd.ttf"));
      }
    return hUDStatisticsManager;
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

  void Base::setCaption(const std::string& caption) {
    this->caption = caption;
    if(captionline){
      captionline->setText(caption);
    }
  }

  void Base::setTitle(const std::string& title) {
    this->title = title;
    if(titleline){
      titleline->setText(title);
    }
  }

  void Base::makeScene(OsgScene* scene, const OsgConfig& config){
    scene->root = new Group; // master node containing scene but also hud and other stuff
    scene->world = new Group;// actual world with sky, ground, scene and so on.
    scene->world_noshadow = new Group;// like world but without shadow
    scene->scene = new Group;// actual scene with robots and stuff
    // the base and sky subgraphs go to set the earth sky of the
    // model and clear the color and depth buffer for us, by using
    // osg::Depth, and setting their bin numbers to less than 0,
    // to force them to draw before the rest of the scene.
    ClearNode* clearNode = new ClearNode;

    // use a transform to make the sky and base around with the eye point.
    scene->worldtransform = new osg::Transform;//MoveEarthySkyWithEyePointTransform;
    // add the transform to the earth sky.
    clearNode->addChild(scene->worldtransform);
    // transform's value isn't known until in the cull traversal so its bounding
    // volume can't be determined, therefore culling will be invalid,
    // so switch it off, this cause all our parents to switch culling
    // off as well. But don't worry culling will be back on once underneath
    // this node or any other branch above this transform.
    scene->worldtransform->setCullingActive(false);
    // add the sky and base layer.
    scene->world->addChild(clearNode);

    // do the same clearnote and transform for world_noshadow
    ClearNode* clearNodeNS = new ClearNode;
    osg::Transform* transformNS = new osg::Transform;
    clearNodeNS->addChild(transformNS);
    transformNS->setCullingActive(false);

    // add the sky and base layer.
    scene->world_noshadow->addChild(clearNodeNS);


    osg::Node* sky = makeSky(config);
    scene->worldtransform->addChild(sky); // bin number -2 so drawn first.
    transformNS->addChild(sky);          // bin number -2 so drawn first.

    scene->groundScene = makeGround(config);
    // add it  to the noshadow world
    // for the shadow we have to distinguish between different modes
    transformNS->addChild(scene->groundScene);


    // scene->lightSource = makeLights(scene->world->getOrCreateStateSet());
    // scene->worldtransform->addChild(scene->lightSource);
    // transformNS->addChild(makeLights(transformNS->getOrCreateStateSet()));
    makeLights(scene->worldtransform, config);
    makeLights(transformNS, config);


    int shadowType=(int)osgHandle.cfg->shadowType;
    if(shadowType){
      // create root of shadowedScene
      scene->shadowedSceneRoot = new osg::Group;
      scene->shadowedSceneRoot->addChild(scene->scene);
      scene->shadowedScene = createShadowedScene(scene->shadowedSceneRoot,
                                                 scene->lightSource,(int)osgHandle.cfg->shadowType);

      // 20090325; guettler: if using pssm (shadowtype 3), add also the ground to the shadowed scene
      if (shadowType==3)
              scene->shadowedSceneRoot->addChild(scene->groundScene); // bin number -1 so draw second.
      else
        scene->worldtransform->addChild(scene->groundScene); // bin number -1 so draw second.


      scene->root->addChild(scene->world);
    }else {
      scene->root->addChild(scene->world_noshadow);
    }

    // add the shadowed scene to the world
    scene->worldtransform->addChild(scene->shadowedScene);
    // scene->world->addChild(scene->shadowedScene);
    // add the normal scene to the root
    //scene->world_noshadow->addChild(scene->scene);
    transformNS->addChild(scene->scene);

    dummy=new osg::Group; // we uses this hack to prevent the nodes from being deleted
    dummy->addChild(scene->world);
    dummy->addChild(scene->world_noshadow);


    hud = createHUD(scene, config);
    if(hud) scene->root->addChild(hud);
  }

  Node* Base::makeSky(const OsgConfig& config) {
    // taken from osghangglider example
    int i, j;
    float lev[] = {-5.0, -1.0, 2.0, 12.0, 30.0, 60.0, 90.0  };
    float x, y, z;
    float alpha, theta;
    float radius = 1000.0f;
    int nlev = sizeof( lev )/sizeof(float);

    Geometry *geom = new Geometry;

    Vec3Array& coords = *(new Vec3Array(19*nlev));
    Vec4Array& colors = *(new Vec4Array(19*nlev));
    Vec2Array& tcoords = *(new Vec2Array(19*nlev));


    Color cc[] =
      { config.cs->color("sky1"),// horizon
        config.cs->color("sky2"),
        config.cs->color("sky3"),
        config.cs->color("sky4"),
        config.cs->color("sky5"),
        config.cs->color("sky6"),
        config.cs->color("sky7"), // zenit
      };

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

            colors[ci] = cc[i];

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

  void Base::makePhysicsScene(){
    // add ODE Ground here (physical plane)
    ground = dCreatePlane ( odeHandle.space , 0 , 0 , 1 , 0 );
    dGeomSetCategoryBits(ground,Primitive::Stat);
    dGeomSetCollideBits(ground,~Primitive::Stat);
    // assign a dummy primitive to the ground plane to have substance (material) support
    plane = new Plane();
    dGeomSetData(ground, (void*)plane);
    //    std::cout << "GROUND: " << ground << std::endl;
  }

  Substance Base::getGroundSubstance(){
    if(plane) return plane->substance;
    else return Substance();
  }

  void Base::setGroundSubstance(const Substance& substance){
    if(plane) plane->setSubstance(substance);
  }


  Node* Base::makeGround(const OsgConfig& config){ // the old ground, is NOT used for shadowing except for shadow mode 3!
    float ir = 1000.0f;
    float texscale =0.2;
    Vec3Array *coords = new Vec3Array(4);
    Vec2Array *tcoords = new Vec2Array(4);
    Vec4Array *colors = new Vec4Array(1);

    (*colors)[0]=config.cs->color("ground");

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
    geode->setNodeMask(geode->getNodeMask() & ~CastsShadowTraversalMask);


    return geode;
  }


  void Base::makeLights(osg::Group* node, const OsgConfig& config)
  {

    // create a default light that is used independently of the shadow. Only ambient and specular
    osg::Light* myLight0 = new osg::Light();
    myLight0->setLightNum(1);
    myLight0->setDiffuse(osg::Vec4(0.0f,0.0f,0.0f,0.0f));
    myLight0->setSpecular(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    //myLight0->setAmbient(osg::Vec4(.0f,.0f,.0f,0.f));
    myLight0->setAmbient(osg::Vec4(.3f,.3f,.3f,1.0f));
    //myLight0->setAmbient(osg::Vec4(1.0,1.0,1.0,1.0f));
    myLight0->setPosition(osg::Vec4(1.0,1.0,1.0,0.0f));
    myLight0->setDirection(osg::Vec3(-1.0, -1.0, -1.0));
    osg::LightSource* lightS0 = new osg::LightSource();
    node->addChild(lightS0);
    lightS0->setLight(myLight0);
    lightS0->setStateSetModes(*(node->getOrCreateStateSet()),osg::StateAttribute::ON);


    // create a directional diffuse light for shadowing (infinite distance place at 45 degrees)
    osg::Light* myLight1 = new osg::Light();
    myLight1->setLightNum(0);
    myLight1->setPosition(osg::Vec4(1.0,1.0,1.0,0.0f));
    myLight1->setDirection(osg::Vec3(-1.0, -1.0, -1.0));
    myLight1->setAmbient(osg::Vec4(0.f,0.f,0.f,0.f));
    myLight1->setSpecular(osg::Vec4(0.f,0.f,0.f,0.f));
    myLight1->setDiffuse(osg::Vec4(1.0f,1.0f,1.0f,1.0f));

    osg::LightSource* lightS1 = new osg::LightSource();
    lightS1->setLight(myLight1);
    // lightS->setLocalStateSetModes(osg::StateAttribute::ON);
    node->addChild(lightS1);
    lightS1->setStateSetModes(*(node->getOrCreateStateSet()),osg::StateAttribute::ON);

   //  //       /*
   //  // // create a spot light.
   //  // Light* light_0 = new Light;
   //  // light_0->setLightNum(0);
   //  // //    light_0->setPosition(Vec4(0.0f, 0.0f, 50.0f, 1.0f));
   //  // light_0->setPosition(Vec4(40.0f, 40.0f, 50.0f, 1.0f));
   //  // //    light_0->setAmbient(Vec4(0.25f, 0.25f, 0.25f, 1.0f));
   //  // light_0->setAmbient(Vec4(0.7f, 0.7f, 0.7f, 1.0f));  // Georg 21.07.2007 changed from 0.5 to 0.7
   //  // //light_0->setAmbient(Vec4(0.9f, 0.9f, 0.9f, 1.0f));  // Georg 05.01.2008 changed from 0.7 to 0.9
   //  // light_0->setDiffuse(Vec4(0.8f, 0.8f, 0.8f, 1.0f));
   //  // //    light_0->setDirection(Vec3(-1.0f, -1.0f, -1.2f));
   //  // light_0->setSpecular(Vec4(1.0f, 0.9f, 0.8f, 1.0f));

   //  // LightSource* light_source_0 = new LightSource;
   //  // light_source_0->setLight(light_0);
   //  // light_source_0->setLocalStateSetModes(StateAttribute::ON);
   //  // light_source_0->setStateSetModes(*stateset, StateAttribute::ON);

   //  // return light_source_0;
   //  // */


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


  void Base::changeShadowTechnique()
  {
    OsgScene* scene = osgHandle.scene;
    std::string shadowName;
    int shadowType = ++(osgHandle.cfg->shadowType);
    switch (shadowType) {
    case 6:
      shadowType=0; // max shadowtype at the moment: 5
    case 0:
      scene->root->removeChild(scene->world);
      scene->root->addChild(scene->world_noshadow);
      shadowName = std::string("NoShadow");
      break;
    case 1:
    case 2:
      shadowType=3; // temporarily disable volume shadows (1) and ShadowTextue (2)
    case 3:
      scene->root->removeChild(scene->world_noshadow);
      scene->root->addChild(scene->world);
      if(scene->shadowedScene) {
        scene->world->removeChild(scene->shadowedScene);
      }
      scene->shadowedSceneRoot = new osg::Group;
      scene->shadowedSceneRoot->addChild(scene->groundScene);
      scene->shadowedSceneRoot->addChild(scene->scene);

      scene->shadowedScene = createShadowedScene(scene->shadowedSceneRoot,
                                                 scene->lightSource,
                                                 shadowType);
      scene->world->addChild(scene->shadowedScene);
      // 20090325; guettler: if using pssm (shadowtype 3), add also the ground to the shadowed scene
      scene->worldtransform->removeChild(scene->groundScene);
      shadowName = std::string("ParallelSplitShadowMap");
      break;
    case 4:
      scene->world->removeChild(scene->shadowedScene);
      scene->shadowedScene = createShadowedScene(scene->scene,scene->lightSource, shadowType);
      scene->world->addChild(scene->shadowedScene);
      scene->worldtransform->addChild(scene->groundScene); // bin number -1 so draw second.
      scene->shadowedSceneRoot->removeChild(scene->groundScene);
      shadowName = std::string("SoftShadowMap");
      break;
    case 5:
      scene->world->removeChild(scene->shadowedScene);
      scene->shadowedScene = createShadowedScene(scene->scene,scene->lightSource, shadowType);
      // add the shadowed scene to the root
      scene->world->addChild(scene->shadowedScene);
      shadowName = std::string("ShadowMap (simple)");
      break;
    default:
      shadowName = std::string("NoShadow");
      break;
    }
    printf("Changed shadowType to %i (%s)\n",shadowType,shadowName.c_str());
    osgHandle.cfg->shadowType=shadowType;
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

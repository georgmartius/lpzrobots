/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield 
 *
 * This application is open source and may be redistributed and/or modified   
 * freely and without restriction, both in commericial and non commericial applications,
 * as long as this copyright notice is maintained.
 * 
 * This application is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 g++ -losgShadow -losgText -losgUtil -losgViewer -losgGA -lOpenThreads -losg -lGL -lGLU -lglut test_georg.cpp

*/

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>

#include <osg/Switch>
#include <osgText/Text>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>

#include <iostream>
#include <sstream>
#include <string.h>

class WindowCaptureCallback : public osg::Camera::DrawCallback
{
    public:
        WindowCaptureCallback()
        {
        }

        virtual void operator () (osg::RenderInfo& renderInfo) const
        {
            //printf("hello from pbo\n");
        }
};

int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");

    osgViewer::Viewer viewer(arguments);
    viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);

    unsigned int helpType = 0;
    // if ((helpType = arguments.readHelpType()))
//     {
//         arguments.getApplicationUsage()->write(std::cout, helpType);
//         return 1;
//     }
    
    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }
    
    if (arguments.argc()<=1)
    {
        arguments.getApplicationUsage()->write(std::cout,osg::ApplicationUsage::COMMAND_LINE_OPTION);
        return 1;
    }

    // set up the camera manipulators.
    {
        osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

        keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
        keyswitchManipulator->addMatrixManipulator( '2', "Flight", new osgGA::FlightManipulator() );
        keyswitchManipulator->addMatrixManipulator( '3', "Drive", new osgGA::DriveManipulator() );
        keyswitchManipulator->addMatrixManipulator( '4', "Terrain", new osgGA::TerrainManipulator() );

        std::string pathfile;
        char keyForAnimationPath = '5';
        while (arguments.read("-p",pathfile))
        {
            osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
            if (apm || !apm->valid()) 
            {
                unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
                keyswitchManipulator->addMatrixManipulator( keyForAnimationPath, "Path", apm );
                keyswitchManipulator->selectMatrixManipulator(num);
                ++keyForAnimationPath;
            }
        }

        viewer.setCameraManipulator( keyswitchManipulator.get() );
    }

    // add the state manipulator
    viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
    
    // add the thread model handler
    viewer.addEventHandler(new osgViewer::ThreadingHandler);

    // add the window size toggle handler
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);
        
    // add the stats handler
    viewer.addEventHandler(new osgViewer::StatsHandler);

    // add the help handler
    viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));

    // add the record camera path handler
    viewer.addEventHandler(new osgViewer::RecordCameraPathHandler);

    // add the LOD Scale handler
    viewer.addEventHandler(new osgViewer::LODScaleHandler);
    
    unsigned int width=512;
    unsigned int height=512;

    osg::ref_ptr<osg::GraphicsContext> pbuffer;
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = width;
    traits->height = height;
    traits->red = 8;
    traits->green = 8;
    traits->blue = 8;
    traits->alpha = 8;
    traits->windowDecoration = false;
    traits->pbuffer = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;

    pbuffer = osg::GraphicsContext::createGraphicsContext(traits.get());
    if (pbuffer.valid())
    {
        osg::notify(osg::NOTICE)<<"Pixel buffer has been created successfully."<<std::endl;
    }
    else
    {
        osg::notify(osg::NOTICE)<<"Pixel buffer has not been created successfully."<<std::endl;
        return 1;
    }
        
    // load the data
    osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFiles(arguments);
    if (!loadedModel) 
    {
        std::cout << arguments.getApplicationName() <<": No data loaded" << std::endl;
        return 1;
    }
    // load the data
    osg::ref_ptr<osg::Node> loadedModel2 = osgDB::readNodeFile(std::string("cow.osg"));
    if (!loadedModel2) 
    {
        std::cout << "ERROR: No data loaded" << std::endl;
        return 1;
    }

    osg::Group* dummy = new osg::Group();
    osg::Group* root = new osg::Group();

    osg::Group* scene = new osg::Group();
    osg::Group* hidden = new osg::Group();
    root->addChild(scene);
    scene->addChild(loadedModel.get());
    //hidden->addChild(loadedModel2.get());

    dummy->addChild(scene);
    dummy->addChild(hidden);

    // any option left unread are converted into errors to write out later.
    arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
      {
        arguments.writeErrorMessages(std::cout);
        return 1;
      }

    // optimize the scene graph, remove redundant nodes and state etc.
    osgUtil::Optimizer optimizer;
    optimizer.optimize(loadedModel.get());

    viewer.setSceneData(root);

    osg::ref_ptr<osg::Camera> pbo_camera;
    dummy->addChild(pbo_camera.get());
    pbo_camera = new osg::Camera;
    pbo_camera->setGraphicsContext(pbuffer.get());
    pbo_camera->setViewport(new osg::Viewport(0,0,width,height));
    GLenum buffer = pbuffer->getTraits()->doubleBuffer ? GL_BACK : GL_FRONT;
    pbo_camera->setDrawBuffer(buffer);
    pbo_camera->setReadBuffer(buffer);
    pbo_camera->setFinalDrawCallback(new WindowCaptureCallback());
    
    
    // Create the texture to render to
    osg::Texture2D* texture = new osg::Texture2D;
    texture->setTextureSize(256, 256);
    texture->setInternalFormat(GL_RGBA);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

    // set up the render to texture camera.
    osg::Camera* cam = new osg::Camera;
    cam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // set up projection.
    cam->setProjectionMatrixAsPerspective(45, 1,0.1,30);    
    // set view
    cam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    cam->setViewport(0, 0, 256, 256);
    // Frame buffer objects are the best option
    cam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);    
    // We need to render to the texture BEFORE we render to the screen
    cam->setRenderOrder(osg::Camera::PRE_RENDER);    
    // The camera will render into the texture that we created earlier
    cam->attach(osg::Camera::COLOR_BUFFER, texture);
    // Add world to be drawn to the texture
    cam->addChild(loadedModel2.get());
    hidden->addChild(cam);
    root->addChild(hidden);
  
    osg::ref_ptr<osg::Geometry> screenQuad;
    screenQuad = osg::createTexturedQuadGeometry(osg::Vec3(),
                                                 osg::Vec3(256, 0.0, 0.0),
                                                 osg::Vec3(0.0, 256, 0.0));
    osg::ref_ptr<osg::Geode> quadGeode = new osg::Geode;
    quadGeode->addDrawable(screenQuad.get());
    osg::StateSet *quadState = quadGeode->getOrCreateStateSet();
    quadState->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    quadState->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);    
    osg::ref_ptr<osg::Camera> orthoCamera = new osg::Camera;
    // We don't want to apply perspective, just overlay using orthographic
    orthoCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, 256, 0, 256));    
    orthoCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    orthoCamera->setViewMatrix(osg::Matrix::identity());
        
    orthoCamera->setViewport(0, 0, 256,256);      
    orthoCamera->setRenderOrder(osg::Camera::POST_RENDER);
    orthoCamera->addChild(quadGeode.get());
    scene->addChild(orthoCamera.get());    

    viewer.realize();
    pbuffer->realize();

    osg::Camera* orig_camera = viewer.getCamera();
    dummy->addChild(orig_camera);

    viewer.frame();
    root->removeChild(hidden);
    osg::Vec3 eye; osg::Vec3 center; osg::Vec3 up; 
    viewer.getCamera()->getViewMatrixAsLookAt(eye,center,up);
    cam->setViewMatrixAsLookAt(eye, center, up);           
    int frame_count=0;
    const int swap_every=3;
    while(!viewer.done())
      {
        if (0 == (frame_count % swap_every)) {
	  viewer.setCamera(pbo_camera.get());
	  orig_camera->setRenderer(0);
	  root->removeChild(scene);
	  root->addChild(hidden);
	  printf("%u\n",root->getNumChildren());
          viewer.frame();

	  viewer.setCamera(orig_camera);
 	  pbo_camera->setRenderer(0);
	  root->removeChild(hidden);
	  root->addChild(scene);
          
	}else{
	  viewer.frame();
	}
	frame_count++;      
      }

    return 0;
}

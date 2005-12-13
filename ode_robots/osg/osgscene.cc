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
 *   $Log$
 *   Revision 1.1.2.3  2005-12-13 18:11:13  martius
 *   transform primitive added, some joints stuff done, forward declaration
 *
 *   Revision 1.1.2.2  2005/12/09 16:54:16  martius
 *   camera is woring now
 *
 *   Revision 1.1.2.1  2005/12/06 10:13:25  martius
 *   openscenegraph integration started
 *
 *
 ***************************************************************************/

#include <osgProducer/Viewer>

#include <osg/Projection>
#include <osg/Geometry>
#include <osg/Texture>
#include <osg/TexGen>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PolygonOffset>
#include <osg/MatrixTransform>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/Material>

#include <osgUtil/TransformCallback>

#include "osgprimitive.h"

using namespace osg;
using namespace lpzrobots;


ref_ptr<Group> _create_scene()
{

  ref_ptr<Group> scene = new Group;

  ref_ptr<Geode> geode_1 = new Geode;
  scene->addChild(geode_1.get());

  ref_ptr<Geode> geode_2 = new Geode;
  ref_ptr<MatrixTransform> transform_2 = new MatrixTransform;
  transform_2->addChild(geode_2.get());
  transform_2->setUpdateCallback(new osgUtil::TransformCallback(Vec3(0, 0, 0), Y_AXIS, inDegrees(45.0f)));
  scene->addChild(transform_2.get());

  const float radius = 0.8f;
  const float height = 1.0f;
  ref_ptr<TessellationHints> hints = new TessellationHints;
  hints->setDetailRatio(2.0f);
  ref_ptr<ShapeDrawable> shape;

  shape = new ShapeDrawable(new Box(Vec3(0.0f, -2.0f, 0.0f), 10, 0.1f, 10), hints.get());
  shape->setColor(Vec4(0.5f, 0.5f, 0.7f, 1.0f));
  geode_1->addDrawable(shape.get());

  shape = new ShapeDrawable(new Sphere(Vec3(0.0f, 0.0f, 0.0f), radius * 2), hints.get());
  shape->setColor(Vec4(0.8f, 0.8f, 0.8f, 1.0f));
  geode_1->addDrawable(shape.get());

  shape = new ShapeDrawable(new Sphere(Vec3(-3.0f, 0.0f, 0.0f), radius), hints.get());
  shape->setColor(Vec4(0.6f, 0.8f, 0.8f, 1.0f));
  geode_2->addDrawable(shape.get());

  shape = new ShapeDrawable(new Box(Vec3(3.0f, 0.0f, 0.0f), 2 * radius), hints.get());
  shape->setColor(Vec4(0.4f, 0.9f, 0.3f, 1.0f));
  geode_2->addDrawable(shape.get());

  shape = new ShapeDrawable(new Cone(Vec3(0.0f, 0.0f, -3.0f), radius, height), hints.get());
  shape->setColor(Vec4(0.2f, 0.5f, 0.7f, 1.0f));
  geode_2->addDrawable(shape.get());

  shape = new ShapeDrawable(new Cylinder(Vec3(0.0f, 0.0f, 3.0f), radius, height), hints.get());
  shape->setColor(Vec4(1.0f, 0.3f, 0.3f, 1.0f));
  geode_2->addDrawable(shape.get());

  OsgHandle osgHandle(scene.get(), hints.get(), Color(1.0f, 0.0f, 0.0f, 1.0f) );

  Matrix m;

  OSGPlane* plane = new OSGPlane();
  plane->init(osgHandle);
  m.makeTranslate(Vec3(0.0f,0.0f,-1.0f));
  plane->setMatrix(m);

  OSGBox* box = new OSGBox(2,3,4);
  box->init(osgHandle);
  m.makeTranslate(Vec3(1.0f,2.0f,0.0f));
  box->setMatrix(m);

  OSGSphere* sphere= new OSGSphere(2);
  sphere->init(osgHandle);
  m.makeTranslate(Vec3(-1.0f,1.0f,0.0f));
  
  sphere->setMatrix(m);

  // material
  ref_ptr<Material> matirial = new Material;
  matirial->setColorMode(Material::DIFFUSE);
  matirial->setAmbient(Material::FRONT_AND_BACK, Vec4(0, 0, 0, 1));
  matirial->setSpecular(Material::FRONT_AND_BACK, Vec4(1, 1, 1, 1));
  matirial->setShininess(Material::FRONT_AND_BACK, 64.0f);
  scene->getOrCreateStateSet()->setAttributeAndModes(matirial.get(), StateAttribute::ON);

  return scene;
}

int main(int argc, char** argv)
{
  // use an ArgumentParser object to manage the program arguments.
  ArgumentParser arguments(&argc, argv);

  // set up the usage document, in case we need to print out how to use this program.
  arguments.getApplicationUsage()->setDescription(arguments.getApplicationName() + 
     " is the example which demonstrates using of GL_ARB_shadow extension implemented in osg::Texture class");
  arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName());
  arguments.getApplicationUsage()->addCommandLineOption("-h or --help", "Display this information");
   
  // construct the viewer.
  osgProducer::Viewer viewer(arguments);

  // set up the value with sensible default event handlers.
  viewer.setUpViewer(osgProducer::Viewer::STANDARD_SETTINGS);
    
  // get details on keyboard and mouse bindings used by the viewer.
  viewer.getUsage(*arguments. getApplicationUsage());

  // if user request help write it out to cout.
  if (arguments.read("-h") || arguments.read("--help")) {
    arguments.getApplicationUsage()->write(std::cout);
    return 1;
  }

  // any option left unread are converted into errors to write out later.
  arguments.reportRemainingOptionsAsUnrecognized();

  // report any errors if they have occured when parsing the program aguments.
  if (arguments.errors()) {
    arguments.writeErrorMessages(std::cout);
    return 1;
  }

  ref_ptr<MatrixTransform> scene = new MatrixTransform;
  scene->setMatrix(osg::Matrix::rotate(osg::DegreesToRadians(125.0),1.0,0.0,0.0));
  ref_ptr<Group> main_scene = _create_scene();    
  if (!main_scene.valid()) return 1;
  scene->addChild(main_scene.get());

  // add model to viewer.
  viewer.setSceneData(scene.get()); 

  // create the windows and run the threads.
  viewer.realize();

  while (!viewer.done())
    {
      // wait for all cull and draw threads to complete.
      viewer.sync();

      // update the scene by traversing it with the the update visitor which will
      // call all node update callbacks and animations.
      viewer.update();
      
      // fire off the cull and draw traversals of the scene.
      viewer.frame();
    }
    
  // wait for all cull and draw threads to complete before exit.
  viewer.sync();

  return 0;
}

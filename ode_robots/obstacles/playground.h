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
 *   $Log$
 *   Revision 1.10.4.14  2006-07-11 12:43:39  robot5
 *   Collision model of playground walls corrected.
 *
 *   Revision 1.10.4.13  2006/07/11 04:24:19  robot5
 *   Walls now display repeating textures.
 *
 *   Revision 1.10.4.12  2006/06/29 16:39:55  robot3
 *   -you can now see bounding shapes if you type ./start -drawboundings
 *   -includes cleared up
 *   -abstractobstacle and abstractground have now .cpp-files
 *
 *   Revision 1.10.4.11  2006/05/23 13:37:34  robot3
 *   -fixed some creating bugs
 *   -setColor,setTexture and createGround must be
 *    called before setPosition now
 *
 *   Revision 1.10.4.10  2006/05/19 08:42:54  robot3
 *   -some code moved to abstractground.h
 *   -it's now possible creating a playground without a groundplane
 *
 *   Revision 1.10.4.9  2006/05/18 14:32:12  robot3
 *   walls are now textured with a wood texture
 *
 *   Revision 1.10.4.8  2006/05/18 12:54:24  robot3
 *   -fixed not being able to change the color after positioning
 *    the obstacle
 *   -cleared the files up
 *
 *   Revision 1.10.4.7  2006/05/18 09:43:24  robot3
 *   using existing texture image in cvs for the groundplane now
 *
 *   Revision 1.10.4.6  2006/05/18 07:42:36  robot3
 *   Grounds have now a groundPlane for shadowing issues
 *   osgprimitive.cpp contains a bug that repeating textures (wrapping)
 *   don't work, needs to be fixed
 *
 *   Revision 1.10.4.5  2006/05/11 08:59:15  robot3
 *   -fixed a positioning bug (e.g. for passivesphere)
 *   -some methods moved to abstractobstacle.h for avoiding inconsistencies
 *
 *   Revision 1.10.4.4  2006/03/29 15:04:39  martius
 *   have pose now
 *
 *   Revision 1.10.4.3  2006/01/10 20:27:15  martius
 *   protected members
 *
 *   Revision 1.10.4.2  2006/01/10 17:17:33  martius
 *   new mode for primitives
 *
 *   Revision 1.10.4.1  2005/12/06 10:13:23  martius
 *   openscenegraph integration started
 *
 *   Revision 1.10  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.9  2005/09/13 13:19:57  martius
 *   no texture
 *
 *   Revision 1.8  2005/08/02 14:09:06  fhesse
 *   factor between length in x and y direction
 *   added to constructor
 *
 *   Revision 1.7  2005/07/29 14:27:59  martius
 *   color set to some red
 *
 *   Revision 1.6  2005/07/18 14:52:33  martius
 *   world and space are not pointers anymore.
 *
 *   Revision 1.5  2005/07/07 10:24:23  martius
 *   avoid internal collisions
 *
 *   Revision 1.4  2005/06/15 14:22:11  martius
 *   GPL included
 *                                                                 *
 ***************************************************************************/
#ifndef __PLAYGROUND_H
#define __PLAYGROUND_H

#include "mathutils.h"
#include "abstractground.h"

#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/TexEnv>

namespace lpzrobots {

//Fixme: playground creates collisions with ground and itself
//fixed: collisions with ground
class Playground : public AbstractGround {

protected:

  double length, width, height;
  double factorlength2;

public:

  Playground(const OdeHandle& odeHandle, const OsgHandle& osgHandle ,
	     const osg::Vec3& dimension = osg::Vec3(7.0, 0.2, 0.5) ,
	     double factorxy = 1, bool createGround=true):
    AbstractGround::AbstractGround(odeHandle, osgHandle, createGround){

    length=dimension.x();
    width=dimension.y();
    height=dimension.z();
    factorlength2=factorxy;

  };

 protected:
  void realPlane(float x1, float y1, float z1,
                 float x2, float y2, float z2,
                 float x3, float y3, float z3,
                 float x4, float y4, float z4,
                 float tlength, float theight) {
   osg::Vec3Array *coords = new osg::Vec3Array(4);
   (*coords)[0].set(x1,y1,z1);
   (*coords)[1].set(x2,y2,z2);
   (*coords)[2].set(x3,y3,z3);
   (*coords)[3].set(x4,y4,z4);

   osg::Vec2Array *tcoords = new osg::Vec2Array(4);
   (*tcoords)[0].set(   0.0f,theight);
   (*tcoords)[1].set(   0.0f,   0.0f);
   (*tcoords)[2].set(tlength,theight);
   (*tcoords)[3].set(tlength,   0.0f);

   osg::Geometry *geom = new osg::Geometry;
   geom->setVertexArray(coords);
   geom->setTexCoordArray(0,tcoords);
   geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP,0,4));

   osg::Texture2D *tex = new osg::Texture2D;
   tex->setImage(osgDB::readImageFile(wallTextureFileName));
   tex->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::REPEAT);
   tex->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::REPEAT);
   osg::StateSet *dstate = new osg::StateSet;
   dstate->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
   dstate->setTextureAttributeAndModes(0,tex,osg::StateAttribute::ON);
   dstate->setTextureAttribute(0,new osg::TexEnv);
   geom->setStateSet(dstate);
   osg::Geode *geode = new osg::Geode;
   geode->addDrawable(geom);
   osgHandle.scene->addChild(geode);
  }

  virtual void create() {
   const float w=width;
   const float h=height;
   //inner walls
   float x1=-0.5f*length, y1=-0.5f*length;
   float x2= 0.5f*length, y2=-0.5f*length;
   float x3= 0.5f*length, y3= 0.5f*length;
   float x4=-0.5f*length, y4= 0.5f*length;
   realPlane(x1-w,y1,0.0f,
             x1-w,y1,   h,
             x2+w,y2,0.0f,
             x2+w,y2,   h,
             length+w,h);
   realPlane(x2,y2-w,0.0f,
             x2,y2-w,   h,
             x3,y3+w,0.0f,
             x3,y3+w,   h,
             length+w,h);
   realPlane(x3+w,y3,0.0f,
             x3+w,y3,   h,
             x4-w,y4,0.0f,
             x4-w,y4,   h,
             length+w,h);
   realPlane(x4,y4+w,0.0f,
             x4,y4+w,   h,
             x1,y1-w,0.0f,
             x1,y1-w,   h,
             length+w,h);

   //outer walls
   x1-=w; y1-=w;
   x2+=w; y2-=w;
   x3+=w; y3+=w;
   x4-=w; y4+=w;
   realPlane(x1,y1,0.0f,
             x1,y1,   h,
             x2,y2,0.0f,
             x2,y2,   h,
             length+w,h);
   realPlane(x2,y2,0.0f,
             x2,y2,   h,
             x3,y3,0.0f,
             x3,y3,   h,
             length+w,h);
   realPlane(x3,y3,0.0f,
             x3,y3,   h,
             x4,y4,0.0f,
             x4,y4,   h,
             length+w,h);
   realPlane(x4,y4,0.0f,
             x4,y4,   h,
             x1,y1,0.0f,
             x1,y1,   h,
             length+w,h);

   //top of walls
   realPlane(x1,  y1,h+0.0001f,
             x1,y1+w,h+0.0001f,
             x2,  y2,h+0.0001f,
             x2,y2+w,h+0.0001f,
             length+w,0.4f*w);
   realPlane(  x2,y2,h,
             x2-w,y2,h,
               x3,y3,h,
             x3-w,y3,h,
             length+w,0.4f*w);
   realPlane(x3,  y3,h+0.0001f,
             x3,y3-w,h+0.0001f,
             x4,  y4,h+0.0001f,
             x4,y4-w,h+0.0001f,
             length+w,0.4f*w);
   realPlane(  x4,y4,h,
             x4+w,y4,h,
               x1,y1,h,
             x1+w,y1,h,
             length+w,0.4f*w);


    Box* box;
    osg::Vec3 offset(- (length/2 + width/2),
		     0,
		     height/2+0.01f); //reduces graphic errors and ode collisions
    box = new Box( width , (length * factorlength2) + 2 * width , height);
    box->init(odeHandle, 0, osgHandle, Primitive::Geom);
    box->setPose(osg::Matrix::translate(offset) * pose);
    //box->getOSGPrimitive()->setTexture(wallTextureFileName);
    obst.push_back(box);

    offset.x() = length/2 + width/2;
    box = new Box( width , (length * factorlength2) + 2 * width , height);
    box->init(odeHandle, 0, osgHandle, Primitive::Geom);
    box->setPose(osg::Matrix::translate(offset) * pose);
    //box->getOSGPrimitive()->setTexture(wallTextureFileName);
    obst.push_back(box);

    offset.x() = 0;
    offset.y() = -( (length*factorlength2)/2 +width/2);
    box = new Box( length, width, height);
    box->init(odeHandle, 0, osgHandle, Primitive::Geom);
    box->setPose(osg::Matrix::translate(offset) * pose);
    //box->getOSGPrimitive()->setTexture(wallTextureFileName);
    obst.push_back(box);

    offset.y() = (length*factorlength2)/2 +width/2;
    box = new Box( length, width, height);
    box->init(odeHandle, 0, osgHandle, Primitive::Geom);
    box->setPose(osg::Matrix::translate(offset) * pose);
    //box->getOSGPrimitive()->setTexture(wallTextureFileName);
    obst.push_back(box);

    // size of groundplane
    ground_length=length;

    obstacle_exists=true;
  };

};

}

#endif

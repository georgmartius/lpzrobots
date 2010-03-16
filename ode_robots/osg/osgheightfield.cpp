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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *                                                                         *
 *   $Log$
 *   Revision 1.8  2010-03-16 15:47:46  martius
 *   osgHandle has now substructures osgConfig and osgScene
 *    that minimized amount of redundant data (this causes a lot of changes)
 *   Scenegraph is slightly changed. There is a world and a world_noshadow now.
 *    Main idea is to have a world without shadow all the time avaiable for the
 *    Robot cameras (since they do not see the right shadow for some reason)
 *   tidied up old files
 *
 *   Revision 1.7  2009/07/30 11:36:01  guettler
 *   added check if noGraphics in OsgHandle is set
 *
 *   Revision 1.6  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.5  2008/05/07 16:45:51  martius
 *   code cosmetics and documentation
 *
 *   Revision 1.4  2006/09/20 12:55:44  martius
 *   Light
 *
 *   Revision 1.3  2006/08/04 15:05:43  martius
 *   documentation
 *
 *   Revision 1.2  2006/07/14 12:23:35  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.3  2006/06/23 09:04:48  robot3
 *   added #include <assert.h>
 *
 *   Revision 1.1.2.2  2006/06/09 15:43:49  fhesse
 *   include assert.h added
 *
 *   Revision 1.1.2.1  2006/05/28 22:14:56  martius
 *   heightfield included
 *
 *   Revision 1.1.2.1  2005/12/06 17:38:21  martius
 *   *** empty log message ***
 *
 *                                                                 *
 ***************************************************************************/

#include "osgheightfield.h"

#include <string>
#include <iostream>
#include <assert.h>

#include <osg/Texture2D>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osgDB/FileUtils>
#include <osg/Material>
// #include <osg/Geode>
// #include <osgDB/ReadFile>
// #include <osg/Texture>
// #include <osg/TexGen>
// #include <osg/PolygonOffset>
// #include <osg/Light>
// #include <osg/LightSource>
// #include <osg/TexEnv>

#include "imageppm.h"

namespace lpzrobots {

  using namespace osg;

  // returns a material with the given color (defined in osgprimitive.cpp)
  ref_ptr<Material> getMaterial (const Color& c, Material::ColorMode mode = Material::DIFFUSE );



  /******************************************************************************/
  OSGHeightField::OSGHeightField(osg::HeightField* heightfield,float x_size, float y_size) 
    : field(heightfield), x_size(x_size), y_size(y_size)
  {
    int cols = field->getNumColumns();
    int rows = field->getNumRows();
    field->setXInterval(x_size/(float)(cols-1));
    field->setYInterval(y_size/(float)(rows-1));
  }

  OSGHeightField::OSGHeightField(const std::string& filename, 
				 float x_size, float y_size, float height)
    : x_size(x_size), y_size(y_size) {
    field = osgDB::readHeightFieldFile(filename);
    if(!field){
      std::cerr << "could not open HeigthFieldFile: " << filename << std::endl;
      exit(1);
    }
    int cols = field->getNumColumns();
    int rows = field->getNumRows();
    field->setXInterval(x_size/(float)(cols-1));
    field->setYInterval(y_size/(float)(rows-1));
    // scale the height // Todo: find out maximum, currently 1 is assumed
    for(int i=0; i< rows; i++){
      for(int j=0; j< cols; j++){
	field->setHeight(i,j, field->getHeight(i,j) * height);  
      }
    }
  }

  // overloaded, because transformation goes into heightfield directly
  void OSGHeightField::setMatrix(const osg::Matrix& m4x4){    
    assert(field);
    field->setOrigin(m4x4.getTrans()-Vec3(x_size/2.0, y_size/2.0,0 ));
    Quat q;
    m4x4.get(q);
    field->setRotation(q);
  }


  void OSGHeightField::init(const OsgHandle& _osgHandle, Quality quality){
    osgHandle=_osgHandle;
    assert(osgHandle.parent || osgHandle.cfg->noGraphics);
    transform = new MatrixTransform;
    if (osgHandle.cfg->noGraphics)
      return;
    geode = new Geode;  
    transform->addChild(geode.get());
    osgHandle.parent->addChild(transform.get());

    shape = new ShapeDrawable(field, osgHandle.cfg->tesselhints[quality]);
    shape->setColor(osgHandle.color);
    geode->addDrawable(shape.get());
    if(osgHandle.color.alpha() < 1.0){
      shape->setStateSet(osgHandle.cfg->transparentState);
    }else{
      shape->setStateSet(osgHandle.cfg->normalState);
    }
    shape->getOrCreateStateSet()->setAttributeAndModes(getMaterial(osgHandle.color).get(), 
						       StateAttribute::ON);

    applyTextures();
  }

  double OSGHeightField::coding(CodingMode mode, const unsigned char* data){
    switch(mode){
    case Red:
      return (data[0])/256.0;
      break;
    case Sum:
      return (data[0] + data[1] + data[2])/(3*256.0);
      break;
    case LowMidHigh:
      return ((long(data[0])  << 16) + (long(data[1]) << 8) + data[2])/65536.0;
      break;
    default:
      return 0;
    }
  }

  
  HeightField* OSGHeightField::loadFromPPM(const std::string& filename, double height, CodingMode codingMode){
    HeightField* field = new HeightField();
    ImagePPM image;
    std::string filenamepath = osgDB::findDataFile(filename);
    if(!image.loadImage(filenamepath.c_str())) {
      std::cerr << "could not open PPM image file: '" << filename << "'" << std::endl;
      exit(1);
    }
    int cols = image.width();
    int rows = image.height();

    field->allocate(cols, rows);    
    
    // copy and convert the image from RGB chars to double heights
    unsigned char* data = image.data();
    for(int j=0; j< cols; j++){
      for(int i=0; i< rows; i++){
	// use the coding th get the height and scale it to height
	field->setHeight(i,j, coding(codingMode, data) * height);  
	data+=3;
      }
    }
    return field;
  }



}

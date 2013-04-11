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

#include "heightfieldprimitive.h"

#include "pos.h"
#include <string>
#include <iostream>
#include <assert.h>
#include <cstdio>

#include "odehandle.h"

namespace lpzrobots {

  struct Indices {
    Indices(){}
    Indices(dTriIndex a, dTriIndex b, dTriIndex c) { i[0] = a; i[1] = b; i[2] = c; }
    Indices(dTriIndex d[3]) { i[0] = d[0]; i[1] = d[1]; i[2] = d[2]; }

    dTriIndex i[3];
  };


  struct Vertex {
    Vertex(){}
    Vertex(const osg::Vec3& vec) { v[0] = vec.x(); v[1] = vec.y(); v[2] = vec.z();}

    dVector3 v;  // 4th component can be left out, reducing memory usage
  };



  /******************************************************************************/

  HeightField::HeightField(const std::string& filename, float x_size, float y_size, float height) {
    osgheightfield = new OSGHeightField(filename, x_size, y_size, height);
    data=0;
  }

  HeightField::HeightField(osg::HeightField* heightfield, float x_size, float y_size){
    osgheightfield = new OSGHeightField(heightfield, x_size, y_size);
    data=0;
  }

  HeightField::~HeightField(){
    if(data) dGeomTriMeshDataDestroy (data);
    if(osgheightfield) delete osgheightfield;
  }

  void HeightField::init(const OdeHandle& odeHandle, double mass, const OsgHandle& osgHandle,
                     char mode) {
    assert(mode & Geom);
    substance = odeHandle.substance;
    this->mode=mode;
    if (mode & Draw){
      osgheightfield->init(osgHandle);
    }
    if (mode & Geom){
      const osg::HeightField* f = osgheightfield->getHeightField();
      int cols = f->getNumColumns();
      int rows = f->getNumRows();
      Vertex* vertices = new Vertex[cols*rows];
      //      Vertex* normales = new Vertex[cols*rows];
      Indices* indices = new Indices[(cols-1)*(rows-1)*2];

      for(int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
          vertices[i*cols+j] = Vertex(f->getVertex(j,i));
          //  normales[i*cols+j] = Vertex(f->getNormal(i,j));
        }
      }
      int k=0;
      for(int i=0; i<rows-1; i++){
        for(int j=0; j<cols-1; j++){
          indices[k] = Indices(i*cols+j, i*cols+j+1, (i+1)*cols+j);
          k++;
          indices[k] = Indices(i*cols+j+1, (i+1)*cols+j+1, (i+1)*cols+j);
          k++;
        }
      }
      assert(k==(cols-1)*(rows-1)*2);
      data = dGeomTriMeshDataCreate();
      //      dGeomTriMeshDataBuildDouble1 (data, vertices, sizeof(osg::Vec3f) , rows*cols,
      //                                    indices, k, sizeof(Indices), normales);
      dGeomTriMeshDataBuildSimple (data, (dReal*)vertices, rows*cols, (dTriIndex*)indices, k*3);
      geom = dCreateTriMesh (odeHandle.space, data, 0, 0, 0);
      dGeomSetData(geom, (void*)this); // set primitive as geom data


// #define VertexCount 5
// #define IndexCount 12

// static dVector3 Size;
// static dVector3 Vertices[VertexCount];
// static int Is[IndexCount];



//   Size[0] = 25.0f;
//   Size[1] = 25.0f;
//   Size[2] = 2.5f;

//   Vertices[0][0] = -Size[0];
//   Vertices[0][1] = -Size[1];
//   Vertices[0][2] = Size[2];

//   Vertices[1][0] = Size[0];
//   Vertices[1][1] = -Size[1];
//   Vertices[1][2] = Size[2];

//   Vertices[2][0] = Size[0];
//   Vertices[2][1] = Size[1];
//   Vertices[2][2] = Size[2];

//   Vertices[3][0] = -Size[0];
//   Vertices[3][1] = Size[1];
//   Vertices[3][2] = Size[2];

//   Vertices[4][0] = 0;
//   Vertices[4][1] = 0;
//   Vertices[4][2] = 0;

//   Is[0] = 0;
//   Is[1] = 1;
//   Is[2] = 4;

//   Is[3] = 1;
//   Is[4] = 2;
//   Is[5] = 4;

//   Is[6] = 2;
//   Is[7] = 3;
//   Is[8] = 4;

//   Is[9] = 3;
//   Is[10] = 0;
//   Is[11] = 4;

//  data = dGeomTriMeshDataCreate();

//   dGeomTriMeshDataBuildSimple(data, (dReal*)Vertices, VertexCount, Is, IndexCount);

//       geom = dCreateTriMesh (odeHandle.space, data, 0, 0, 0);



    }
  }

  void HeightField::update(){
    if(mode & Draw) {
      osgheightfield->setMatrix(osgPose(geom));
    }
  }

  void HeightField::setPose(const osg::Matrix& pose){
    osgheightfield->setMatrix(pose);
    // todo : update ode stuff!
  }


}

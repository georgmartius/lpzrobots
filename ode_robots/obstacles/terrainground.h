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
 *   Revision 1.6  2005-09-22 13:17:11  martius
 *   OdeHandle and GlobalData finished
 *   doInternalStuff included
 *
 *   Revision 1.5  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.4  2005/09/13 13:19:32  martius
 *   if image not found handled graciously
 *
 *   Revision 1.3  2005/09/12 14:32:08  martius
 *   use new texture interface
 *
 *   Revision 1.2  2005/09/08 14:28:25  robot2
 *   *** empty log message ***
 *
 *   Revision 1.1  2005/08/26 09:22:23  robot2
 *   terrain
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
#ifndef __TERRAINGROUND_H
#define __TERRAINGROUND_H

#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "abstractobstacle.h"
#include <drawstuff/drawstuff.h>
#include "imageppm.h"

//Fixme: Terrainground creates collisions with ground and itself
class Terrainground : public AbstractObstacle {

  double base_x, base_y, base_z;

  ImagePPM heightmap;

  bool obstacle_exists;

  dGeomID terrainZ;
  char *filename;
  int TERRAINNODES;  // only in one direction
  double *pTerrainHeights;

  double size;  // for both directions
  double height;
  int displayListNumber;
  int texture;

public:
  
  // Konstruktor
  Terrainground(const OdeHandle& odehandle, double size, double height, char *filename) 
    : AbstractObstacle::AbstractObstacle(odehandle)
  {
    this->height = height;
    this->size   = size;
    obstacle_exists=false;
    texture = 0;
    base_x=0.0;
    base_y=0.0;
    base_z=0.0;
    displayListNumber=-1;
    pTerrainHeights=0;


    if(!heightmap.loadImage(filename)) return;
    // image width and image height MUST be the same AND A POWER OF 2 ******************
    TERRAINNODES = heightmap.width();
    assert(heightmap.height() == TERRAINNODES);
    int bitcounter = 0;
    for(unsigned int i=0; i < sizeof(TERRAINNODES)*8; i++){
      bitcounter += (TERRAINNODES & (1<<i)) != 0;
    }
    assert(bitcounter==1);

    pTerrainHeights = new double[TERRAINNODES*TERRAINNODES];

    // copy and convert the heightmap from RGB chars to double
    for(int i=0; i< TERRAINNODES*TERRAINNODES; i++)     
      pTerrainHeights[i] = (heightmap.data()[i*3] / 255.0)*height;  // select only the RED channel of the picture and scale the height

    displayListNumber = dsCreateDisplayListTerrainZD(size, TERRAINNODES, pTerrainHeights);

	
  };

  virtual ~Terrainground()  
  {   
    destroy();
    if(pTerrainHeights) delete[] pTerrainHeights;
  }

  void setTextureID(int t)
  {   
    texture = t;
    setColor(1,1,1);
  }

  
  // draws the obstacle (terrain)   
  virtual void draw()
  {
    if(pTerrainHeights && displayListNumber >= 0) {
      dsSetColor (color.r, color.g, color.b);  // color settings have no effect on object if textured
      //     dsDrawTerrainZD( (int)base_x, (int)base_z, size, size/ (double)TERRAINNODES, TERRAINNODES, pTerrainHeights, dGeomGetRotation(terrainZ), dGeomGetPosition(terrainZ));
      dsSetTexture (texture,1);    
      dsCallList(displayListNumber, dGeomGetRotation(terrainZ), dGeomGetPosition(terrainZ));
    }
  };
  
  
  virtual void setPosition(double x, double y, double z){
    base_x = x;
    base_y = y;
    base_z = z;
    if (obstacle_exists) destroy();
    create();
  };

  virtual void getPosition(double& x, double& y, double& z){
    x = base_x;
    y = base_y;
    z = base_z;
  }
  
  virtual void setGeometry(double length_, double width_, double height_){
    size   = length_;
    height = height_;
  };

  //  virtual void setGeometry(double length_, double width_, double height_, double factorlength2_){
  //    length=length_;
  //    width=width_;
  //    height =height_;
  //    factorlength2=factorlength2_;
  //  };

  virtual void setColor(double r, double g, double b){
    color.r=r;
    color.g=g;
    color.b=b;
  };

protected:
  virtual void create(){
    if(!pTerrainHeights) return;
    //     pTerrainHeights = new double[TERRAINNODES*TERRAINNODES];

    // copy and convert the heightmap from RGB chars to double
    //     for(int i=0; i< TERRAINNODES*TERRAINNODES; i++)     
    //        pTerrainHeights[i] = (heightmap.data()[i*3] / 255.0)*height;  // select only the RED channel of the picture
     
    terrainZ = dCreateTerrainZ( space, pTerrainHeights, size, TERRAINNODES, 1, 1);
    dGeomSetPosition ( terrainZ, base_x, base_y, base_z);
	
    obstacle_exists=true;
  };


  virtual void destroy(){
    if(pTerrainHeights)
      dGeomDestroy( terrainZ );
    obstacle_exists=false;
  };

};

#endif

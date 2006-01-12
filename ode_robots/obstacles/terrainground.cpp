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
 *   Revision 1.2.4.1  2006-01-12 15:12:34  martius
 *   disabled for now
 *
 *   Revision 1.2  2005/10/28 12:06:10  martius
 *   *** empty log message ***
 *
 *   Revision 1.1  2005/10/25 22:22:46  martius
 *   moved implementation to cpp
 *   data constructor
 *   store method
 *   different heighmap codings
 *
 *                                                                 *
 ***************************************************************************/

// #include <stdio.h>
// #include <math.h>
// #include <assert.h>

// #include "terrainground.h"
// #include <drawstuff/drawstuff.h>
// #include "imageppm.h"

// double Terrainground::coding(CodingMode mode, const unsigned char* data){
//   switch(mode){
//   case Red:
//     return (data[0])/256.0;
//     break;
//   case Sum:
//     return (data[0] + data[1] + data[2])/(3*256.0);
//     break;
//   case LowMidHigh:
//     return ((long(data[0])  << 16) + (long(data[1]) << 8) + data[2])/65536.0;
//     break;
//   default:
//     return 0;
//   }
  
// }

// Terrainground::Terrainground(const OdeHandle& odehandle, double size, double height, char *filename,
// 			     CodingMode codingMode)
//   : AbstractObstacle::AbstractObstacle(odehandle)
// {
//   this->height = height;
//   this->size   = size;
//   obstacle_exists=false;
//   texture = 0;
//   base_x=0.0;
//   base_y=0.0;
//   base_z=0.0;
//   displayListNumber=-1;
//   pTerrainHeights=0;
//   texture=DS_NONE;  
  
//   if(!heightmap.loadImage(filename)) return;
//   // image width and image height MUST be the same AND A POWER OF 2 ******************
//   TERRAINNODES = heightmap.width();
//   assert(heightmap.height() == TERRAINNODES);
//   int bitcounter = 0;
//   for(unsigned int i=0; i < sizeof(TERRAINNODES)*8; i++){
//     bitcounter += (TERRAINNODES & (1<<i)) != 0;
//   }
//   assert(bitcounter==1);
  
//   pTerrainHeights = new double[TERRAINNODES*TERRAINNODES];
  
//   // copy and convert the heightmap from RGB chars to double
//   for(int i=0; i< TERRAINNODES*TERRAINNODES; i++){
//     // use the coding th get the height and scale it to height
//     pTerrainHeights[i] = coding(codingMode, heightmap.data() + i*3)*height;  
//   }
  
//   displayListNumber = dsCreateDisplayListTerrainZD(size, TERRAINNODES, pTerrainHeights);
  
// };


// Terrainground::~Terrainground()  
// {   
//   destroy();
//   if(pTerrainHeights) delete[] pTerrainHeights;
// }

  
// // draws the obstacle (terrain)   
// void Terrainground::draw()
// {
//   if(pTerrainHeights && displayListNumber >= 0) {
//     dsSetColor (color.r, color.g, color.b);  // color settings have no effect on object if textured
//     //     dsDrawTerrainZD( (int)base_x, (int)base_z, size, size/ (double)TERRAINNODES, TERRAINNODES, pTerrainHeights, dGeomGetRotation(terrainZ), dGeomGetPosition(terrainZ));
//     dsSetTexture (texture,1);    
//     dsCallList(displayListNumber, dGeomGetRotation(terrainZ), dGeomGetPosition(terrainZ));
//   }
// };


// void Terrainground::setTextureID(int t) {
//   texture = t; 
//   setColor(1,1,1);
// } 
  
  
// void Terrainground::setPosition(double x, double y, double z){
//   base_x = x;
//   base_y = y;
//   base_z = z;
//   if (obstacle_exists) destroy();
//   create();
// };

// void Terrainground::getPosition(double& x, double& y, double& z){
//   x = base_x;
//   y = base_y;
//   z = base_z;
// }
  
// void Terrainground::setGeometry(double length_, double width_, double height_){
//   size   = length_;
//   height = height_;
// };

// //  virtual void setGeometry(double length_, double width_, double height_, double factorlength2_){
// //    length=length_;
// //    width=width_;
// //    height =height_;
// //    factorlength2=factorlength2_;
// //  };

// void Terrainground::setColor(double r, double g, double b){
//   color.r=r;
//   color.g=g;
//   color.b=b;
// };


// void Terrainground::create(){
//   if(!pTerrainHeights) return;
     
//   terrainZ = dCreateTerrainZ( space, pTerrainHeights, size, TERRAINNODES, 1, 1);
//   dGeomSetPosition ( terrainZ, base_x, base_y, base_z);
	
//   obstacle_exists=true;
// };


// void Terrainground::destroy(){
//   if(pTerrainHeights)
//     dGeomDestroy( terrainZ );
//   obstacle_exists=false;
// };



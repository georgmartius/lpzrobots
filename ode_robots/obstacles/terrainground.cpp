// /***************************************************************************
//  *   Copyright (C) 2005 by Robot Group Leipzig                             *
//  *    martius@informatik.uni-leipzig.de                                    *
//  *    fhesse@informatik.uni-leipzig.de                                     *
//  *    der@informatik.uni-leipzig.de                                        *
//  *                                                                         *
//  *   This program is free software; you can redistribute it and/or modify  *
//  *   it under the terms of the GNU General Public License as published by  *
//  *   the Free Software Foundation; either version 2 of the License, or     *
//  *   (at your option) any later version.                                   *
//  *                                                                         *
//  *   This program is distributed in the hope that it will be useful,       *
//  *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
//  *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
//  *   GNU General Public License for more details.                          *
//  *                                                                         *
//  *   You should have received a copy of the GNU General Public License     *
//  *   along with this program; if not, write to the                         *
//  *   Free Software Foundation, Inc.,                                       *
//  *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
//  *                                                                         *
//  *   $Log$
//  *   Revision 1.2.4.2  2006-05-28 22:14:56  martius
//  *   heightfield included
//  *
//  *   Revision 1.2.4.1  2006/01/12 15:12:34  martius
//  *   disabled for now
//  *
//  *   Revision 1.2  2005/10/28 12:06:10  martius
//  *   *** empty log message ***
//  *
//  *   Revision 1.1  2005/10/25 22:22:46  martius
//  *   moved implementation to cpp
//  *   data constructor
//  *   store method
//  *   different heighmap codings
//  *
//  *                                                                 *
//  ***************************************************************************/

// #include <stdio.h>
// #include <math.h>
// #include <assert.h>

// #include "terrainground.h"
// #include "imageppm.h"

// namespace lpzrobots {

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
// 			     char* texture, CodingMode codingMode)
//   : AbstractObstacle::AbstractObstacle(odehandle)
// {
//   this->height = height;
//   this->size   = size;
//   obstacle_exists=false;

//   if(!heightmap.loadImage(filename)) return;
//   // image width and image height MUST be the same AND A POWER OF 2 ******************
//   cols = heightmap.width();
//   rows = heightmap.height();
//   assert(cols == rows);
//   // power of 2 check
//   int bitcounter = 0;
//   for(unsigned int i=0; i < sizeof(cols)*8; i++){
//     bitcounter += (width & (1<<i)) != 0;
//   }
//   assert(bitcounter==1);
  
//   heights = new double[rows*cols];
  
//   // copy and convert the heightmap from RGB chars to double
//   for(int i=0; i< rows*colls; i++){
//     // use the coding th get the height and scale it to height
//     heights[i] = coding(codingMode, heightmap.data() + i*3)*height;  
//   }
  
// };


// Terrainground::~Terrainground()  
// {   
//   destroy();
//   if(heights) delete[] heights;
// }

  

// void Terrainground::update()
// {
// };

  
// void Terrainground::setPose(const osg::Matrix& pose){
//   this->pose = pose
//   if (obstacle_exists) destroy();
//   create();
// };


// void Terrainground::create(){
//   if(!heights) return;
    
 
  
//   terrainZ = dCreateTerrainZ( space, pTerrainHeights, size, TERRAINNODES, 1, 1);
//   osg::Vec3 pos = pose.getTrans();
//   dGeomSetPosition(mesh, pos.x(), pos.y(), pos.z());
	
//   obstacle_exists=true;
// };


// void Terrainground::destroy(){
//   if(pTerrainHeights)
//     dGeomDestroy( terrainZ );
//   obstacle_exists=false;
// };


// }

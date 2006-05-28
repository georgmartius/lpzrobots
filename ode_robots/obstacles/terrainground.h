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
 *   Revision 1.7.4.2  2006-05-28 22:14:56  martius
 *   heightfield included
 *
 *   Revision 1.7.4.1  2006/01/12 15:12:34  martius
 *   disabled for now
 *
 *   Revision 1.7  2005/10/25 22:22:46  martius
 *   moved implementation to cpp
 *   data constructor
 *   store method
 *   different heighmap codings
 *
 *   Revision 1.6  2005/09/22 13:17:11  martius
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


#include "abstractobstacle.h"
#include "imageppm.h"

namespace lpzrobots {

class Terrainground : public AbstractObstacle {

  ImagePPM image;

  bool obstacle_exists;

  //  HeighField heighfield;
  char *filename;
  char *texture;

  double height;

public:
  /// height coding using in the read in bitmap.
  // Red: just the red channel is used;
  // Sum: the sum of all channels is used;
  // HighMidLow: Blue is least significant, Green is medium significant and Red is most significant
  typedef enum CodingMode {Red, Sum, LowMidHigh};
  
  /// Constructor
  // @param size size in world coordinates (in both directions)
  // @param height height in world coordinates
  Terrainground(const OdeHandle& odehandle, double x_size, double y_size, double height, char *filename, char* texture, CodingMode codingMode=Red);
  virtual ~Terrainground();
  
  virtual void update();
  
  virtual void setPose(const osg::Matrix& pose);

protected:
  virtual void create();

  virtual void destroy();

  // return the height using the giben coding mode. The data pointer points to RGB data point
  static double coding(CodingMode mode, const unsigned char* data);
  

};


}

#endif

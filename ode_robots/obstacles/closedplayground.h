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
 *   Revision 1.6  2007-03-16 11:01:37  martius
 *   abstractobstacle gets mor functionallity
 *   setSubstance
 *
 *   Revision 1.5  2006/09/12 08:23:13  robot8
 *   -corrected the wrong call for pos with getPosition ()
 *
 *   Revision 1.4  2006/08/11 15:41:04  martius
 *   playgrounds handle non-quadratic ground planes
 *
 *   Revision 1.3  2006/07/14 12:23:32  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.2.4.2  2006/05/11 08:59:15  robot3
 *   -fixed a positioning bug (e.g. for passivesphere)
 *   -some methods moved to abstractobstacle.h for avoiding inconsistencies
 *
 *   Revision 1.2.4.1  2006/01/10 20:25:08  martius
 *   moved to osg
 *
 *   Revision 1.2  2005/09/22 12:24:36  martius
 *   removed global variables
 *   OdeHandle and GlobalData are used instead
 *   sensor prepared
 *
 *   Revision 1.1  2005/07/21 12:01:54  robot8
 *   adding a new obstacle, which is similar to playground, but closed to all directions
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
#ifndef __CLOSEDPLAYGROUND_H
#define __CLOSEDPLAYGROUND_H


#include "playground.h"

namespace lpzrobots {

  class ClosedPlayground : public Playground {

  protected:
    Box* roof;

  public:
  
    ClosedPlayground(const OdeHandle& odeHandle, const OsgHandle& osgHandle , 
		     const osg::Vec3& dimension = osg::Vec3(7.0, 0.2, 0.5) , double factorxy = 1)
      : Playground(odeHandle, osgHandle, dimension, factorxy){
    };
    

  protected:
    virtual void create(){
      Playground::create();
      roof = new Box(length + 2 * width , (length * factorlength2) + 2 * width , width);
      roof->init(odeHandle, 0, osgHandle, Primitive::Geom | Primitive::Draw);

      roof->setPosition(getPosition () + osg::Vec3(0,0,height+width/2));
      obst.push_back(roof);
    };


    virtual void destroy(){
      Playground::destroy();
    }
  };

}

#endif

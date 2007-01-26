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
 *   Revision 1.2  2007-01-26 12:05:04  martius
 *   servos combinied into OneAxisServo
 *
 *   Revision 1.1  2006/09/20 12:57:26  martius
 *   snake with feet and body
 *
 *   Revision 1.7  2006/07/20 17:19:44  martius
 *   removed using namespace std from matrix.h
 *
 *   Revision 1.6  2006/07/14 12:23:41  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.5.4.7  2006/06/25 16:57:15  martius
 *   abstractrobot is configureable
 *   name and revision
 *
 *   Revision 1.5.4.6  2006/03/30 12:34:56  martius
 *   documentation updated
 *
 *   Revision 1.5.4.5  2006/02/01 18:33:40  martius
 *   use Axis type for Joint axis. very important, since otherwise Vec3 * pose is not the right direction vector anymore
 *
 *   Revision 1.5.4.4  2005/12/29 16:46:24  martius
 *   inherits from Schlange
 *   moved to osg
 *
 *   Revision 1.5.4.3  2005/11/16 11:26:53  martius
 *   moved to selforg
 *
 *   Revision 1.5.4.2  2005/11/15 12:29:27  martius
 *   new selforg structure and OdeAgent, OdeRobot ...
 *
 *   Revision 1.5.4.1  2005/11/14 17:37:18  martius
 *   moved to selforg
 *
 *   Revision 1.5  2005/11/09 13:24:42  martius
 *   added GPL
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __PLATFUSSSCHLANGESERVO
#define __PLATFUSSSCHLANGESERVO

#include "schlangeservo2.h"
#include "oneaxisservo.h"

namespace lpzrobots {

  /**
   * This is a class, which models a snake like robot with flat ends and a big body in the middle. 
   * It consists of a number of equal elements, each linked 
   * by a universal joint powered by 2 servos
   **/
  class PlattfussSchlange: public SchlangeServo2
  {

  public:
    PlattfussSchlange ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
			const SchlangeConf& conf, 
			const std::string& name);
    
    virtual ~PlattfussSchlange();
	
  private:
    virtual Primitive* createSegment(int index); 
  };

}

#endif

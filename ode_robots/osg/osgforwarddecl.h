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
 *  Declares some common osg classes in forward declaration.               *
 *  This saves a lot of compile time if included in header files           *
 *  instead of the real osg header files.                                  *
 *                                                                         *
 *   $Log$
 *   Revision 1.2  2006-07-14 12:23:35  martius
 *   selforg becomes HEAD
 *
 *   Revision 1.1.2.3  2006/01/12 14:21:00  martius
 *   drawmode, material
 *
 *   Revision 1.1.2.2  2005/12/14 15:36:45  martius
 *   joints are visible now
 *
 *   Revision 1.1.2.1  2005/12/13 18:11:13  martius
 *   transform primitive added, some joints stuff done, forward declaration
 *
 *   Revision 1.1.2.1  2005/12/06 17:38:21  martius
 *   *** empty log message ***
 *
 *                                                                 *
 ***************************************************************************/
#ifndef __OSGFORWARDDECL_H
#define __OSGFORWARDDECL_H

namespace osg{
  class Geode;
  class Group;

  class ShapeDrawable;
  class TessellationHints;
  class StateSet;
  class BlendFunc;
  class LightSource;

  class Texture2D;
  class TexGen;
  

  class Vec3f;
  typedef Vec3f Vec3;

  class Matrixd;
  typedef Matrixd Matrix;

  class Transform;
  class MatrixTransform;

}

#endif

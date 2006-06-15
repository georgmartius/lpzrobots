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
 *                                                                         *
 *   $Log$
 *   Revision 1.1.2.7  2006-06-15 09:16:59  martius
 *   restores changeAlpha
 *
 *   Revision 1.1.2.6  2006/06/12 13:40:02  robot3
 *   changeAlpha is now a dummy method (for fast implement issues)
 *
 *   Revision 1.1.2.5  2006/06/12 13:37:12  robot3
 *   added missing const OsgHandle->changeAlpha(const float& alpha);
 *
 *   Revision 1.1.2.4  2006/01/12 14:39:06  martius
 *   transparent stateset
 *
 *   Revision 1.1.2.3  2005/12/29 16:48:06  martius
 *   changeColor
 *
 *   Revision 1.1.2.2  2005/12/22 14:10:14  martius
 *   different tesselhints
 *
 *   Revision 1.1.2.1  2005/12/13 18:12:20  martius
 *   some utils
 *
 *
 *                                                                 *
 ***************************************************************************/


#include <osg/Group>
#include <osg/ShapeDrawable>
#include "osghandle.h"

namespace lpzrobots {

  OsgHandle::OsgHandle( ) {}
  OsgHandle::OsgHandle( osg::Group* scene, osg::TessellationHints* tesselhints[3], 
			osg::StateSet* normalState, osg::StateSet* transparentState, 
			const Color& color){
    this->scene = scene;
    for(int i=0; i<3; i++){
      this->tesselhints[i] = tesselhints[i];
    }
    this->normalState = normalState;
    this->transparentState = transparentState;
    this->color = color;
  }

  OsgHandle OsgHandle::changeColor(const Color& color) const {
    OsgHandle copy(*this);
    copy.color = color;
    return copy;
  }

  OsgHandle OsgHandle::changeAlpha(double alpha) const {
    OsgHandle copy(*this);
    copy.color.alpha() = alpha;
    return copy;
  }
}

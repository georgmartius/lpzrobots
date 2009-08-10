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
 *   Revision 1.4  2009-08-10 07:48:37  guettler
 *   fixed bug in if statement
 *
 *   Revision 1.3  2009/03/13 09:19:53  martius
 *   changed texture handling in osgprimitive
 *   new OsgBoxTex that supports custom texture repeats and so on
 *   Box uses osgBoxTex now. We also need osgSphereTex and so on.
 *   setTexture has to be called before init() of the primitive
 *
 *   Revision 1.2  2006/09/21 16:17:18  der
 *   *** empty log message ***
 *
 *   Revision 1.1  2006/09/20 12:57:21  martius
 *   snake with feet and body
 *
 *
 ***************************************************************************/

#include "plattfussschlange.h"
using namespace std;

namespace lpzrobots {

  PlattfussSchlange:: 
  PlattfussSchlange ( const OdeHandle& odeHandle, const OsgHandle& osgHandle,
		      const SchlangeConf& conf, const std::string& name)	      
    : SchlangeServo2(odeHandle, osgHandle, conf, name, 
		     "$Id$") 
  {
    
  }
	
  PlattfussSchlange::~PlattfussSchlange() { }
	

  Primitive* PlattfussSchlange::createSegment(int index){
    Primitive* p;

    /////////// MIDDLE SEGMENT (BODY)
    if ( index*2 == conf.segmNumber-1) {
      //p = new Box(conf.segmLength*1.5,conf.segmLength*1.5, conf.segmLength*.6);
      //p = new Capsule(conf.segmDia*.8/*2.8*/ , conf.segmLength*1); 

       //  p = new Capsule(conf.segmLength/2 , conf.segmLength*2);
      p = new Sphere(conf.segmLength*.8);
      p->setTexture("Images/wood.rgb");
      p->init(odeHandle, conf.segmMass*2, osgHandle);      
      // p->setPose( osg::Matrix::rotate(M_PI/2, 0, 1, 0)*osg::Matrix::translate( conf.segmDia, 0, 0) );      
    } /////// FEED
    else if( (index == 0) | (index== conf.segmNumber-1)) {
      // p = new Capsule(conf.segmDia*.8/*2.8*/ , conf.segmLength*1); 
       // p = new Sphere(conf.segmLength/2*2);
      p = new Box(1.8*conf.segmLength,3*conf.segmLength, conf.segmLength*.3);
      p->setTexture("Images/whitemetal_farbig_small.rgb");
      p->init(odeHandle, conf.segmMass*3, osgHandle);	      
    } /////// NORMAL SEGMENT
    else {	
      p = SchlangeServo2::createSegment(index);
    }  
    return p;
  }

}

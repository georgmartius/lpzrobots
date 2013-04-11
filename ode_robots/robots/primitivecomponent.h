/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    marcel@informatik.uni-leipzig.de                                     *
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
 ***************************************************************************/
#ifndef __PRIMTIVECOMPONENT_h
#define __PRIMTIVECOMPONENT_h


#include "components.h"


namespace lpzrobots
{

/**
 * Component consisting of one Primitive
 *
 *
 */
class PrimitiveComponent : public Component
{
 public:

  PrimitiveComponent( Primitive* p, const OdeHandle &odeHandle, const OsgHandle &osgHandle, const ComponentConf& conf);

  ~PrimitiveComponent ();

 public:

  virtual void         update ();
  virtual void         place (const Pos &pos);
  virtual void         place (const osg:Matrix&);


  // virtual void         setColor (const Color &col);         sets color of the robot; not nessecary

  virtual Position getPosition () const; //returns position of the object; relates to the robot or Primitive belonging to the component

  /**
   *return reference to the simple Primitive, or to the main Primitive of the robot assigend to the component. If nothimng is assigned, NULL is returned.
   **/
  virtual Primitive* getMainPrimitive () const;

 protected:
  Primitive* primitive;

};

}
#endif

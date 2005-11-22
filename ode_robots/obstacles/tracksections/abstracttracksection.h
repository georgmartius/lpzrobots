/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *   Revision 1.2  2005-11-22 15:51:23  robot3
 *   testing
 *
 *   Revision 1.1  2005/11/15 14:29:25  robot3
 *   first version
 *
 *                                                                         *
 ***************************************************************************/
#ifndef __ABSTRACTTRACKSECTION_H
#define __ABSTRACTTRACKSECTION_H

#include "matrix.h"
using namespace matrix;
#include "position.h"
#include <drawstuff/drawstuff.h>
#include "simulation.h"

/**
 *  Abstract class (interface) for obstacles
 */
class AbstractTrackSection{

 public:
  /**
   * Constructor, segment is initialized with Position (0,0,0)
   * and a rotation angle=0
   */
  AbstractTrackSection() {};

  /**
   * Constructor where you can set the position and rotation by:
   @param Position& p is the position of the segment
   @param double angle is the rotation of the segment
   */
  AbstractTrackSection(const Position& p,const double angle) {};

  /**
   * Constructor where you can set the pos-matrix by this constructor:
   @param Matrix& position is the position AND rotation of the segment
   */
  AbstractTrackSection(const Matrix& position)
    : pos(position) {
  };

  virtual ~AbstractTrackSection(){}
  
  /**
   * sets position of the obstacle and creates/recreates obstacle if necessary
   */
  virtual void setPosition(const Position& p) = 0;

  /**
   * gives actual position of the obstacle
   */
  virtual Position getPosition() = 0;

  virtual Matrix getPositionMatrix()=0;


  /**
   * sets rotation of the abstract segment and creates/recreates abstract line if necessary
   */
  virtual void setRotation(const Matrix& m) = 0;

  virtual void create(dSpaceID space) = 0;


  virtual void destroy() = 0;

  virtual void draw() = 0;

  /**
   * gives actual rotation of the abstract segment
   */
  virtual Matrix getRotation() = 0;


  /**
   * gives the position and rotation(angle) of the segment at the
   * end of the segment so that a new segment could be placed there
   * the result is a matrix
   */
  virtual Matrix getTransformedEndMatrix() = 0;

  /**
   * returns true if the real coordinates lay inside of the segment
   */
  virtual bool isInside(const Position& p) = 0;

  /**
 * returns a value between 0 and 100 that tells at which section
 * you are on the segment.
 * 0 means you are on the beginning
 * 100 means you are at the end
 * returns -1 if no IdValue can be given
 */
  virtual double getSectionIdValue(const Position& p)=0;


/**
 * returns a value between 0 and 100 that tells at which width
 * you are on the segment, more to right or more to the left.
 * 0 means you are on the left
 * 50 means you are in the middle
 * 100 means you are on the right
 * returns -1 if no WidthValue can be given
 */
virtual double getWidthIdValue(const Position& p)=0;



/**
 * returns the length of the segment,
 * here it is the length of the arc
 * formula is: radius * angle;
 */
 virtual double getLength()=0;


/**
 * returns the width of the segment,
 */
 virtual double getWidth()=0;

/**
 * sets the width of the segment,
 */
 virtual double setWidth(double w)=0;

 protected:
  // saves the actual position AND rotation of the segment
  Matrix pos;
};

#endif

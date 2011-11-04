/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
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
 ***************************************************************************/
#ifndef __DEGREESEGMENT_H
#define __DEGREESEGMENT_H

#include "abstracttracksection.h"

/**
 *  class for degree segments
 */
class DegreeSegment : public AbstractTrackSection {

 public:

  /**
   * Constructor
   */
  DegreeSegment(const Position& p,const double& angle);


  /**
   * Constructor
   */
  DegreeSegment(const Matrix& position);

  virtual ~DegreeSegment(){}


/**
 * returns the length of the segment,
 * here it is the length of the arc
 */

virtual double getLength();

/**
 * returns the width of the segment,
 */
 virtual double getWidth();

/**
 * sets the width of the segment,
 */
 virtual void setWidth(double w);


virtual void setCurveAngle(const double& alpha);

virtual void setRadius(const double& rad);

  /**
   * gives the position and rotation(angle) of the segment at the
   * end of the segment so that a new segment could be placed there
   * if you want to place the new segment, you must muliplicate:
   * getTransformedEndMatrix()*getPositionMatrix();
   */
  virtual Matrix getTransformedEndMatrix();


/**
 * returns true if the real coordinates lay inside of the segment
 */
virtual bool isInside(const Position& p);


/**
 * returns a value between 0 and 100 that tells at which section
 * you are on the segment.
 * 0 means you are on the beginning
 * 100 means you are at the end
 * returns -1 if no IdValue can be given
 */
virtual double getSectionIdValue(const Position& p);


/**
 * returns a value between 0 and 100 that tells at which width
 * you are on the segment, more to right or more to the left.
 * 0 means you are on the left
 * 50 means you are in the middle
 * 100 means you are on the right
 * returns -1 if no WidthValue can be given
 */
virtual double getWidthIdValue(const Position& p);


/**
 * draws the obstacle (4 boxes for the playground)
 */
 virtual  void draw();



virtual void create(dSpaceID space);


virtual  void destroy();

 protected:
  // this is the radius of the curve
  double radius;
  // this is the width of the segment
  // normally it should be the same like alle the other segments
  double width;

  bool show_aabb;

  // the wall to be drawed
  list<dGeomID> innerWalls;
  list<dGeomID> outerWalls;

  double widthWall;
  double heightWall;

  // angle is for 90DegreeSegment is 90
  double angle;

  // determines if the curve goes left or right
  int left;

  bool obstacle_exists;

  /**
   * obstacle color
   */
  Color color;
 
  void setProperties();

  /**
   * returns the local coordinates  on the track at the given radius and angle
   that are responsible for the segment of the
   */
  Position getLocalCoordinates(double radius, double alpha);

  /**
   * returns the global coordinates on the track at the given radius and angle
   that are responsible for the segment of the
   */
  Position getGlobalCoordinates(double radius, double alpha);

};

#endif



